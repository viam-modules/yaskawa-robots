#include "yaskawa_arm.hpp"
#include "yaskawa_arm_config.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <boost/asio/io_context.hpp>
#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <format>
#include <future>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/dll/runtime_symbol_info.hpp>
#include <boost/format.hpp>
#include <boost/io/ostream_joiner.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm.hpp>

#include <viam/lib/robot_socket.hpp>
#include <viam/sdk/common/pose.hpp>
#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/module/module.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/resource.hpp>

#include <third_party/trajectories/Trajectory.h>

#include "utils.hpp"

// this chunk of code uses the rust FFI to handle the spatialmath calculations to turn a UR vector to a pose
extern "C" void* quaternion_from_axis_angle(double x, double y, double z, double theta);
extern "C" void free_quaternion_memory(void* q);

extern "C" void* orientation_vector_from_quaternion(void* q);
extern "C" void free_orientation_vector_memory(void* ov);

extern "C" double* orientation_vector_get_components(void* ov);
extern "C" void free_orientation_vector_components(double* ds);

namespace {

constexpr double k_waypoint_equivalancy_epsilon_rad = 1e-4;

std::string unix_time_iso8601() {
    namespace chrono = std::chrono;
    std::stringstream stream;

    const auto now = chrono::system_clock::now();
    const auto seconds_part = chrono::duration_cast<chrono::seconds>(now.time_since_epoch());
    const auto tt = chrono::system_clock::to_time_t(chrono::system_clock::time_point{seconds_part});
    const auto delta_us = chrono::duration_cast<chrono::microseconds>(now.time_since_epoch() - seconds_part);

    struct tm buf;
    auto* ret = gmtime_r(&tt, &buf);
    if (ret == nullptr) {
        throw std::runtime_error("failed to convert time to iso8601");
    }
    stream << std::put_time(&buf, "%FT%T");
    stream << "." << std::setw(6) << std::setfill('0') << delta_us.count() << "Z";

    return stream.str();
}

}  // namespace

std::vector<std::string> validate_config_(const ResourceConfig& cfg) {
    if (!find_config_attribute<std::string>(cfg, "host")) {
        throw std::invalid_argument("attribute `host` is required");
    }
    if (!find_config_attribute<double>(cfg, "speed_rad_per_sec")) {
        throw std::invalid_argument("attribute `speed_rad_per_sec` is required");
    }
    if (!find_config_attribute<double>(cfg, "acceleration_rad_per_sec2")) {
        throw std::invalid_argument("attribute `acceleration_rad_per_sec2` is required");
    }

    auto threshold = find_config_attribute<double>(cfg, "reject_move_request_threshold_rad");
    constexpr double k_min_threshold = 0.0;
    constexpr double k_max_threshold = 2 * M_PI;
    if (threshold && (*threshold < k_min_threshold || *threshold > k_max_threshold)) {
        std::stringstream sstream;
        sstream << "attribute `reject_move_request_threshold_rad` should be between " << k_min_threshold << " and " << k_max_threshold
                << ", it is : " << *threshold << " degrees";
        throw std::invalid_argument(
            std::format("attribute `reject_move_request_threshold_rad` should be between {} and {} , it is : {} radians",
                        k_min_threshold,
                        k_max_threshold,
                        *threshold));
    }

    return {};
}

const ModelFamily& YaskawaArm::model_family() {
    static const auto family = ModelFamily{"viam", "yaskawa-robots"};
    return family;
}

Model YaskawaArm::model(std::string model_name) {
    return {model_family(), std::move(model_name)};
}

std::vector<std::shared_ptr<ModelRegistration>> YaskawaArm::create_model_registrations(boost::asio::io_context& io_context) {
    const auto model_strings = {
        "gp12",
    };

    const auto arm = API::get<Arm>();
    const auto registration_factory = [&](auto m) {
        const auto model = YaskawaArm::model(m);
        return std::make_shared<ModelRegistration>(
            arm,
            model,
            // NOLINTNEXTLINE(performance-unnecessary-value-param): Signature is fixed by ModelRegistration.
            [&, model](auto deps, auto config) { return std::make_shared<YaskawaArm>(model, deps, config, io_context); },
            [](auto const& config) { return validate_config_(config); });
    };

    auto registrations = model_strings | boost::adaptors::transformed(registration_factory);
    return {std::make_move_iterator(begin(registrations)), std::make_move_iterator(end(registrations))};
}

YaskawaArm::YaskawaArm(Model model, const Dependencies& deps, const ResourceConfig& cfg, boost::asio::io_context& io_context)
    : Arm(cfg.name()), model_(std::move(model)), io_context_(io_context) {
    VIAM_SDK_LOG(info) << "Yaskawa Arm constructor called (model: " << model_.to_string() << ")";

    configure_(deps, cfg);
}

void YaskawaArm::configure_(const Dependencies&, const ResourceConfig& config) {
    VIAM_SDK_LOG(info) << "Yaskawa arm  starting up";
    auto host = find_config_attribute<std::string>(config, "host").value();

    const auto module_executable_path = boost::dll::program_location();
    const auto module_executable_directory = module_executable_path.parent_path();

    auto resource_root = std::filesystem::canonical(module_executable_directory / k_relpath_bindir_to_datadir / "universal-robots");
    VIAM_SDK_LOG(info) << "Universal robots module executable found in `" << module_executable_path << "; resources will be found in `"
                       << resource_root << "`";

    threshold_ = find_config_attribute<double>(config, "reject_move_request_threshold_rad");

    auto speed = find_config_attribute<double>(config, "speed_rads_per_sec").value();
    auto acceleration = find_config_attribute<double>(config, "acceleration_rads_per_sec").value();

    robot_ = std::make_unique<YaskawaController>(io_context_, speed, acceleration, host);

    constexpr int k_max_connection_try = 5;
    int connection_try = 0;

    while (1) {
        try {
            ++connection_try;
            robot_->connect().get();
        } catch (std::exception& ex) {
            VIAM_SDK_LOG(error) << std::format(
                "connection {} out of {} failed because {}", connection_try, k_max_connection_try, ex.what());
            if (k_max_connection_try == connection_try)
                throw;
        }
    }
}
void YaskawaArm::reconfigure(const Dependencies& deps, const ResourceConfig& cfg) {
    const std::unique_lock wlock{config_mutex_};
    VIAM_SDK_LOG(warn) << "Reconfigure called: configuring new state";
    configure_(deps, cfg);
    VIAM_SDK_LOG(info) << "Reconfigure completed OK";
}

std::vector<double> YaskawaArm::get_joint_positions(const ProtoStruct&) {
    const std::shared_lock rlock{config_mutex_};
    auto joint_rads = StatusMessage(robot_->get_robot_position_velocity_torque().get());
    auto joint_position_degree = joint_rads.position | boost::adaptors::transformed(radians_to_degrees<const double&>);
    return {std::begin(joint_position_degree), std::end(joint_position_degree)};
}

void YaskawaArm::move_through_joint_positions(const std::vector<std::vector<double>>& positions,
                                              const MoveOptions&,
                                              const viam::sdk::ProtoStruct&) {
    std::shared_lock rlock{config_mutex_};

    if (!positions.empty()) {
        std::list<Eigen::VectorXd> waypoints;
        for (const auto& position : positions) {
            auto next_waypoint_deg = Eigen::VectorXd::Map(position.data(), boost::numeric_cast<Eigen::Index>(position.size()));
            auto next_waypoint_rad = degrees_to_radians(std::move(next_waypoint_deg)).eval();
            if ((!waypoints.empty()) && (next_waypoint_rad.isApprox(waypoints.back(), k_waypoint_equivalancy_epsilon_rad))) {
                continue;
            }
            waypoints.emplace_back(std::move(next_waypoint_rad));
        }

        const auto unix_time = unix_time_iso8601();

        // move will throw if an error occurs
        robot_->move(std::move(waypoints), unix_time)->wait();
    }
}

void YaskawaArm::move_to_joint_positions(const std::vector<double>&, const ProtoStruct&) {
    throw std::runtime_error(std::format("move-to_joint_position is unimplemented for arm model {}", model_.to_string()));
}

YaskawaArm::KinematicsData YaskawaArm::get_kinematics(const ProtoStruct&) {
    throw std::runtime_error(std::format("get_kinematics is unimplemented for arm model {}", model_.to_string()));
}

pose YaskawaArm::get_end_position(const ProtoStruct&) {
    throw std::runtime_error(std::format("get_end_position is unimplemented for arm model {}", model_.to_string()));
}

void YaskawaArm::stop(const ProtoStruct&) {
    robot_->stop().get();
}

ProtoStruct YaskawaArm::do_command(const ProtoStruct&) {
    ProtoStruct resp = ProtoStruct{};
    return resp;
}

YaskawaArm::~YaskawaArm() {
    robot_->disconnect();
}

bool YaskawaArm::is_moving() {
    return RobotStatusMessage(robot_->get_robot_status().get()).in_motion;
}
