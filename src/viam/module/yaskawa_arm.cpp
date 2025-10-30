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
#include <fstream>
#include <future>
#include <iomanip>
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

pose cartesian_position_to_pose(CartesianPosition&& pos) {
    const double norm = std::hypot(degrees_to_radians(pos.rx), degrees_to_radians(pos.ry), degrees_to_radians(pos.rz));
    if (std::isnan(norm) || (norm == 0)) {
        throw std::invalid_argument("Cannot normalize with NaN or zero norm");
    }
    auto q = std::unique_ptr<void, decltype(&free_quaternion_memory)>(
        quaternion_from_axis_angle(
            degrees_to_radians(pos.rx) / norm, degrees_to_radians(pos.ry) / norm, degrees_to_radians(pos.rz) / norm, norm),
        &free_quaternion_memory);
    auto ov = std::unique_ptr<void, decltype(&free_orientation_vector_memory)>(orientation_vector_from_quaternion(q.get()),
                                                                               &free_orientation_vector_memory);

    auto components = std::unique_ptr<double[], decltype(&free_orientation_vector_components)>(orientation_vector_get_components(ov.get()),
                                                                                               &free_orientation_vector_components);
    auto position = coordinates{pos.x, pos.y, pos.z};
    auto orientation = pose_orientation{components[0], components[1], components[2]};
    auto theta = radians_to_degrees(components[3]);

    return {position, orientation, theta};
}

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
        throw std::invalid_argument(
            std::format("attribute `reject_move_request_threshold_rad` should be between {} and {} , it is : {} radians",
                        k_min_threshold,
                        k_max_threshold,
                        *threshold));
    }

    auto group_id = find_config_attribute<double>(cfg, "group_index");
    constexpr int k_min_group_number = 0;
    constexpr int k_max_group_number = 2; // only 3 arms total for the control box
    if (group_id && (*group_id < k_min_group_number || *group_id > k_max_group_number || floor(*group_id) != *group_id)){
        throw std::invalid_argument(
            std::format("attribute `group_index` should be a whole number between {} and {} , it is : {}",
                        k_min_group_number,
                        k_max_group_number,
                        *group_id));
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

    resource_root_ = std::filesystem::canonical(module_executable_directory / k_relpath_bindir_to_datadir / "yaskawa-robots");
    VIAM_SDK_LOG(info) << "Yaskawa robots module executable found in `" << module_executable_path << "; resources will be found in `"
                       << resource_root_ << "`";

    threshold_ = find_config_attribute<double>(config, "reject_move_request_threshold_rad");

    auto speed = find_config_attribute<double>(config, "speed_rad_per_sec").value();
    auto acceleration = find_config_attribute<double>(config, "acceleration_rad_per_sec2").value();
    auto group_id = find_config_attribute<double>(config, "group_index");

    robot_ = std::make_shared<YaskawaController>(io_context_, speed, acceleration, std::move(group_id), host);

    constexpr int k_max_connection_try = 5;
    int connection_try = 0;

    // Attempt connection with retry logic
    while (connection_try < k_max_connection_try) {
        try {
            ++connection_try;
            robot_->connect().get();
            VIAM_SDK_LOG(info) << "Successfully connected to robot on attempt " << connection_try;
            break;  // Exit on successful connection
        } catch (std::exception& ex) {
            VIAM_SDK_LOG(error) << std::format(
                "connection {} out of {} failed because {}", connection_try, k_max_connection_try, ex.what());
            if (k_max_connection_try == connection_try)
                throw;
        }
    }
    if (!CheckGroupMessage(robot_->checkGroupIndex().get()).is_valid){
        robot_->disconnect();
        std::ostringstream buffer;
        buffer << std::format("group_index {} is not available on the arm", robot_->get_group_index());
        throw std::invalid_argument(buffer.str());
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

        // Convert joint positions from degrees to radians and filter duplicate waypoints
        for (const auto& position : positions) {
            auto next_waypoint_deg = Eigen::VectorXd::Map(position.data(), boost::numeric_cast<Eigen::Index>(position.size()));
            auto next_waypoint_rad = degrees_to_radians(std::move(next_waypoint_deg)).eval();

            // Skip waypoints that are too close to the previous one to avoid redundant motion
            if ((!waypoints.empty()) && (next_waypoint_rad.isApprox(waypoints.back(), k_waypoint_equivalancy_epsilon_rad))) {
                continue;
            }
            waypoints.emplace_back(std::move(next_waypoint_rad));
        }

        const auto unix_time = unix_time_iso8601();

        // Execute the move command and block until completion
        // This will throw if an error occurs during motion
        robot_->move(std::move(waypoints), unix_time)->wait();
    }
}

void YaskawaArm::move_to_joint_positions(const std::vector<double>& positions, const ProtoStruct&) {
    auto next_waypoint_deg = Eigen::VectorXd::Map(positions.data(), boost::numeric_cast<Eigen::Index>(positions.size())).eval();
    auto next_waypoint_rad = degrees_to_radians(std::move(next_waypoint_deg));
    std::list<Eigen::VectorXd> waypoints;
    waypoints.emplace_back(std::move(next_waypoint_rad));

    const auto unix_time = unix_time_iso8601();

    // move will throw if an error occurs
    robot_->move(std::move(waypoints), unix_time)->wait();
}

YaskawaArm::KinematicsData YaskawaArm::get_kinematics(const ProtoStruct&) {
    const std::shared_lock rlock{config_mutex_};

    const auto sva_file_path = resource_root_ / "kinematics/gp12.json";

    // Open the file in binary mode
    std::ifstream sva_file(sva_file_path, std::ios::binary);
    if (!sva_file) {
        throw std::runtime_error(
        boost::str(boost::format("unable to open kinematics file '%1%'") % sva_file_path.string()));
    }

    // Read the entire file into a vector without computing size ahead of time
    std::vector<char> temp_bytes(std::istreambuf_iterator<char>(sva_file), {});
    if (sva_file.bad()) {
        throw std::runtime_error(
        boost::str(boost::format("error reading kinematics file '%1%'") % sva_file_path.string()));
    }

    // Convert to unsigned char vector
    return KinematicsDataSVA({temp_bytes.begin(), temp_bytes.end()});
}

pose YaskawaArm::get_end_position(const ProtoStruct&) {
    return cartesian_position_to_pose(CartesianPosition(robot_->getCartPosition().get()));
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
