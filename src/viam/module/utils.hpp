#pragma once

#include <chrono>
#include <mutex>
#include <optional>
#include <sstream>
#include <string_view>
#include <vector>

#include <boost/variant.hpp>

#include <Eigen/Dense>
#include <viam/lib/logger.hpp>
#include <viam/lib/robot_socket.hpp>
#include <viam/sdk/components/arm.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/log/logging.hpp>

/// Logger implementation that writes to VIAM SDK logging infrastructure
class ViamSdkLogger final : public viam::yaskawa::ILogger {
   public:
    explicit ViamSdkLogger(viam::yaskawa::LogLevel min_level = viam::yaskawa::LogLevel::INFO) {
        set_min_level(min_level);
    }

    ~ViamSdkLogger() final = default;

    ViamSdkLogger(const ViamSdkLogger&) = delete;
    ViamSdkLogger& operator=(const ViamSdkLogger&) = delete;
    ViamSdkLogger(ViamSdkLogger&&) noexcept = delete;
    ViamSdkLogger& operator=(ViamSdkLogger&&) noexcept = delete;

    void set_min_level(viam::yaskawa::LogLevel level) final {
        const std::lock_guard<std::mutex> lock{mutex_};
        using namespace viam::sdk;
        switch (level) {
            case viam::yaskawa::LogLevel::DEBUG:
                viam::sdk::LogManager::get().set_global_log_level(log_level::debug);
                break;
            case viam::yaskawa::LogLevel::INFO:
                viam::sdk::LogManager::get().set_global_log_level(log_level::info);
                break;
            case viam::yaskawa::LogLevel::WARNING:
                viam::sdk::LogManager::get().set_global_log_level(log_level::warn);
                break;
            case viam::yaskawa::LogLevel::ERROR:
            case viam::yaskawa::LogLevel::CRITICAL:
                viam::sdk::LogManager::get().set_global_log_level(log_level::error);
                break;
        }

        min_level_ = level;
    }

    viam::yaskawa::LogLevel get_min_level() const noexcept final {
        const std::lock_guard<std::mutex> lock{mutex_};
        return min_level_;
    }

   protected:
    void write_log(viam::yaskawa::LogLevel level, std::string_view message) final {
        if (level < get_min_level()) {
            return;
        }

        switch (level) {
            case viam::yaskawa::LogLevel::DEBUG:
                VIAM_SDK_LOG(debug) << message;
                break;
            case viam::yaskawa::LogLevel::INFO:
                VIAM_SDK_LOG(info) << message;
                break;
            case viam::yaskawa::LogLevel::WARNING:
                VIAM_SDK_LOG(warn) << message;
                break;
            case viam::yaskawa::LogLevel::ERROR:
            case viam::yaskawa::LogLevel::CRITICAL:
                VIAM_SDK_LOG(error) << message;
                break;
        }
    }

   private:
    mutable std::mutex mutex_;
    viam::yaskawa::LogLevel min_level_;
};

/// Configure the global logger for the yaskawa library to use VIAM SDK logging.
void configure_logger(const viam::sdk::ResourceConfig& cfg);

// helper function to extract an attribute value from its key within a
// ResourceConfig
template <class T>
std::optional<T> find_config_attribute(const viam::sdk::ResourceConfig& cfg, const std::string& attribute) {
    auto key = cfg.attributes().find(attribute);
    if (key == cfg.attributes().end()) {
        return std::nullopt;
    }
    const auto* const val = key->second.get<T>();
    if (!val) {
        std::ostringstream buffer;
        buffer << "attribute `" << attribute << " could not be converted to the required type";
        throw std::invalid_argument(buffer.str());
    }
    return std::make_optional(*val);
}

Eigen::Vector3d transform_vector(const Eigen::Vector3d& vector, const Eigen::Matrix3d& rotation_matrix);

// Generate ISO8601 timestamp string with microsecond precision
std::string unix_time_iso8601();

// Validates a config attribute is a strictly positive scalar or array of positive doubles.
// Returns the dimension of the element
// Throws if absent, wrong type, empty, zero, or negative.
size_t validate_joint_limit_attribute(const viam::sdk::ResourceConfig& cfg, const std::string& attribute);

// Apply a per-move limit override from MoveOptions to an Eigen limits vector.
// Scalar: fills all joints uniformly. Throws if value <= 0.
// Vector: sets per-joint limits. Requires matching DOF, all non-negative.
void apply_move_limit(Eigen::VectorXd& limits, const boost::variant<double, std::vector<double>>& value);

// Default DOF when both velocity and acceleration are scalars.
constexpr Eigen::Index k_default_dof = 6;

// Determines the configured number of DOF from two config attributes (velocity and acceleration).
// If both are scalars, returns k_default_dof. If one is an array, returns its size.
Eigen::Index number_of_dof_configured(const viam::sdk::ResourceConfig& config, const std::string& attr_a, const std::string& attr_b);

// Reads a validated config attribute (scalar or array of doubles) into an Eigen::VectorXd.
// Scalars are broadcast to target_dof elements. Arrays must match target_dof exactly.
Eigen::VectorXd read_limit_vector(const viam::sdk::ResourceConfig& config, const std::string& attribute, Eigen::Index target_dof);

// Converts an sdk trajectory_point into the trajectory_point_t we put on the wire.
//
// The sdk gives us every joint value in degrees and the time as a microsecond offset from the
// start of the stream. The controller wants radians and a duration_t split into whole seconds plus
// nanoseconds. Both sides measure time from the start of the trajectory, so we do not have to keep
// a running total and each point converts on its own.
//
// Streamed points come in already time parameterized, so this path skips trajex and does not
// recheck the configured speed_rad_per_sec or acceleration_rad_per_sec2. The caller picked the
// profile. The controller still checks the first point's speed against its per axis limits and the
// first position against START_MAX_PULSE_DEVIATION, so a stream the arm cannot reach gets rejected
// when we open the goal instead of being run.
//
// Throws std::invalid_argument if the point has no constraints, since we need velocities, or if
// any of its joint vectors is not dof long.
trajectory_point_t convert_streamed_point(const viam::sdk::Arm::trajectory_point& point, std::size_t dof);

// Converts a whole batch of streamed points and keeps them in order. Throws std::invalid_argument
// if the protocol cannot carry dof joints, or for any reason convert_streamed_point does.
std::vector<trajectory_point_t> convert_streamed_batch(const std::vector<viam::sdk::Arm::trajectory_point>& batch, std::size_t dof);

// Rejects a batch whose points are closer together than the controller can interpolate. The
// controller moves forward by at least one interpolation period every cycle, so it cannot run
// points that are closer together than that. It takes them faster than real time instead, and the
// arm runs a trajectory the caller did not ask for. Pass the time of the last point we accepted in
// through previous, so we check the gap between two batches as well as the gaps inside one, and we
// set it to this batch's last point before returning. An empty batch leaves it alone.
//
// Throws std::invalid_argument saying which gap was too small.
void check_streamed_spacing(const std::vector<viam::sdk::Arm::trajectory_point>& batch,
                            std::chrono::microseconds interpolation_period,
                            std::optional<std::chrono::microseconds>& previous);
