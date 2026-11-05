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

// Returns the SVA kinematics document with per-joint max_velocity and max_acceleration added,
// so the motion service can see what this arm instance will actually honor rather than position
// bounds alone. The limits are in radians, matching what we read from config, and are written as
// the degrees the SVA schema uses.
//
// The two vectors are indexed in the document's own joints order, since the config gives us
// positional arrays and no joint names. They must both be exactly as long as that array, and a
// mismatch throws rather than patching the joints we happen to have values for, because a config
// whose DOF disagrees with the model is not describing this arm.
//
// Throws std::invalid_argument if the document is not parseable, is not SVA, has no joints
// array, if either vector's length disagrees with the joint count, or if any joint is not an
// independent revolute joint or already carries a limit of its own.
std::string sva_with_joint_limits(const std::string& sva_json,
                                  const Eigen::VectorXd& velocity_rad_per_sec,
                                  const Eigen::VectorXd& acceleration_rad_per_sec2);

// Reads `stream_limit_tolerance_ratio` from config. Returns std::nullopt if it is absent.
// Throws std::invalid_argument if it is not a number or is less than 1.
std::optional<double> parse_stream_limit_tolerance_ratio(const viam::sdk::ResourceConfig& cfg);

// Per-joint bounds that streamed trajectory points are checked against. They are held in the
// degrees the sdk points carry, so the check needs no conversion and its errors read in the units
// the client sent. Both vectors are one entry per joint, and their length is the arm's dof.
struct streamed_limits {
    Eigen::VectorXd velocity_degs_per_sec;
    Eigen::VectorXd acceleration_degs_per_sec2;
};

// Builds streamed_limits from the configured limits, which are in radians, scaled by
// tolerance_ratio so planner rounding just over a limit is not rejected.
streamed_limits make_streamed_limits(const Eigen::VectorXd& velocity_rad_per_sec,
                                     const Eigen::VectorXd& acceleration_rad_per_sec2,
                                     double tolerance_ratio);

// Converts an sdk trajectory_point into the trajectory_point_t we put on the wire.
//
// The sdk gives us every joint value in degrees and the time as a microsecond offset from the
// start of the stream. The controller wants radians and a duration_t split into whole seconds plus
// nanoseconds. Both sides measure time from the start of the trajectory, so we do not have to keep
// a running total and each point converts on its own.
//
// Streamed points come in already time parameterized, so this path skips trajex. The caller picked
// the profile, but it must stay within the given limits: every velocity, and every acceleration
// when the point carries them, is checked by magnitude against its joint's limit. The controller
// still checks the first point's speed against its per axis limits and the first position against
// START_MAX_PULSE_DEVIATION, so a stream the arm cannot reach gets rejected when we open the goal
// instead of being run.
//
// The arm's dof is the length of the limit vectors.
//
// Throws std::invalid_argument if the point has no constraints, since we need velocities, if any
// of its joint vectors is not dof long, or if any velocity or acceleration exceeds its limit.
trajectory_point_t convert_streamed_point(const viam::sdk::Arm::trajectory_point& point, const streamed_limits& limits);

// Converts a whole batch of streamed points and keeps them in order. Throws std::invalid_argument
// if the protocol cannot carry the limits' dof, if the two limit vectors disagree on it, or for
// any reason convert_streamed_point does.
std::vector<trajectory_point_t> convert_streamed_batch(const std::vector<viam::sdk::Arm::trajectory_point>& batch,
                                                       const streamed_limits& limits);

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
