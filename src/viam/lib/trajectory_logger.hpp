#pragma once

#include <json/json.h>
#include <Eigen/Core>
#include <list>
#include <string>
#include <vector>

extern "C" {
#include "protocol.h"
}

namespace robot {

struct StatusMessage;

// Logs failed trajectory generation (waypoints + config only)
class FailedTrajectoryLogger {
   public:
    static void log_failure(const std::string& telemetry_path,
                            const std::string& timestamp,
                            const std::string& robot_model,
                            uint32_t group_index,
                            double max_velocity_rad_per_sec,
                            double max_acceleration_rad_per_sec2,
                            const std::list<Eigen::VectorXd>& waypoints_rad,
                            const std::string& error_message);

   private:
    static void write_json_file(const std::string& filepath, const Json::Value& root);
    static std::string failed_trajectory_filename(const std::string& telemetry_path,
                                                  const std::string& robot_model,
                                                  const std::string& unix_time);
};

// Real-time trajectory logger this is a move only type and should not be shared between threads
class RealtimeTrajectoryLogger {
   public:
    RealtimeTrajectoryLogger(const std::string& telemetry_path,
                             const std::string& timestamp,
                             const std::string& robot_model,
                             uint32_t group_index);

    ~RealtimeTrajectoryLogger();

    void set_max_velocity(double max_velocity_rad_per_sec);
    void set_max_acceleration(double max_acceleration_rad_per_sec2);
    void set_waypoints(const std::list<Eigen::VectorXd>& waypoints_rad);
    void set_planned_trajectory(const std::vector<trajectory_point_t>& planned_trajectory_points);

    void append_realtime_sample(const StatusMessage& status);

    RealtimeTrajectoryLogger(const RealtimeTrajectoryLogger&) = delete;
    RealtimeTrajectoryLogger& operator=(const RealtimeTrajectoryLogger&) = delete;
    RealtimeTrajectoryLogger(RealtimeTrajectoryLogger&&) noexcept;
    RealtimeTrajectoryLogger& operator=(RealtimeTrajectoryLogger&&) noexcept;

   private:
    std::string filepath_;
    Json::Value root_;
    int64_t last_timestamp_{-1};

    void write_and_flush();
    static std::string realtime_trajectory_filename(const std::string& telemetry_path,
                                                    const std::string& robot_model,
                                                    const std::string& unix_time);
};

}  // namespace robot
