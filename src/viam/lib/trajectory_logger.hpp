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

// Logs generated trajectories (full arrays: time, position, velocity, acceleration)
class GeneratedTrajectoryLogger {
   public:
    static void log_trajectory(const std::string& telemetry_path,
                               const std::string& timestamp,
                               const std::string& robot_model,
                               uint32_t group_index,
                               double max_velocity_rad_per_sec,
                               double max_acceleration_rad_per_sec2,
                               const std::list<Eigen::VectorXd>& waypoints_rad,
                               const std::vector<trajectory_point_t>& trajectory_points);

   private:
    static void write_json_file(const std::string& filepath, const Json::Value& root);
    static std::string trajectory_filename(const std::string& telemetry_path, const std::string& robot_model, const std::string& unix_time);
};

}  // namespace robot
