#include "trajectory_logger.hpp"

#include <boost/format.hpp>
#include <filesystem>
#include <fstream>
#include <system_error>

#include "logger.hpp"

namespace robot {

namespace {

constexpr int k_json_precision = 17;  // IEEE 754 double precision

// Convert Eigen vector to JSON array with high precision
Json::Value eigen_vector_to_json(const Eigen::VectorXd& vec) {
    Json::Value arr(Json::arrayValue);
    for (Eigen::Index i = 0; i < vec.size(); ++i) {
        arr.append(vec[i]);
    }
    return arr;
}

// Convert list of Eigen vectors to JSON array
Json::Value waypoints_to_json(const std::list<Eigen::VectorXd>& waypoints) {
    Json::Value arr(Json::arrayValue);
    for (const auto& waypoint : waypoints) {
        arr.append(eigen_vector_to_json(waypoint));
    }
    return arr;
}

// Convert trajectory_point_t array to JSON with 6-joint positions/velocities/accelerations
Json::Value trajectory_array_to_json(const std::vector<trajectory_point_t>& trajectory_points) {
    Json::Value arr(Json::arrayValue);
    for (const auto& point : trajectory_points) {
        Json::Value point_obj(Json::objectValue);

        // TODO(npm) don't use NUMBER_OF_DOF
        // Positions
        Json::Value positions(Json::arrayValue);
        for (int i = 0; i < NUMBER_OF_DOF; ++i) {
            positions.append(point.positions[i]);
        }
        point_obj["positions_rad"] = positions;

        // Velocities
        Json::Value velocities(Json::arrayValue);
        for (int i = 0; i < NUMBER_OF_DOF; ++i) {
            velocities.append(point.velocities[i]);
        }
        point_obj["velocities_rad_per_sec"] = velocities;

        // Accelerations
        Json::Value accelerations(Json::arrayValue);
        for (int i = 0; i < NUMBER_OF_DOF; ++i) {
            accelerations.append(point.accelerations[i]);
        }
        point_obj["accelerations_rad_per_sec2"] = accelerations;

        // Time from start (convert duration_t to seconds as double)
        const double time_sec = static_cast<double>(point.time_from_start.sec) + (static_cast<double>(point.time_from_start.nanos) * 1e-9);
        point_obj["time_from_start_sec"] = time_sec;

        arr.append(point_obj);
    }
    return arr;
}

}  // namespace

void FailedTrajectoryLogger::write_json_file(const std::string& filepath, const Json::Value& root) {
    try {
        // Configure high-precision streaming
        Json::StreamWriterBuilder builder;
        builder["precision"] = k_json_precision;
        builder["precisionType"] = "decimal";
        builder["indentation"] = "  ";

        std::ofstream file(filepath, std::ios::out | std::ios::trunc);
        if (!file.is_open()) {
            LOGGING(error) << "Failed to open trajectory log file: " << filepath;
            return;
        }

        const std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(root, &file);
        file << '\n';

        LOGGING(debug) << "Failed trajectory logged to: " << filepath;
        file.flush();
        file.close();
        if (file.fail()) {
            LOGGING(error) << "failed to write to file:  " << filepath;
        }
    } catch (const std::exception& e) {
        LOGGING(error) << "Failed to write trajectory log: " << e.what();
    }
}

std::string FailedTrajectoryLogger::failed_trajectory_filename(const std::string& telemetry_path,
                                                               const std::string& robot_model,
                                                               const std::string& unix_time) {
    constexpr char k_failed_trajectory_json_name_template[] = "/trajectory_logs/%1%_%2%_failed_trajectory.json";
    auto fmt = boost::format(telemetry_path + k_failed_trajectory_json_name_template);
    return (fmt % unix_time % robot_model).str();
}

void FailedTrajectoryLogger::log_failure(const std::string& telemetry_path,
                                         const std::string& timestamp,
                                         const std::string& robot_model,
                                         const uint32_t group_index,
                                         const double max_velocity_rad_per_sec,
                                         const double max_acceleration_rad_per_sec2,
                                         const std::list<Eigen::VectorXd>& waypoints_rad,
                                         const std::string& error_message) {
    try {
        // Ensure the trajectory_logs directory exists
        const std::filesystem::path log_dir = std::filesystem::path(telemetry_path) / "trajectory_logs";
        std::error_code ec;
        // returns fals if directory already exists we can safely ignore the return value
        std::filesystem::create_directories(log_dir, ec);
        if (ec) {
            LOGGING(error) << "Failed to created directory " << log_dir << " cause:" << ec.message();
        }

        Json::Value root(Json::objectValue);
        root["timestamp"] = timestamp;
        root["robot_model"] = robot_model;
        root["group_index"] = group_index;
        root["error_message"] = error_message;

        Json::Value config(Json::objectValue);
        config["max_velocity_rad_per_sec"] = max_velocity_rad_per_sec;
        config["max_acceleration_rad_per_sec2"] = max_acceleration_rad_per_sec2;
        root["configuration"] = config;

        root["waypoints_rad"] = waypoints_to_json(waypoints_rad);

        const std::string filename = failed_trajectory_filename(telemetry_path, robot_model, timestamp);
        write_json_file(filename, root);
    } catch (const std::exception& e) {
        LOGGING(error) << "Exception in FailedTrajectoryLogger::log_failure: " << e.what();
    }
}

void GeneratedTrajectoryLogger::write_json_file(const std::string& filepath, const Json::Value& root) {
    try {
        // Configure high-precision streaming
        Json::StreamWriterBuilder builder;
        builder["precision"] = k_json_precision;
        builder["precisionType"] = "decimal";
        builder["indentation"] = "  ";

        std::ofstream file(filepath, std::ios::out | std::ios::trunc);
        if (!file.is_open()) {
            LOGGING(error) << "Failed to open trajectory log file: " << filepath;
            return;
        }

        const std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(root, &file);
        file << '\n';

        LOGGING(debug) << "Generated trajectory logged to: " << filepath;
        file.flush();
        file.close();
        if (file.fail()) {
            LOGGING(error) << "failed to write to file:  " << filepath;
        }
    } catch (const std::exception& e) {
        LOGGING(error) << "Failed to write trajectory log: " << e.what();
    }
}

std::string GeneratedTrajectoryLogger::trajectory_filename(const std::string& telemetry_path,
                                                           const std::string& robot_model,
                                                           const std::string& unix_time) {
    constexpr char k_trajectory_json_name_template[] = "/trajectory_logs/%1%_%2%_trajectory.json";
    auto fmt = boost::format(telemetry_path + k_trajectory_json_name_template);
    return (fmt % unix_time % robot_model).str();
}

void GeneratedTrajectoryLogger::log_trajectory(const std::string& telemetry_path,
                                               const std::string& timestamp,
                                               const std::string& robot_model,
                                               const uint32_t group_index,
                                               const double max_velocity_rad_per_sec,
                                               const double max_acceleration_rad_per_sec2,
                                               const std::list<Eigen::VectorXd>& waypoints_rad,
                                               const std::vector<trajectory_point_t>& trajectory_points) {
    try {
        // Ensure the trajectory_logs directory exists
        const std::filesystem::path log_dir = std::filesystem::path(telemetry_path) / "trajectory_logs";
        std::error_code ec;
        // returns fals if directory already exists we can safely ignore the return value
        std::filesystem::create_directories(log_dir, ec);
        if (ec) {
            LOGGING(error) << "Failed to created directory " << log_dir << " cause:" << ec.message();
        }

        Json::Value root(Json::objectValue);
        root["timestamp"] = timestamp;
        root["robot_model"] = robot_model;
        root["group_index"] = group_index;

        Json::Value config(Json::objectValue);
        config["max_velocity_rad_per_sec"] = max_velocity_rad_per_sec;
        config["max_acceleration_rad_per_sec2"] = max_acceleration_rad_per_sec2;
        root["configuration"] = config;

        root["waypoints_rad"] = waypoints_to_json(waypoints_rad);
        root["trajectory_points"] = trajectory_array_to_json(trajectory_points);

        const std::string filename = trajectory_filename(telemetry_path, robot_model, timestamp);
        write_json_file(filename, root);
    } catch (const std::exception& e) {
        LOGGING(error) << "Exception in GeneratedTrajectoryLogger::log_trajectory: " << e.what();
    }
}

}  // namespace robot
