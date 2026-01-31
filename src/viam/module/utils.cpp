#include "utils.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>

#include <Eigen/Dense>

namespace {
/// Convert a string log level to the LogLevel enum
/// @param level_str The string log level
/// @return The corresponding LogLevel enum value (defaults to INFO)
viam::yaskawa::LogLevel string_to_log_level(const std::string& level_str) {
    if (level_str == "debug") {
        return viam::yaskawa::LogLevel::DEBUG;
    }
    if (level_str == "info") {
        return viam::yaskawa::LogLevel::INFO;
    }
    if (level_str == "warn" || level_str == "warning") {
        return viam::yaskawa::LogLevel::WARNING;
    }
    if (level_str == "error") {
        return viam::yaskawa::LogLevel::ERROR;
    }
    if (level_str == "critical" || level_str == "fatal") {
        return viam::yaskawa::LogLevel::CRITICAL;
    }
    // Default to WARNING
    return viam::yaskawa::LogLevel::INFO;
}
}  // namespace

void configure_logger(const viam::sdk::ResourceConfig& cfg) {
    const auto log_level_str = find_config_attribute<std::string>(cfg, "log_level").value_or("warn");

    const viam::yaskawa::LogLevel level = string_to_log_level(log_level_str);

    auto logger = std::make_shared<ViamSdkLogger>(level);

    viam::yaskawa::set_global_logger(logger);
}

/// Transform a vector using the inverse of a rotation matrix
/// @param vector The vector to transform
/// @param rotation_matrix The rotation matrix (applied in transpose/inverse)
/// @return The transformed vector
Eigen::Vector3d transform_vector(const Eigen::Vector3d& vector, const Eigen::Matrix3d& rotation_matrix) {
    return rotation_matrix.transpose() * vector;
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
