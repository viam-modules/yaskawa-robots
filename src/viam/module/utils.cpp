#include "utils.hpp"

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
