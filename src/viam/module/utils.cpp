#include "utils.hpp"

#include <chrono>
#include <format>
#include <iomanip>
#include <sstream>
#include <vector>

#include <boost/variant.hpp>

#include <Eigen/Dense>
#include <viam/lib/robot_socket.hpp>
#include <viam/sdk/common/proto_value.hpp>

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
    const viam::yaskawa::LogLevel level = string_to_log_level(viam::sdk::to_string(cfg.get_log_level()));

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

size_t validate_joint_limit_attribute(const viam::sdk::ResourceConfig& cfg, const std::string& attribute) {
    using viam::sdk::ProtoValue;

    const auto& attributes = cfg.attributes();
    auto it = attributes.find(attribute);
    if (it == attributes.end()) {
        throw std::invalid_argument(std::format("attribute `{}` is required", attribute));
    }

    const auto& value = it->second;

    if (const auto* scalar = value.get<double>()) {
        if (*scalar <= 0.0) {
            throw std::invalid_argument(std::format("attribute `{}` must be strictly positive, got {}", attribute, *scalar));
        }
        return 1;
    }

    if (const auto* arr = value.get<std::vector<ProtoValue>>()) {
        if (arr->empty()) {
            throw std::invalid_argument(std::format("attribute `{}` array must not be empty", attribute));
        }
        for (size_t i = 0; i < arr->size(); ++i) {
            const auto* elem = (*arr)[i].get<double>();
            if (!elem) {
                throw std::invalid_argument(std::format("attribute `{}` element {} is not a number", attribute, i));
            }
            if (*elem <= 0.0) {
                throw std::invalid_argument(
                    std::format("attribute `{}` element {} must be strictly positive, got {}", attribute, i, *elem));
            }
        }
        return arr->size();
    }

    throw std::invalid_argument(std::format("attribute `{}` must be a number or array of numbers", attribute));
}

void apply_move_limit(Eigen::VectorXd& limits, const boost::variant<double, std::vector<double>>& value) {
    struct visitor {
        Eigen::VectorXd& limits;

        void operator()(double s) const {
            if (s <= 0) {
                throw std::invalid_argument(std::format("scalar move limit must be positive, got: {}", s));
            }
            limits.setConstant(degrees_to_radians(s));
        }

        void operator()(const std::vector<double>& v) const {
            const auto dof = limits.size();
            if (static_cast<Eigen::Index>(v.size()) != dof) {
                throw std::invalid_argument(
                    std::format("move limit vector must have exactly {} elements (one per joint), got {}", dof, v.size()));
            }
            for (size_t i = 0; i < v.size(); ++i) {
                if (v[i] < 0) {
                    throw std::invalid_argument(std::format("move limit element {} cannot be negative, got: {}", i, v[i]));
                }
            }
            for (size_t i = 0; i < v.size(); ++i) {
                limits[static_cast<Eigen::Index>(i)] = degrees_to_radians(v[i]);
            }
        }
    };
    boost::apply_visitor(visitor{limits}, value);
}
