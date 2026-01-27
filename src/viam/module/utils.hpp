#pragma once

#include <mutex>
#include <optional>
#include <sstream>
#include <string_view>

#include <Eigen/Dense>
#include <viam/lib/logger.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/log/logging.hpp>

/// Logger implementation that writes to VIAM SDK logging infrastructure
class ViamSdkLogger final : public viam::yaskawa::ILogger {
   public:
    explicit ViamSdkLogger(viam::yaskawa::LogLevel min_level = viam::yaskawa::LogLevel::INFO) noexcept : min_level_(min_level) {}

    ~ViamSdkLogger() final = default;

    ViamSdkLogger(const ViamSdkLogger&) = delete;
    ViamSdkLogger& operator=(const ViamSdkLogger&) = delete;
    ViamSdkLogger(ViamSdkLogger&&) noexcept = delete;
    ViamSdkLogger& operator=(ViamSdkLogger&&) noexcept = delete;

    void set_min_level(viam::yaskawa::LogLevel level) final {
        const std::lock_guard<std::mutex> lock{mutex_};
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
