#pragma once

#include <chrono>
#include <concepts>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include <boost/regex.hpp>

namespace viam {
namespace yaskawa {

/// Log levels in ascending order of severity
enum class LogLevel : std::uint8_t { DEBUG, INFO, WARNING, ERROR, CRITICAL };

/// Convert log level to string representation
constexpr std::string_view to_string(LogLevel level) noexcept {
    switch (level) {
        case LogLevel::DEBUG:
            return "DEBUG";
        case LogLevel::INFO:
            return "INFO";
        case LogLevel::WARNING:
            return "WARNING";
        case LogLevel::ERROR:
            return "ERROR";
        case LogLevel::CRITICAL:
            return "CRITICAL";
    }
    return "UNKNOWN";
}

// Forward declarations
class ILogger;
class Logger;

/// RAII proxy object for stream-style logging
/// Accumulates log messages and writes them when destroyed
class LogStream {
   public:
    LogStream(ILogger* logger, LogLevel level);
    ~LogStream();

    // Delete copy operations
    LogStream(const LogStream&) = delete;
    LogStream& operator=(const LogStream&) = delete;

    // Allow move operations
    LogStream(LogStream&& other) noexcept;
    LogStream& operator=(LogStream&& other) noexcept;

    /// Stream insertion operator for any type that supports operator<<
    template <typename T>
    LogStream& operator<<(const T& value) {
        if (active_) {
            stream_ << value;
        }
        return *this;
    }

    /// Check if this log stream is active (level >= min_level)
    bool is_active() const noexcept {
        return active_;
    }

   private:
    ILogger* logger_;
    LogLevel level_;
    std::ostringstream stream_;
    bool active_;
};

/// Abstract interface for logging
/// Users can inherit from this class to provide custom logging implementations
class ILogger {
   public:
    virtual ~ILogger() = default;

    /// Create a log stream for the specified level
    /// Usage: logger.log(LogLevel::INFO) << "message";
    LogStream log(LogLevel level) {
        return LogStream(this, level);
    }

    /// Convenience methods for specific log levels
    LogStream debug() {
        return log(LogLevel::DEBUG);
    }

    LogStream info() {
        return log(LogLevel::INFO);
    }

    LogStream warning() {
        return log(LogLevel::WARNING);
    }

    LogStream error() {
        return log(LogLevel::ERROR);
    }

    LogStream critical() {
        return log(LogLevel::CRITICAL);
    }

    /// Get/set the minimum log level to display
    virtual void set_min_level(LogLevel level) = 0;
    virtual LogLevel get_min_level() const noexcept = 0;

   protected:
    friend class LogStream;

    /// Internal method called by LogStream to write the final message
    virtual void write_log(LogLevel level, std::string_view message) = 0;
};

/// Default logger implementation that writes to stdout
/// This class is thread-safe and uses modern C++20 features
class Logger final : public ILogger {
   public:
    /// Construct a logger with the specified minimum log level
    explicit Logger(LogLevel min_level = LogLevel::INFO) noexcept;

    ~Logger() override = default;

    // Delete copy operations
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    // Delete move operations because we have a mutex in the Logger implementation
    Logger(Logger&&) noexcept = delete;
    Logger& operator=(Logger&&) noexcept = delete;

    /// Set the minimum log level
    void set_min_level(LogLevel level) final;

    /// Get the current minimum log level
    LogLevel get_min_level() const noexcept final;

    /// Enable or disable timestamps in log messages
    void set_show_timestamps(bool show) noexcept;

    /// Check if timestamps are enabled
    bool get_show_timestamps() const noexcept;

   protected:
    /// Write a log message (called by LogStream)
    void write_log(LogLevel level, std::string_view message) final;

   private:
    /// Format a log message with timestamp and level
    std::string format_message(LogLevel level, std::string_view message) const;

    /// Get current timestamp as string
    static std::string get_timestamp();

    mutable std::mutex mutex_;
    LogLevel min_level_;
    bool show_timestamps_{true};
};

/// Create a shared pointer to a Logger instance
/// This is the recommended way to create logger instances
/// Note: make_shared can throw a std::bad_alloc, which would result in a call to std::terminate.
/// if we are already out of memory at this point then things are already really bad, so terminating is okay.
inline std::shared_ptr<ILogger> make_logger(LogLevel min_level = LogLevel::INFO) noexcept {
    return std::make_shared<Logger>(min_level);
}

/// Global logger instance used by the library
/// Users can replace this with their own logger implementation using
/// set_global_logger()
inline std::shared_ptr<ILogger> global_logger = make_logger();

/// Set the global logger instance
/// This allows users to provide custom logger implementations
/// Thread-safe: should be called before any logging operations
inline void set_global_logger(std::shared_ptr<ILogger> logger) {
    if (logger) {
        global_logger = std::move(logger);
    }
}

/// Get the global logger instance
inline std::shared_ptr<ILogger> get_global_logger() {
    return global_logger;
}

/// Parser for broadcast log messages from the Yaskawa controller.
/// Messages are formatted as:
/// - Timestamp: "%Y-%m-%d %H:%M:%S "
/// - Level: "-X-" where X = C (critical), E (error), W (warn), I (info), D (debug)
/// - Termination: \r\n (there can be many \r\n in the log statement)
///
class ViamControllerLogParser {
   public:
    /// Construct parser and compile regex patterns
    /// @throws std::runtime_error if regex compilation fails
    ViamControllerLogParser();

    /// Process incoming data from a broadcast packet
    /// @param data String view of data received from the socket
    void process_data(std::string_view data);

    /// Flush any buffered data. There will be an attempt to extract a partial log statement
    void flush();

    /// Check if the buffer has pending data
    bool has_pending_data() const;

   private:
    /// Extract log level from a level marker like "-I-"
    static LogLevel parse_level(char level_char);

    /// Process complete statements in the buffer
    void process_buffer();

    std::string buffer_;
    boost::regex full_log_pattern_;
    boost::regex partial_log_pattern_;
};

}  // namespace yaskawa
}  // namespace viam

/// Convenience macro for logging
/// Usage: LOGGING(info) << "message";
/// Uses the global logger instance
#define LOGGING(level) viam::yaskawa::get_global_logger()->level()
