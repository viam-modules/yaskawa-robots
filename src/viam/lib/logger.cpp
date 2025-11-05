#include "logger.hpp"

#include <iomanip>
#include <iostream>

namespace viam {
namespace yaskawa {

// ============================================================================
// LogStream Implementation
// ============================================================================

LogStream::LogStream(ILogger* logger, LogLevel level)
    : logger_(logger), level_(level), active_(logger && level >= logger->get_min_level()) {}

LogStream::~LogStream() {
    if (active_ && logger_) {
        logger_->write_log(level_, stream_.str());
    }
}

LogStream::LogStream(LogStream&& other) noexcept
    : logger_(other.logger_), level_(other.level_), stream_(std::move(other.stream_)), active_(other.active_) {
    other.active_ = false;  // Prevent the moved-from object from logging
}

LogStream& LogStream::operator=(LogStream&& other) noexcept {
    if (this != &other) {
        // Flush current stream if active
        if (active_ && logger_) {
            logger_->write_log(level_, stream_.str());
        }

        logger_ = other.logger_;
        level_ = other.level_;
        stream_ = std::move(other.stream_);
        active_ = other.active_;
        other.active_ = false;
    }
    return *this;
}

// ============================================================================
// Logger Implementation
// ============================================================================

Logger::Logger(LogLevel min_level) : min_level_(min_level) {}

void Logger::set_min_level(LogLevel level) {
    const std::lock_guard<std::mutex> lock(mutex_);
    min_level_ = level;
}

LogLevel Logger::get_min_level() const noexcept {
    const std::lock_guard<std::mutex> lock(mutex_);
    return min_level_;
}

void Logger::set_show_timestamps(bool show) noexcept {
    const std::lock_guard<std::mutex> lock(mutex_);
    show_timestamps_ = show;
}

bool Logger::get_show_timestamps() const noexcept {
    const std::lock_guard<std::mutex> lock(mutex_);
    return show_timestamps_;
}

void Logger::write_log(LogLevel level, std::string_view message) {
    if (level < get_min_level()) {
        return;
    }

    const std::string formatted = format_message(level, message);

    // Lock only for the actual output operation
    const std::lock_guard<std::mutex> lock(mutex_);
    std::cout << formatted << '\n';
}

std::string Logger::format_message(LogLevel level, std::string_view message) const {
    std::ostringstream oss;

    if (get_show_timestamps()) {
        oss << "[" << get_timestamp() << "] ";
    }

    oss << "[" << to_string(level) << "] " << message;

    return oss.str();
}

std::string Logger::get_timestamp() {
    using namespace std::chrono;

    auto now = system_clock::now();
    auto now_time_t = system_clock::to_time_t(now);
    auto now_ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    std::ostringstream oss;

    // Use std::put_time for thread-safe time formatting (C++20)
    std::tm tm_buf;
#if defined(_WIN32)
    localtime_s(&tm_buf, &now_time_t);
#else
    auto* result_ptr = localtime_r(&now_time_t, &tm_buf);
    if (result_ptr == NULL) {
        perror("Error converting time with localtime_r");  // Print error message based on errno
        return "Error converting time with localtime_r";   // Indicate error
    }
#endif

    oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S") << '.' << std::setfill('0') << std::setw(3) << now_ms.count();

    return oss.str();
}

}  // namespace yaskawa
}  // namespace viam
