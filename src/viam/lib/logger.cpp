#include "logger.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>

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

Logger::Logger(LogLevel min_level) noexcept : min_level_(min_level) {}

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
    std::cout << formatted << std::endl;  // NOLINT(performance-avoid-endl)
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
        // TODO(RSDK-12309) evaluate if returning a string here is ok
        return "Error converting time with localtime_r";  // Indicate error
    }
#endif

    oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S") << '.' << std::setfill('0') << std::setw(3) << now_ms.count();

    return oss.str();
}

// Static pattern strings

// This regex captures anything between "%Y-%m-%d %H:%M:%S -I-" and "%Y-%m-%d %H:%M:%S -I-" not counting the last \r\n
constexpr const char* k_full_log_pattern_str{
    R"((\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\s)-([a-zA-Z])-(?s:(.+?))(?=[\r\n]{2}\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\s-[a-zA-Z]-))"};
// finds anything that resemble a log statement
constexpr const char* k_partial_log_pattern_str{R"((\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\s)-([a-zA-Z])-(?s:(.+))(?<![\s]))"};

ViamControllerLogParser::ViamControllerLogParser() {
    try {
        full_log_pattern_ = boost::regex(k_full_log_pattern_str);
        partial_log_pattern_ = boost::regex(k_partial_log_pattern_str);
    } catch (const boost::regex_error& e) {
        throw std::runtime_error(std::string("Failed to compile broadcast log regex: ") + e.what());
    }
}

LogLevel ViamControllerLogParser::parse_level(char level_char) {
    switch (level_char) {
        case 'D':
            return LogLevel::DEBUG;
        case 'I':
            return LogLevel::INFO;
        case 'W':
            return LogLevel::WARNING;
        case 'E':
            return LogLevel::ERROR;
        case 'C':
            return LogLevel::CRITICAL;
        default:
            return LogLevel::INFO;
    }
}

void ViamControllerLogParser::process_data(const char* data) {
    // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange) Removing this triggers an error in boost/regex/v5/match_flags.hpp
    if (data != nullptr) {
        buffer_.append(data);
        process_buffer();
    }
}
void ViamControllerLogParser::process_buffer() {
    boost::smatch what;

    while (boost::regex_search(buffer_, what, full_log_pattern_, boost::regex_constants::match_any)) {
        auto begin = what[0].begin();
        // if we have some characters before a matched log statement then log them separately.
        if (what.prefix().matched) {
            LOGGING(info) << "[CONTROLLER] " << what.prefix().str();
            begin = what.prefix().begin();
        }
        auto level = parse_level(what[2].str()[0]);

        get_global_logger()->log(level) << "[CONTROLLER] " << what[3].str();
        auto end = what[0].end();
        std::advance(end, std::min(std::distance(end, buffer_.cend()), std::string::difference_type(2)));
        buffer_.erase(begin, end);
    }
}

void ViamControllerLogParser::flush() {
    // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange) Removing this triggers an error in boost/regex/v5/match_flags.hpp
    if (!buffer_.empty()) {
        boost::smatch what;
        if (boost::regex_search(buffer_, what, partial_log_pattern_)) {
            // if we have some characters before a matched log statement then log them separately.
            if (what.prefix().matched) {
                LOGGING(info) << "[CONTROLLER] " << what.prefix().str();
            }

            auto level = parse_level(what[2].str()[0]);
            get_global_logger()->log(level) << "[CONTROLLER] " << what[3].str();
        }

        buffer_.clear();
    }
}

bool ViamControllerLogParser::has_pending_data() const {
    return !buffer_.empty();
}

}  // namespace yaskawa
}  // namespace viam
