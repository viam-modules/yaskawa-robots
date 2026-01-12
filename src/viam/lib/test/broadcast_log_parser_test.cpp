#define BOOST_TEST_MODULE ViamControllerLogParserTest
#include <boost/test/unit_test.hpp>

#include <string>
#include <vector>

#include "../logger.hpp"

using viam::yaskawa::ILogger;
using viam::yaskawa::LogLevel;
using viam::yaskawa::ViamControllerLogParser;

/// Structure to capture parsed log messages for testing
struct ParsedLogMessage {
    LogLevel level;
    std::string message;
};

/// Test logger that captures log messages for verification
class TestLogger : public ILogger {
   public:
    std::vector<ParsedLogMessage> messages;

    TestLogger() : min_level_(LogLevel::DEBUG) {}

    void set_min_level(LogLevel level) override {
        min_level_ = level;
    }

    LogLevel get_min_level() const noexcept override {
        return min_level_;
    }

    void clear() {
        messages.clear();
    }

   protected:
    void write_log(LogLevel level, std::string_view message) override {
        messages.push_back({level, std::string(message)});
    }

   private:
    LogLevel min_level_;
};

/// Test fixture that sets up the test logger and parser
struct ParserFixture {
    std::shared_ptr<TestLogger> test_logger;
    std::shared_ptr<ILogger> original_logger;
    std::unique_ptr<ViamControllerLogParser> parser;

    ParserFixture() {
        // Save original logger
        original_logger = viam::yaskawa::get_global_logger();

        // Create and set test logger
        test_logger = std::make_shared<TestLogger>();
        viam::yaskawa::set_global_logger(test_logger);

        // Create parser
        parser = std::make_unique<ViamControllerLogParser>();
    }

    ~ParserFixture() {
        // Restore original logger
        viam::yaskawa::set_global_logger(original_logger);
    }

    /// Feed a null-terminated string to the parser for testing
    void feed_data(const char* data) {
        parser->process_data(data);
    }

    /// Trigger a timeout flush
    void flush() {
        parser->flush();
    }

    void reset() {
        parser->flush();
        test_logger->clear();
    }

    /// Access captured messages
    const std::vector<ParsedLogMessage>& messages() const {
        return test_logger->messages;
    }
};

BOOST_FIXTURE_TEST_SUITE(ViamControllerLogParserTests, ParserFixture)

BOOST_AUTO_TEST_CASE(test_broadcast_log_parsing) {
    feed_data(
        "2025-12-29 10:15:30 -I-Your test message\r\n2025-12-29 10:15:30 -I-Your second \r\n test message\r\n2025-12-29 10:15:30 -I-Your "
        "third test "
        "message\r\n");
    BOOST_CHECK_EQUAL(messages().size(), 2);
    BOOST_CHECK(messages()[0].level == LogLevel::INFO);
    BOOST_CHECK_EQUAL(messages()[0].message, "[CONTROLLER] Your test message");
    BOOST_CHECK_EQUAL(messages()[1].message, "[CONTROLLER] Your second \r\n test message");
    flush();
    BOOST_CHECK_EQUAL(messages().size(), 3);
    BOOST_CHECK_EQUAL(messages()[2].message, "[CONTROLLER] Your third test message");

    reset();
    BOOST_CHECK_EQUAL(messages().size(), 0);

    feed_data("2026-12-29 10:15:30 -I-Your \r\n");
    feed_data("test message\r\n");
    BOOST_CHECK_EQUAL(messages().size(), 0);
    flush();
    BOOST_CHECK(messages()[0].level == LogLevel::INFO);
    BOOST_CHECK_EQUAL(messages()[0].message, "[CONTROLLER] Your \r\ntest message");
    reset();
    feed_data(
        "Partial message \r\n going partial 2025-12-29 10:15:30 -E-Your test message\r\n2025-12-29 10:15:30 -K-Your second \r\n test "
        "message\r\n2025-12-29 "
        "10:15:30 -W-Your "
        "third test "
        "message\r\n");

    BOOST_CHECK_EQUAL(messages().size(), 3);

    flush();
    BOOST_CHECK_EQUAL(messages().size(), 4);

    BOOST_CHECK(messages()[0].level == LogLevel::INFO);
    BOOST_CHECK_EQUAL(messages()[0].message, "[CONTROLLER] Partial message \r\n going partial ");

    BOOST_CHECK(messages()[1].level == LogLevel::ERROR);
    BOOST_CHECK_EQUAL(messages()[1].message, "[CONTROLLER] Your test message");

    BOOST_CHECK(messages()[2].level == LogLevel::INFO);
    BOOST_CHECK_EQUAL(messages()[2].message, "[CONTROLLER] Your second \r\n test message");

    BOOST_CHECK(messages()[3].level == LogLevel::WARNING);
    BOOST_CHECK_EQUAL(messages()[3].message, "[CONTROLLER] Your third test message");
}

BOOST_AUTO_TEST_SUITE_END()
