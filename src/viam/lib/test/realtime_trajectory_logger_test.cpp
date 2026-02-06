#define BOOST_TEST_MODULE RealtimeTrajectoryLoggerTest
#include <boost/test/included/unit_test.hpp>

#include <json/json.h>
#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <list>
#include <string>
#include <vector>

#include "../robot_socket.hpp"
#include "../trajectory_logger.hpp"

extern "C" {
#include "../protocol.h"
}

namespace fs = std::filesystem;

struct TempDirFixture {
    fs::path temp_dir;

    TempDirFixture() {
        temp_dir = fs::temp_directory_path() /
                   ("realtime_logger_test_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()));
        fs::create_directories(temp_dir);
    }

    ~TempDirFixture() {
        std::error_code ec;
        fs::remove_all(temp_dir, ec);
    }

    static std::list<Eigen::VectorXd> make_waypoints(size_t count = 1) {
        std::list<Eigen::VectorXd> waypoints;
        for (size_t i = 0; i < count; ++i) {
            waypoints.push_back(Eigen::VectorXd::Constant(6, static_cast<double>(i)));
        }
        return waypoints;
    }

    static std::vector<trajectory_point_t> make_trajectory(size_t count = 1) {
        std::vector<trajectory_point_t> trajectory;
        trajectory.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            trajectory.push_back(trajectory_point_t{{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0}, {0}, {0, 0}});
        }
        return trajectory;
    }

    static robot::StatusMessage make_status(int64_t timestamp, double scale = 1.0) {
        robot::StatusMessage status;
        status.timestamp = timestamp;
        status.num_axes = 6;
        status.position = {0.1 * scale, 0.2 * scale, 0.3 * scale, 0.4 * scale, 0.5 * scale, 0.6 * scale};
        status.velocity = {0.01 * scale, 0.02 * scale, 0.03 * scale, 0.04 * scale, 0.05 * scale, 0.06 * scale};
        status.torque = {1.0 * scale, 1.1 * scale, 1.2 * scale, 1.3 * scale, 1.4 * scale, 1.5 * scale};
        status.position_corrected = status.position;
        return status;
    }

    static void configure_logger(robot::RealtimeTrajectoryLogger& logger,
                                 double velocity = 1.5,
                                 double acceleration = 2.0,
                                 size_t waypoint_count = 1,
                                 size_t trajectory_count = 1) {
        logger.set_max_velocity(velocity);
        logger.set_max_acceleration(acceleration);
        logger.set_waypoints(make_waypoints(waypoint_count));
        logger.set_planned_trajectory(make_trajectory(trajectory_count));
    }

    static Json::Value parse_json(const std::string& filepath) {
        std::ifstream file(filepath);
        BOOST_REQUIRE(file.is_open());

        Json::Value root;
        Json::CharReaderBuilder builder;
        std::string errs;
        BOOST_REQUIRE(Json::parseFromStream(builder, file, &root, &errs));
        return root;
    }

    std::string log_path(const std::string& timestamp, const std::string& model = "gp12") const {
        return temp_dir.string() + "/trajectory_logs/" + timestamp + "_" + model + "_realtime_trajectory.json";
    }
};

BOOST_FIXTURE_TEST_SUITE(RealtimeTrajectoryLoggerTests, TempDirFixture)

BOOST_AUTO_TEST_CASE(test_destructor_writes_file) {
    const std::string timestamp = "1234567890";
    std::string filepath = log_path(timestamp);

    {
        robot::RealtimeTrajectoryLogger logger(temp_dir.string(), timestamp, "gp12", 0);
        configure_logger(logger, 1.5, 2.0, 2);
        BOOST_CHECK(!fs::exists(filepath));
    }

    BOOST_CHECK(fs::exists(filepath));

    auto root = parse_json(filepath);
    BOOST_CHECK_EQUAL(root["timestamp"].asString(), timestamp);
    BOOST_CHECK_EQUAL(root["robot_model"].asString(), "gp12");
    BOOST_CHECK_EQUAL(root["group_index"].asUInt(), 0);
    BOOST_CHECK(root["realtime_samples"].isArray());
    BOOST_CHECK_EQUAL(root["realtime_samples"].size(), 0);
}

BOOST_AUTO_TEST_CASE(test_timestamp_deduplication) {
    const std::string timestamp = "1234567891";
    std::string filepath = log_path(timestamp);

    {
        robot::RealtimeTrajectoryLogger logger(temp_dir.string(), timestamp, "gp12", 0);
        configure_logger(logger);

        logger.append_realtime_sample(make_status(100));
        logger.append_realtime_sample(make_status(100));  // duplicate - should be ignored
        logger.append_realtime_sample(make_status(200));
    }

    auto root = parse_json(filepath);
    BOOST_CHECK_EQUAL(root["realtime_samples"].size(), 2);
    BOOST_CHECK_EQUAL(root["realtime_samples"][0]["timestamp_ms"].asInt64(), 100);
    BOOST_CHECK_EQUAL(root["realtime_samples"][1]["timestamp_ms"].asInt64(), 200);
}

BOOST_AUTO_TEST_CASE(test_append_realtime_samples) {
    const std::string timestamp = "1234567892";
    std::string filepath = log_path(timestamp);

    {
        robot::RealtimeTrajectoryLogger logger(temp_dir.string(), timestamp, "gp12", 0);
        configure_logger(logger);

        for (int i = 0; i < 5; ++i) {
            logger.append_realtime_sample(make_status(1000 + (i * 100), static_cast<double>(i)));
        }
    }

    auto root = parse_json(filepath);
    BOOST_CHECK_EQUAL(root["realtime_samples"].size(), 5);

    for (int i = 0; i < 5; ++i) {
        const auto& sample = root["realtime_samples"][i];
        BOOST_CHECK_EQUAL(sample["timestamp_ms"].asInt64(), 1000 + (i * 100));
        BOOST_CHECK_EQUAL(sample["positions_rad"].size(), 6);
        BOOST_CHECK_EQUAL(sample["velocities_rad_per_sec"].size(), 6);
        BOOST_CHECK_EQUAL(sample["torques_nm"].size(), 6);
        BOOST_CHECK_EQUAL(sample["positions_corrected_rad"].size(), 6);
        BOOST_CHECK_CLOSE(sample["positions_rad"][0].asDouble(), 0.1 * i, 1e-6);
    }
}

BOOST_AUTO_TEST_CASE(test_empty_realtime_samples) {
    const std::string timestamp = "1234567893";
    std::string filepath = log_path(timestamp);

    {
        robot::RealtimeTrajectoryLogger logger(temp_dir.string(), timestamp, "gp12", 0);
        configure_logger(logger);
    }

    auto root = parse_json(filepath);
    BOOST_CHECK(root["realtime_samples"].isArray());
    BOOST_CHECK_EQUAL(root["realtime_samples"].size(), 0);
}

BOOST_AUTO_TEST_CASE(test_json_structure_validation) {
    const std::string timestamp = "1234567894";
    const std::string model = "gp180-120";
    std::string filepath = log_path(timestamp, model);

    {
        robot::RealtimeTrajectoryLogger logger(temp_dir.string(), timestamp, model, 1);
        logger.set_max_velocity(2.5);
        logger.set_max_acceleration(3.0);
        logger.set_waypoints(make_waypoints(2));
        logger.set_planned_trajectory(make_trajectory(1));

        logger.append_realtime_sample(make_status(5000));
    }

    auto root = parse_json(filepath);

    BOOST_CHECK_EQUAL(root["timestamp"].asString(), timestamp);
    BOOST_CHECK_EQUAL(root["robot_model"].asString(), model);
    BOOST_CHECK_EQUAL(root["group_index"].asUInt(), 1);

    BOOST_CHECK(root["configuration"].isObject());
    BOOST_CHECK_CLOSE(root["configuration"]["max_velocity_rad_per_sec"].asDouble(), 2.5, 1e-6);
    BOOST_CHECK_CLOSE(root["configuration"]["max_acceleration_rad_per_sec2"].asDouble(), 3.0, 1e-6);

    BOOST_CHECK(root["waypoints_rad"].isArray());
    BOOST_CHECK_EQUAL(root["waypoints_rad"].size(), 2);

    BOOST_CHECK(root["planned_trajectory"].isArray());
    BOOST_CHECK_EQUAL(root["planned_trajectory"].size(), 1);

    BOOST_CHECK(root["realtime_samples"].isArray());
    BOOST_CHECK_EQUAL(root["realtime_samples"].size(), 1);
    BOOST_CHECK_EQUAL(root["realtime_samples"][0]["timestamp_ms"].asInt64(), 5000);
}

BOOST_AUTO_TEST_SUITE_END()
