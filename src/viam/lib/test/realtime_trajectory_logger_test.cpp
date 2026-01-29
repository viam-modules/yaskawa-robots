#include <cstdint>
#define BOOST_TEST_MODULE RealtimeTrajectoryLoggerTest
#include <boost/test/included/unit_test.hpp>

#include <json/json.h>
#include <Eigen/Core>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <list>
#include <string>
#include <thread>
#include <vector>

#include <third_party/trajectories/Path.h>
#include <third_party/trajectories/Trajectory.h>
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
};

// Helper function to sample trajectories (copied from robot_socket.cpp)
template <typename Func>
void sampling_func(std::vector<trajectory_point_t>& samples, double duration_sec, double sampling_frequency_hz, const Func& f) {
    if (duration_sec <= 0.0 || sampling_frequency_hz <= 0.0) {
        throw std::invalid_argument("duration_sec and sampling_frequency_hz are not both positive");
    }
    static constexpr std::size_t k_max_samples = 20000;
    const auto putative_samples = duration_sec * sampling_frequency_hz;
    if (!std::isfinite(putative_samples) || putative_samples > k_max_samples) {
        throw std::invalid_argument(
            "duration_sec and sampling_frequency_hz exceed "
            "the maximum allowable samples");
    }
    // Calculate the number of samples needed. this will always be at least 2.
    const auto num_samples = static_cast<std::size_t>(std::ceil(putative_samples) + 1);

    // Calculate the actual step size
    const double step = duration_sec / static_cast<double>((num_samples - 1));

    // Generate samples by evaluating f at each time point
    for (std::size_t i = 0; i < num_samples - 1; ++i) {
        samples.push_back(f(static_cast<double>(i) * step, step));
    }

    // Ensure the last sample uses exactly the duration_sec
    samples.push_back(f(duration_sec, step));
}

BOOST_FIXTURE_TEST_SUITE(RealtimeTrajectoryLoggerTests, TempDirFixture)

BOOST_AUTO_TEST_CASE(test_destructor_writes_file) {
    std::string filepath;
    {
        std::list<Eigen::VectorXd> waypoints;
        waypoints.push_back(Eigen::VectorXd::Zero(6));
        waypoints.push_back(Eigen::VectorXd::Ones(6));

        std::vector<trajectory_point_t> trajectory;
        trajectory_point_t point{{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0}, {0}, {0, 0}};
        trajectory.push_back(point);

        robot::RealtimeTrajectoryLogger logger(temp_dir.string(), "1234567890", "gp12", 0);
        logger.set_max_velocity(1.5);
        logger.set_max_acceleration(2.0);
        logger.set_waypoints(waypoints);
        logger.set_planned_trajectory(trajectory);

        filepath = temp_dir.string() + "/trajectory_logs/1234567890_gp12_realtime_trajectory.json";
        BOOST_CHECK(!fs::exists(filepath));
    }

    BOOST_CHECK(fs::exists(filepath));

    std::ifstream file(filepath);
    BOOST_REQUIRE(file.is_open());

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    BOOST_REQUIRE(Json::parseFromStream(builder, file, &root, &errs));

    BOOST_CHECK_EQUAL(root["timestamp"].asString(), "1234567890");
    BOOST_CHECK_EQUAL(root["robot_model"].asString(), "gp12");
    BOOST_CHECK_EQUAL(root["group_index"].asUInt(), 0);
    BOOST_CHECK(root["realtime_samples"].isArray());
    BOOST_CHECK_EQUAL(root["realtime_samples"].size(), 0);
}

BOOST_AUTO_TEST_CASE(test_timestamp_deduplication) {
    std::list<Eigen::VectorXd> waypoints;
    waypoints.push_back(Eigen::VectorXd::Zero(6));

    std::vector<trajectory_point_t> trajectory;
    trajectory_point_t point{{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0}, {0}, {0, 0}};
    trajectory.push_back(point);

    std::string filepath;
    {
        robot::RealtimeTrajectoryLogger logger(temp_dir.string(), "1234567891", "gp12", 0);
        logger.set_max_velocity(1.5);
        logger.set_max_acceleration(2.0);
        logger.set_waypoints(waypoints);
        logger.set_planned_trajectory(trajectory);

        robot::StatusMessage status1;
        status1.timestamp = 100;
        status1.num_axes = 6;
        status1.position = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
        status1.velocity = {0.01, 0.02, 0.03, 0.04, 0.05, 0.06};
        status1.torque = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5};
        status1.position_corrected = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

        robot::StatusMessage status2 = status1;
        status2.timestamp = 100;

        robot::StatusMessage status3 = status1;
        status3.timestamp = 200;

        logger.append_realtime_sample(status1);
        logger.append_realtime_sample(status2);
        logger.append_realtime_sample(status3);

        filepath = temp_dir.string() + "/trajectory_logs/1234567891_gp12_realtime_trajectory.json";
    }

    std::ifstream file(filepath);
    BOOST_REQUIRE(file.is_open());

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    BOOST_REQUIRE(Json::parseFromStream(builder, file, &root, &errs));

    BOOST_CHECK_EQUAL(root["realtime_samples"].size(), 2);
    BOOST_CHECK_EQUAL(root["realtime_samples"][0]["timestamp_ms"].asInt64(), 100);
    BOOST_CHECK_EQUAL(root["realtime_samples"][1]["timestamp_ms"].asInt64(), 200);
}

BOOST_AUTO_TEST_CASE(test_append_realtime_samples) {
    std::list<Eigen::VectorXd> waypoints;
    waypoints.push_back(Eigen::VectorXd::Zero(6));

    std::vector<trajectory_point_t> trajectory;
    trajectory_point_t point{{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0}, {0}, {0, 0}};
    trajectory.push_back(point);

    std::string filepath;
    {
        robot::RealtimeTrajectoryLogger logger(temp_dir.string(), "1234567892", "gp12", 0);
        logger.set_max_velocity(1.5);
        logger.set_max_acceleration(2.0);
        logger.set_waypoints(waypoints);
        logger.set_planned_trajectory(trajectory);

        for (int i = 0; i < 5; ++i) {
            robot::StatusMessage status;
            status.timestamp = 1000 + (i * 100);
            status.num_axes = 6;
            status.position = {0.1 * i, 0.2 * i, 0.3 * i, 0.4 * i, 0.5 * i, 0.6 * i};
            status.velocity = {0.01 * i, 0.02 * i, 0.03 * i, 0.04 * i, 0.05 * i, 0.06 * i};
            status.torque = {1.0 + i, 1.1 + i, 1.2 + i, 1.3 + i, 1.4 + i, 1.5 + i};
            status.position_corrected = {0.1 * i, 0.2 * i, 0.3 * i, 0.4 * i, 0.5 * i, 0.6 * i};
            logger.append_realtime_sample(status);
        }

        filepath = temp_dir.string() + "/trajectory_logs/1234567892_gp12_realtime_trajectory.json";
    }

    std::ifstream file(filepath);
    BOOST_REQUIRE(file.is_open());

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    BOOST_REQUIRE(Json::parseFromStream(builder, file, &root, &errs));

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
    std::list<Eigen::VectorXd> waypoints;
    waypoints.push_back(Eigen::VectorXd::Zero(6));

    std::vector<trajectory_point_t> trajectory;
    trajectory_point_t point{{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0}, {0}, {0, 0}};
    trajectory.push_back(point);

    std::string filepath;
    {
        robot::RealtimeTrajectoryLogger logger(temp_dir.string(), "1234567893", "gp12", 0);
        logger.set_max_velocity(1.5);
        logger.set_max_acceleration(2.0);
        logger.set_waypoints(waypoints);
        logger.set_planned_trajectory(trajectory);
        filepath = temp_dir.string() + "/trajectory_logs/1234567893_gp12_realtime_trajectory.json";
    }

    std::ifstream file(filepath);
    BOOST_REQUIRE(file.is_open());

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    BOOST_REQUIRE(Json::parseFromStream(builder, file, &root, &errs));

    BOOST_CHECK(root["realtime_samples"].isArray());
    BOOST_CHECK_EQUAL(root["realtime_samples"].size(), 0);
}

BOOST_AUTO_TEST_CASE(test_json_structure_validation) {
    std::list<Eigen::VectorXd> waypoints;
    Eigen::VectorXd wp1(6);
    wp1 << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
    waypoints.push_back(wp1);
    Eigen::VectorXd wp2(6);
    wp2 << 0.7, 0.8, 0.9, 1.0, 1.1, 1.2;
    waypoints.push_back(wp2);

    std::vector<trajectory_point_t> trajectory;
    trajectory_point_t point{{0.5, 0.6, 0.7, 0.8, 0.9, 1.0}, {0.1, 0.2, 0.3, 0.4, 0.5, 0.6}, {0}, {0}, {1, 500000000}};
    trajectory.push_back(point);

    std::string filepath;
    {
        robot::RealtimeTrajectoryLogger logger(temp_dir.string(), "1234567894", "gp180-120", 1);
        logger.set_max_velocity(2.5);
        logger.set_max_acceleration(3.0);
        logger.set_waypoints(waypoints);
        logger.set_planned_trajectory(trajectory);

        robot::StatusMessage status;
        status.timestamp = 5000;
        status.num_axes = 6;
        status.position = {1.1, 1.2, 1.3, 1.4, 1.5, 1.6};
        status.velocity = {0.11, 0.12, 0.13, 0.14, 0.15, 0.16};
        status.torque = {2.1, 2.2, 2.3, 2.4, 2.5, 2.6};
        status.position_corrected = {1.1, 1.2, 1.3, 1.4, 1.5, 1.6};
        logger.append_realtime_sample(status);

        filepath = temp_dir.string() + "/trajectory_logs/1234567894_gp180-120_realtime_trajectory.json";
    }

    std::ifstream file(filepath);
    BOOST_REQUIRE(file.is_open());

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    BOOST_REQUIRE(Json::parseFromStream(builder, file, &root, &errs));

    BOOST_CHECK_EQUAL(root["timestamp"].asString(), "1234567894");
    BOOST_CHECK_EQUAL(root["robot_model"].asString(), "gp180-120");
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

BOOST_AUTO_TEST_CASE(test_concurrent_appends) {
    constexpr int num_threads = 10;
    constexpr int samples_per_thread = 10;

    std::list<Eigen::VectorXd> waypoints;
    waypoints.push_back(Eigen::VectorXd::Zero(6));

    std::vector<trajectory_point_t> trajectory;
    trajectory_point_t point{{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0}, {0}, {0, 0}};
    trajectory.push_back(point);

    std::string filepath;
    {
        robot::RealtimeTrajectoryLogger logger(temp_dir.string(), "1234567895", "gp12", 0);
        logger.set_max_velocity(1.5);
        logger.set_max_acceleration(2.0);
        logger.set_waypoints(waypoints);
        logger.set_planned_trajectory(trajectory);

        std::vector<std::thread> threads;

        for (int t = 0; t < num_threads; ++t) {
            threads.emplace_back([&logger, t]() {
                for (int i = 0; i < samples_per_thread; ++i) {
                    robot::StatusMessage status;
                    status.timestamp = (t * samples_per_thread) + i;
                    status.num_axes = 6;
                    status.position = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
                    status.velocity = {0.01, 0.02, 0.03, 0.04, 0.05, 0.06};
                    status.torque = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5};
                    status.position_corrected = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
                    logger.append_realtime_sample(status);
                }
            });
        }

        for (auto& thread : threads) {
            thread.join();
        }

        filepath = temp_dir.string() + "/trajectory_logs/1234567895_gp12_realtime_trajectory.json";
    }

    std::ifstream file(filepath);
    BOOST_REQUIRE(file.is_open());

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    BOOST_REQUIRE(Json::parseFromStream(builder, file, &root, &errs));

    BOOST_CHECK_EQUAL(root["realtime_samples"].size(), num_threads * samples_per_thread);
}

BOOST_AUTO_TEST_CASE(test_sinusoidal_trajectory_with_realtime_samples) {
    // Generate 60 waypoints over 2π radians (one complete cycle)
    constexpr int num_waypoints = 60;
    constexpr double motion_duration = 2.0 * M_PI;

    std::list<Eigen::VectorXd> waypoints;

    for (int i = 0; i < num_waypoints; ++i) {
        const double t = (static_cast<double>(i) / (num_waypoints - 1)) * motion_duration;

        Eigen::VectorXd waypoint(6);

        // Each joint follows a different sinusoidal pattern
        waypoint[0] = 0.5 * std::sin(1.0 * t);
        waypoint[1] = 0.3 * std::sin(1.5 * t);
        waypoint[2] = 0.4 * std::sin(0.8 * t);
        waypoint[3] = 0.6 * std::sin(1.2 * t);
        waypoint[4] = 0.35 * std::sin(0.9 * t);
        waypoint[5] = 0.45 * std::sin(1.1 * t);

        waypoints.push_back(waypoint);
    }

    // Set velocity/acceleration constraints
    constexpr double max_velocity = 1.5;
    constexpr double max_acceleration = 2.0;

    const Eigen::VectorXd max_velocity_vec = Eigen::VectorXd::Constant(6, max_velocity);
    const Eigen::VectorXd max_acceleration_vec = Eigen::VectorXd::Constant(6, max_acceleration);

    // Create trajectory using Path/Trajectory library
    const Path path(waypoints, 0.1);
    const Trajectory trajectory(path, max_velocity_vec, max_acceleration_vec);

    // Validate trajectory
    BOOST_REQUIRE(trajectory.isValid());

    const double duration = trajectory.getDuration();
    BOOST_REQUIRE(std::isfinite(duration));
    BOOST_REQUIRE(duration > 0.0);
    BOOST_REQUIRE(duration < 600.0);

    // Sample trajectory at 3 Hz
    constexpr double sampling_freq_hz = 3.0;
    std::vector<trajectory_point_t> planned_trajectory;

    sampling_func(planned_trajectory, duration, sampling_freq_hz, [&](const double t, const double) {
        auto p = trajectory.getPosition(t);
        auto v = trajectory.getVelocity(t);

        auto secs = static_cast<int32_t>(std::floor(t));
        auto nanos = static_cast<int32_t>((t - secs) * 1e9);

        return trajectory_point_t{{p[0], p[1], p[2], p[3], p[4], p[5]}, {v[0], v[1], v[2], v[3], v[4], v[5]}, {0}, {0}, {secs, nanos}};
    });

    BOOST_REQUIRE(planned_trajectory.size() > 0);

    std::vector<trajectory_point_t> planned_trajectory_echo;
    sampling_func(planned_trajectory_echo, duration, 200, [&](const double t, const double) {
        auto p = trajectory.getPosition(t);
        auto v = trajectory.getVelocity(t);

        auto secs = static_cast<int32_t>(std::floor(t));
        auto nanos = static_cast<int32_t>((t - secs) * 1e9);

        return trajectory_point_t{{p[0], p[1], p[2], p[3], p[4], p[5]}, {v[0], v[1], v[2], v[3], v[4], v[5]}, {0}, {0}, {secs, nanos}};
    });

    BOOST_REQUIRE(planned_trajectory_echo.size() > 0);

    std::string filepath;
    {
        // Create logger with planned trajectory
        robot::RealtimeTrajectoryLogger logger(temp_dir.string(), "1234567896", "test_gp12", 0);
        logger.set_max_velocity(max_velocity);
        logger.set_max_acceleration(max_acceleration);
        logger.set_waypoints(waypoints);
        logger.set_planned_trajectory(planned_trajectory);

        // Feed each planned trajectory point as a realtime sample
        int64_t timestamp_ms = 1738167890000;
        int64_t inc = int64_t(duration * 1000 / (double)planned_trajectory_echo.size());

        for (const auto& traj_point : planned_trajectory_echo) {
            robot::StatusMessage status;
            status.timestamp = timestamp_ms;
            status.num_axes = 6;

            // Copy positions and velocities manually to avoid packed member address warning
            status.position.resize(6);
            status.velocity.resize(6);
            for (size_t i = 0; i < 6; ++i) {
                status.position[i] = traj_point.positions[i];
                status.velocity[i] = traj_point.velocities[i];
            }
            status.torque.resize(6);
            for (size_t i = 0; i < 6; ++i) {
                status.torque[i] = 0;
            }

            status.position_corrected = status.position;

            logger.append_realtime_sample(status);

            timestamp_ms += inc;
        }

        filepath = temp_dir.string() + "/trajectory_logs/1234567896_test_gp12_realtime_trajectory.json";
    }

    // File should exist after logger destruction
    BOOST_REQUIRE(fs::exists(filepath));

    // Parse JSON
    std::ifstream file(filepath);
    BOOST_REQUIRE(file.is_open());

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    BOOST_REQUIRE(Json::parseFromStream(builder, file, &root, &errs));

    // Verify metadata
    BOOST_CHECK_EQUAL(root["timestamp"].asString(), "1234567896");
    BOOST_CHECK_EQUAL(root["robot_model"].asString(), "test_gp12");
    BOOST_CHECK_EQUAL(root["group_index"].asUInt(), 0);

    // Verify configuration
    BOOST_CHECK_CLOSE(root["configuration"]["max_velocity_rad_per_sec"].asDouble(), 1.5, 1e-6);
    BOOST_CHECK_CLOSE(root["configuration"]["max_acceleration_rad_per_sec2"].asDouble(), 2.0, 1e-6);

    // Verify waypoints
    BOOST_CHECK_EQUAL(root["waypoints_rad"].size(), num_waypoints);

    // Verify first waypoint is approximately [0, 0, 0, 0, 0, 0]
    for (int i = 0; i < 6; ++i) {
        BOOST_CHECK_SMALL(root["waypoints_rad"][0][i].asDouble(), 1e-10);
    }

    // Verify planned trajectory
    BOOST_REQUIRE(root["planned_trajectory"].isArray());
    BOOST_CHECK(root["planned_trajectory"].size() > 0);
    BOOST_CHECK_EQUAL(root["planned_trajectory"].size(), planned_trajectory.size());

    // Verify each planned trajectory point has correct structure
    for (const auto& point : root["planned_trajectory"]) {
        BOOST_CHECK(point.isMember("time_from_start_sec"));
        BOOST_CHECK(point.isMember("positions_rad"));
        BOOST_CHECK(point.isMember("velocities_rad_per_sec"));
        BOOST_CHECK(point.isMember("accelerations_rad_per_sec2"));

        BOOST_CHECK_EQUAL(point["positions_rad"].size(), 6);
        BOOST_CHECK_EQUAL(point["velocities_rad_per_sec"].size(), 6);
    }

    // Verify realtime samples (fed at 200Hz via planned_trajectory_echo)
    BOOST_REQUIRE(root["realtime_samples"].isArray());
    BOOST_CHECK_EQUAL(root["realtime_samples"].size(), planned_trajectory_echo.size());

    // Verify each realtime sample has correct structure
    for (const auto& sample : root["realtime_samples"]) {
        BOOST_CHECK(sample.isMember("timestamp_ms"));
        BOOST_CHECK(sample.isMember("positions_rad"));
        BOOST_CHECK(sample.isMember("velocities_rad_per_sec"));
        BOOST_CHECK(sample.isMember("torques_nm"));
        BOOST_CHECK(sample.isMember("positions_corrected_rad"));

        BOOST_CHECK_EQUAL(sample["positions_rad"].size(), 6);
        BOOST_CHECK_EQUAL(sample["velocities_rad_per_sec"].size(), 6);
        BOOST_CHECK_EQUAL(sample["torques_nm"].size(), 6);
        BOOST_CHECK_EQUAL(sample["positions_corrected_rad"].size(), 6);
    }

    // Verify timestamps are monotonically increasing
    int64_t last_timestamp = 0;
    for (const auto& sample : root["realtime_samples"]) {
        int64_t current_timestamp = sample["timestamp_ms"].asInt64();
        BOOST_CHECK(current_timestamp > last_timestamp);
        last_timestamp = current_timestamp;
    }

    // Copy to persistent location for manual inspection
    const std::string persistent_path = "/tmp/sinusoidal_trajectory_test.json";
    std::error_code ec;
    fs::copy_file(filepath, persistent_path, fs::copy_options::overwrite_existing, ec);

    // Print summary for manual inspection
    std::cout << "\n=== Sinusoidal Trajectory Test Summary ===" << std::endl;
    std::cout << "Waypoints: " << root["waypoints_rad"].size() << std::endl;
    std::cout << "Planned trajectory points: " << root["planned_trajectory"].size() << std::endl;
    std::cout << "Realtime samples: " << root["realtime_samples"].size() << std::endl;
    std::cout << "Trajectory duration: " << duration << " seconds" << std::endl;
    std::cout << "Output file (temp): " << filepath << std::endl;
    if (!ec) {
        std::cout << "Persistent copy: " << persistent_path << std::endl;
    }
    std::cout << "==========================================\n" << std::endl;
}

BOOST_AUTO_TEST_SUITE_END()
