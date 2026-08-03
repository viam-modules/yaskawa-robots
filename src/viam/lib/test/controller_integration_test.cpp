#define BOOST_TEST_MODULE ControllerIntegrationTest
#include <boost/test/unit_test.hpp>

#include <boost/asio.hpp>
#include <chrono>
#include <list>
#include <memory>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <viam/sdk/config/resource.hpp>

#include "../robot_socket.hpp"
#include "fake_server.hpp"

#include "protocol.h"

constexpr int k_dof = 6;

viam::sdk::ResourceConfig make_config(uint16_t tcp_port) {
    viam::sdk::ProtoStruct attrs;
    attrs["host"] = viam::sdk::ProtoValue(std::string("127.0.0.1"));
    attrs["speed_rad_per_sec"] = viam::sdk::ProtoValue(1.0);
    attrs["acceleration_rad_per_sec2"] = viam::sdk::ProtoValue(1.0);
    attrs["tcp_port"] = viam::sdk::ProtoValue(static_cast<double>(tcp_port));
    return {"arm", "test-arm", "", std::move(attrs), "rdk:component:arm", viam::sdk::Model("test", "test", "test")};
}

// Parameterized fixture: pass a server group config (default = single 6-DOF robot).
template <const auto& ServerConfig = test::k_single_robot>
struct TestFixture {
    boost::asio::io_context io_ctx;
    std::thread io_thread;
    test::ServerPorts ports;
    test::FakeServer server;
    std::shared_ptr<robot::YaskawaController> controller;

    TestFixture() : ports(test::FakeServer::allocate_ports()), server(ports, ServerConfig) {
        io_thread = std::thread([this]() {
            auto guard = boost::asio::make_work_guard(io_ctx);
            io_ctx.run();
        });
        controller = std::make_shared<robot::YaskawaController>(io_ctx, make_config(ports.tcp_port));
    }

    ~TestFixture() {
        if (controller) {
            try {
                controller->disconnect();
            } catch (...) {
                LOGGING(debug) << "caught unknown exception";
            }
        }
        controller.reset();
        io_ctx.stop();
        if (io_thread.joinable()) {
            io_thread.join();
        }
    }

    void connect() {
        server.robot().mode = ROBOT_MODE_REMOTE;
        server.start_udp_status_pump(10);
        controller->connect().get();
        test::wait_for_connected(controller);
    }

    void reconnect() {
        std::exchange(controller, {})->disconnect();
        controller = std::make_shared<robot::YaskawaController>(io_ctx, make_config(ports.tcp_port));
        connect();
    }

    // Helper: build a waypoint list with a single target offset from zero.
    // The mock starts all joints at 0, so this moves each joint by `offset` radians.
    static std::list<Eigen::VectorXd> make_waypoints(double offset = 0.1) {
        Eigen::VectorXd target = Eigen::VectorXd::Constant(k_dof, offset);
        return {target};
    }

    // Helper: build a minimal 2-point trajectory: start at 0, end at offset.
    static std::vector<trajectory_point_t> make_samples(double offset = 0.1) {
        std::vector<trajectory_point_t> samples;

        trajectory_point_t start{};
        start.time_from_start = {0, 0};
        samples.push_back(start);

        trajectory_point_t end{};
        for (int i = 0; i < k_dof; ++i) {
            end.positions[i] = offset;
        }
        end.time_from_start = {1, 0};
        samples.push_back(end);

        return samples;
    }

    // Helper: build simple trajectory samples and issue execute_trajectory on the given group.
    std::unique_ptr<robot::GoalRequestHandle> do_move(uint32_t group_index = 0, double offset = 0.1) {
        test::drive_mock_to_ready(controller);
        return controller->execute_trajectory(group_index, k_dof, std::make_shared<robot::MoveStream>(make_samples(offset)), {}, 3.0);
    }

    // Helper: hand the same trajectory to the streamed path, so tests can drive extend/close/abort
    // on a move that is (or is about to be) in flight.
    robot::YaskawaController::streamed_move start_streamed_move(uint32_t group_index = 0, double offset = 0.1) {
        test::drive_mock_to_ready(controller);
        return controller->enqueue_streamed_move_request(group_index, k_dof, make_samples(offset), {}, 3.0);
    }

    // Helper: block until the mock reports an active goal, so a test can act on a move that is
    // genuinely running instead of racing the FSM's dispatch.
    bool wait_for_goal_active(uint32_t group_index = 0, std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (server.robot().groups[group_index].current_goal_state == GOAL_STATE_ACTIVE) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }
};

using ControllerFixture = TestFixture<>;
using GantryFixture = TestFixture<test::k_gantry_r1_b1>;
using DualArmFixture = TestFixture<test::k_dual_arm>;

// ==================== Suite 1: Basic Communication ====================

BOOST_AUTO_TEST_SUITE(basic_communication)

BOOST_FIXTURE_TEST_CASE(connect_disconnect, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    BOOST_CHECK_GE(server.robot().connection_count, 1U);
    controller->disconnect();
    // Allow server to notice the disconnect
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

BOOST_FIXTURE_TEST_CASE(heartbeat_running, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    BOOST_CHECK_GT(server.robot().last_heartbeat, 0);
}

BOOST_FIXTURE_TEST_CASE(servo_power_on, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    controller->turn_servo_power_on();
    BOOST_CHECK_EQUAL(server.robot().servo_power_on, 1);
}

BOOST_FIXTURE_TEST_CASE(reset_errors, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    // Put the mock into error state
    server.robot().in_error = true;
    server.robot().error_count = 1;
    server.robot().error_codes[0] = 99;
    // Wait for the status pump to propagate the error state to the controller
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    controller->reset_errors();
    BOOST_CHECK_EQUAL(server.robot().in_error, false);
    BOOST_CHECK_EQUAL(server.robot().error_count, 0);
}

BOOST_FIXTURE_TEST_CASE(set_motion_mode, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    controller->setMotionMode(1);
    BOOST_CHECK_EQUAL(server.robot().motion_mode, 1);
}

BOOST_FIXTURE_TEST_CASE(stop_motion, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    controller->turn_servo_power_on();
    controller->send_test_trajectory();
    BOOST_CHECK(server.robot().groups[0].in_motion);
    bool stopped = controller->stop(0);
    BOOST_CHECK(stopped);
    BOOST_CHECK(!server.robot().groups[0].in_motion);
}

BOOST_FIXTURE_TEST_CASE(test_trajectory_command, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    controller->turn_servo_power_on();
    controller->send_test_trajectory();
    BOOST_CHECK_EQUAL(server.robot().groups[0].trajectory_active, 1);
}

BOOST_FIXTURE_TEST_CASE(test_error_command, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    BOOST_CHECK_THROW(controller->send_test_error_command(), std::runtime_error);
}

BOOST_FIXTURE_TEST_CASE(get_robot_status, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    // Override the fixture's mode=REMOTE so we can observe the mock's default.
    server.robot().mode = 1;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto status = controller->get_robot_status();
    BOOST_CHECK_EQUAL(status.mode, 1);
    BOOST_CHECK(!status.e_stopped);
}

BOOST_FIXTURE_TEST_CASE(get_position_velocity_torque, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    // Set known positions on mock
    for (int i = 0; i < k_dof; ++i) {
        server.robot().groups[0].positions[i] = i * 10.0;
    }
    // Wait for pump to push the new values
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    auto pvt = controller->get_group_position_velocity_torque(0);
    for (int i = 0; i < k_dof; ++i) {
        BOOST_CHECK_CLOSE(pvt.position[static_cast<size_t>(i)], i * 10.0, 1e-6);
    }
}

BOOST_AUTO_TEST_SUITE_END()

// ==================== Suite 2: Goal/Trajectory Lifecycle ====================

BOOST_AUTO_TEST_SUITE(goal_trajectory_lifecycle)

BOOST_FIXTURE_TEST_CASE(move_simple, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    auto handle = do_move();
    auto state = handle->wait();
    BOOST_CHECK_EQUAL(state, GOAL_STATE_SUCCEEDED);
}

BOOST_FIXTURE_TEST_CASE(move_get_status, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    auto handle = do_move();
    auto status = handle->get_status().get();
    BOOST_CHECK(status.state == GOAL_STATE_ACTIVE || status.state == GOAL_STATE_SUCCEEDED);
    handle->wait();
}

BOOST_FIXTURE_TEST_CASE(move_cancel, ControllerFixture, *boost::unit_test::timeout(30)) {
    connect();
    // Use a larger offset so the trajectory takes longer, giving us time to cancel
    auto handle = do_move(0, 1.0);
    // Brief pause so the goal is accepted and active
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    handle->cancel();
    auto state = handle->wait();
    BOOST_CHECK(state == GOAL_STATE_CANCELLED || state == GOAL_STATE_ABORTED);
}

BOOST_FIXTURE_TEST_CASE(cancel_after_completion, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    auto handle = do_move();
    auto state = handle->wait();
    BOOST_CHECK_EQUAL(state, GOAL_STATE_SUCCEEDED);
    // Cancel after completion must not crash
    BOOST_CHECK_NO_THROW(handle->cancel());
}

BOOST_FIXTURE_TEST_CASE(full_move_lifecycle, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    auto handle = do_move();
    BOOST_CHECK_EQUAL(handle->wait(), GOAL_STATE_SUCCEEDED);
}

BOOST_AUTO_TEST_SUITE_END()

// ==================== Suite 2b: Streamed Move Lifecycle ====================
//
// These cover the producer/consumer contract between a streamed move and the goal monitor:
// priming, end-of-stream, aborting, and what a producer sees once the move is over.
//
// What they cannot cover is appending to an already-running goal. The real controller merges a
// second MSG_MOVE_GOAL into the active goal's queue; the mock rejects it outright
// (`mock_robot_accept_goal` refuses while a goal is ACTIVE), which is the same limitation the
// `move_large_offset` case notes. Every trajectory here therefore fits in the first chunk.
// That path has been exercised on hardware, including partial accepts, but has no automated
// coverage. Closing the gap means teaching the mock to merge an append into an ACTIVE goal.

BOOST_AUTO_TEST_SUITE(streamed_move_lifecycle)

// A streamed move has to be primed because the FSM worker thread dispatches the first chunk, and
// that thread also drives the heartbeat -- it must never block there waiting on a producer.
BOOST_FIXTURE_TEST_CASE(streamed_move_requires_priming, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    test::drive_mock_to_ready(controller);

    BOOST_CHECK_THROW(controller->enqueue_streamed_move_request(0, k_dof, {}, {}, 3.0), std::invalid_argument);
}

// One point is not a trajectory: it says where the motion starts, which is where the arm already
// is. The controller refuses to open a goal with fewer than `k_min_goal_points`, so priming with a
// single point has to be caught here rather than becoming a goal the firmware rejects.
BOOST_FIXTURE_TEST_CASE(streamed_move_rejects_single_point_prime, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    test::drive_mock_to_ready(controller);

    auto samples = make_samples();
    samples.resize(1);
    BOOST_REQUIRE_LT(samples.size(), robot::k_min_goal_points);

    BOOST_CHECK_THROW(controller->enqueue_streamed_move_request(0, k_dof, std::move(samples), {}, 3.0), std::invalid_argument);
}

// The mock reports a 4ms interpolation period, so it cannot consume points faster than 250 Hz.
// Sampling above that asks the controller for more points per cycle than it can execute, which
// would run the trajectory faster than it was timed for rather than fail loudly.
BOOST_FIXTURE_TEST_CASE(move_rejects_sampling_faster_than_interpolation, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    test::drive_mock_to_ready(controller);

    BOOST_CHECK_EQUAL(controller->interpolation_period(0).count(), 4000);
    BOOST_CHECK_THROW(controller->enqueue_streamed_move_request(0, k_dof, make_samples(), {}, 300.0), std::invalid_argument);

    auto samples = make_samples();
    BOOST_CHECK_THROW(controller->enqueue_move_request(0, k_dof, std::move(samples), {}, 300.0), std::invalid_argument);

    // Exactly at the limit is still executable, so it must not be rejected.
    BOOST_CHECK_NO_THROW(controller->enqueue_streamed_move_request(0, k_dof, make_samples(), {}, 250.0));
}

BOOST_FIXTURE_TEST_CASE(streamed_move_completes_after_close, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    auto move = start_streamed_move();

    BOOST_CHECK(move.stream->close());
    BOOST_REQUIRE(move.completion.wait_for(std::chrono::seconds(10)) == std::future_status::ready);
    BOOST_CHECK_NO_THROW(move.completion.get());

    // Everything we primed reached the controller as one goal.
    auto* echoed = mock_robot_echo_trajectory(&server.robot(), 0);
    BOOST_REQUIRE(echoed != nullptr);
    BOOST_CHECK_EQUAL(echoed->trajectory_size, 2U);
}

// The arm finishing the points it has is only the end of the move if the producer said the
// trajectory was over. Otherwise the arm ran out of trajectory early, which must be reported
// rather than passed off as a clean completion.
BOOST_FIXTURE_TEST_CASE(unclosed_stream_fails_when_the_arm_runs_out, ControllerFixture, *boost::unit_test::timeout(20)) {
    connect();
    auto move = start_streamed_move();

    // Never close the stream. The goal reaches SUCCEEDED with the producer still nominally
    // feeding it, and the monitor fails the move.
    BOOST_REQUIRE(move.completion.wait_for(std::chrono::seconds(15)) == std::future_status::ready);
    BOOST_CHECK_EXCEPTION(move.completion.get(), std::runtime_error, [](const std::runtime_error& ex) {
        return std::string(ex.what()).find("ended earlier than expected") != std::string::npos;
    });
}

// Aborting is how a producer reports that its move can never finish -- a cancelled RPC, or a
// malformed point later in the stream. The reason it gives is what the caller gets back, whether
// the abort lands before the FSM dispatches the move or after the goal is already running.
BOOST_FIXTURE_TEST_CASE(abort_fails_the_move_with_its_reason, ControllerFixture, *boost::unit_test::timeout(20)) {
    connect();
    auto move = start_streamed_move(0, 1.0);

    move.stream->abort("producer gave up");

    BOOST_REQUIRE(move.completion.wait_for(std::chrono::seconds(15)) == std::future_status::ready);
    BOOST_CHECK_EXCEPTION(move.completion.get(), std::runtime_error, [](const std::runtime_error& ex) {
        return std::string(ex.what()).find("producer gave up") != std::string::npos;
    });
}

// Aborting once the goal is running is the path that has to physically intervene: the monitor
// notices on its next tick, stops the arm, and fails the move. (The case above usually resolves
// before dispatch, where bailing out is all that is needed.)
BOOST_FIXTURE_TEST_CASE(abort_mid_flight_stops_the_arm, ControllerFixture, *boost::unit_test::timeout(20)) {
    connect();
    auto move = start_streamed_move(0, 1.0);

    BOOST_REQUIRE(wait_for_goal_active());
    BOOST_REQUIRE(server.robot().groups[0].in_motion);

    move.stream->abort("producer faulted mid-stream");

    BOOST_REQUIRE(move.completion.wait_for(std::chrono::seconds(15)) == std::future_status::ready);
    BOOST_CHECK_EXCEPTION(move.completion.get(), std::runtime_error, [](const std::runtime_error& ex) {
        return std::string(ex.what()).find("producer faulted mid-stream") != std::string::npos;
    });
    BOOST_CHECK(!server.robot().groups[0].in_motion);
}

// Once the move is over the stream is retired, so a producer that is still feeding it finds out
// from `extend`/`close` rather than by piling points into a buffer nobody will read.
BOOST_FIXTURE_TEST_CASE(producer_learns_the_move_is_over, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    auto move = start_streamed_move();

    BOOST_REQUIRE(move.stream->close());
    BOOST_REQUIRE(move.completion.wait_for(std::chrono::seconds(10)) == std::future_status::ready);
    BOOST_REQUIRE_NO_THROW(move.completion.get());

    BOOST_CHECK(move.stream->finished());
    BOOST_CHECK(!move.stream->extend(make_samples(0.2)));
    BOOST_CHECK(!move.stream->close());
}

// A streamed move is rejected up front for the same reasons a unary one is: the FSM, not the
// stream, decides whether the arm can move at all.
BOOST_FIXTURE_TEST_CASE(streamed_move_rejects_when_e_stopped, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    server.robot().e_stopped = true;
    test::wait_for_state(controller, "independent(");

    BOOST_CHECK_THROW(controller->enqueue_streamed_move_request(0, k_dof, make_samples(), {}, 3.0), std::runtime_error);
}

BOOST_AUTO_TEST_SUITE_END()

// ==================== Suite 3: Trajectory Corner Cases ====================

BOOST_AUTO_TEST_SUITE(trajectory_corner_cases)

BOOST_FIXTURE_TEST_CASE(move_to_current_position, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    // Target is (near) current position — move() should detect this and succeed immediately
    auto handle = do_move(0, 0.0);
    BOOST_CHECK_EQUAL(handle->wait(), GOAL_STATE_SUCCEEDED);
}

BOOST_FIXTURE_TEST_CASE(move_small_offset, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    auto handle = do_move(0, 0.05);
    BOOST_CHECK_EQUAL(handle->wait(), GOAL_STATE_SUCCEEDED);
}

BOOST_FIXTURE_TEST_CASE(move_large_offset, ControllerFixture, *boost::unit_test::timeout(30)) {
    connect();
    // Larger offset but still fits in one chunk (mock doesn't support streaming)
    auto handle = do_move(0, 0.5);
    BOOST_CHECK_EQUAL(handle->wait(), GOAL_STATE_SUCCEEDED);
}

BOOST_AUTO_TEST_SUITE_END()

// ==================== Suite 4: Connectivity Robustness ====================

BOOST_AUTO_TEST_SUITE(connectivity_robustness)

BOOST_AUTO_TEST_CASE(connection_refused, *boost::unit_test::timeout(15)) {
    auto ports = test::FakeServer::allocate_ports();

    boost::asio::io_context io_ctx;
    auto io_thread = std::thread([&io_ctx]() {
        auto guard = boost::asio::make_work_guard(io_ctx);
        io_ctx.run();
    });

    auto ctrl = std::make_shared<robot::YaskawaController>(io_ctx, make_config(ports.tcp_port));
    // The FSM connects asynchronously and retries indefinitely; with no server on the
    // configured port, it stays in state_disconnected_. Give it a few cycles to attempt
    // and fail at least once, then verify we never left disconnected.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    BOOST_CHECK(ctrl->describe_state().starts_with("disconnected"));
    ctrl->disconnect();

    io_ctx.stop();
    if (io_thread.joinable()) {
        io_thread.join();
    }
}

BOOST_FIXTURE_TEST_CASE(client_disconnect_reconnect, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    controller->turn_servo_power_on();
    BOOST_CHECK_EQUAL(server.robot().servo_power_on, 1);

    reconnect();

    controller->turn_servo_power_on();
    BOOST_CHECK_EQUAL(server.robot().servo_power_on, 1);
}

BOOST_AUTO_TEST_SUITE_END()

// ==================== Suite 5: Error Handling ====================

BOOST_AUTO_TEST_SUITE(error_handling)

BOOST_FIXTURE_TEST_CASE(error_propagation, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    try {
        controller->send_test_error_command();
        BOOST_FAIL("Expected exception");
    } catch (const std::runtime_error& e) {
        std::string msg = e.what();
        BOOST_CHECK(msg.find("MSG_TEST_ERROR_COMMAND") != std::string::npos || msg.find("error") != std::string::npos);
    }
}

BOOST_FIXTURE_TEST_CASE(error_reset_then_operate, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    // Trigger error
    BOOST_CHECK_THROW(controller->send_test_error_command(), std::runtime_error);

    // Wait for error state to propagate via UDP pump
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Reset and operate
    controller->reset_errors();
    // Wait for reset to propagate
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    controller->turn_servo_power_on();
    BOOST_CHECK_EQUAL(server.robot().servo_power_on, 1);

    controller->send_test_trajectory();
    BOOST_CHECK_EQUAL(server.robot().groups[0].trajectory_active, 1);
}

BOOST_FIXTURE_TEST_CASE(move_rejects_when_e_stopped, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    // Simulate e-stop and let UDP propagate so the FSM transitions to
    // `independent(estop, ...)`. The FSM's enqueue_move_request rejects from
    // any independent state with a human-required bit set (k_estop is one),
    // which is the production path for e-stop rejection. The IsReady() gate
    // inside turn_servo_power_on/setMotionMode was removed in this PR — the
    // FSM is now the single authority on whether a move can be issued.
    server.robot().e_stopped = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::vector<trajectory_point_t> samples;
    {
        trajectory_point_t start{};
        start.time_from_start = {0, 0};
        samples.push_back(start);
        trajectory_point_t end{};
        for (int i = 0; i < k_dof; ++i) {
            end.positions[i] = 0.1;
        }
        end.time_from_start = {1, 0};
        samples.push_back(end);
    }
    BOOST_CHECK_THROW(controller->enqueue_move_request(0, k_dof, std::move(samples), {}, 3.0), std::runtime_error);
}

BOOST_AUTO_TEST_SUITE_END()

// ==================== Suite 5b: FSM Wake-Up ====================

BOOST_AUTO_TEST_SUITE(fsm_wake_up)

// Force the FSM into state_independent_(k_in_error) and verify that enqueueing
// a move triggers the wake-up path: handle_move_request calls reset_errors,
// the mock clears in_error, upgrade_downgrade transitions to ready on the next
// tick, and the move dispatches and completes successfully.
//
// `enable_auto_error_recovery` is disabled so we exercise the move-triggered
// wake-up path in isolation rather than the passive 3-strikes reset path in
// upgrade_downgrade.
BOOST_AUTO_TEST_CASE(move_wakes_arm_from_in_error, *boost::unit_test::timeout(15)) {
    auto ports = test::FakeServer::allocate_ports();
    test::FakeServer server(ports);

    boost::asio::io_context io_ctx;
    auto io_thread = std::thread([&io_ctx]() {
        auto guard = boost::asio::make_work_guard(io_ctx);
        io_ctx.run();
    });

    viam::sdk::ProtoStruct attrs;
    attrs["host"] = viam::sdk::ProtoValue(std::string("127.0.0.1"));
    attrs["speed_rad_per_sec"] = viam::sdk::ProtoValue(1.0);
    attrs["acceleration_rad_per_sec2"] = viam::sdk::ProtoValue(1.0);
    attrs["tcp_port"] = viam::sdk::ProtoValue(static_cast<double>(ports.tcp_port));
    attrs["enable_auto_error_recovery"] = viam::sdk::ProtoValue(false);
    viam::sdk::ResourceConfig config(
        "arm", "test-arm", "", std::move(attrs), "rdk:component:arm", viam::sdk::Model("test", "test", "test"));

    server.robot().mode = ROBOT_MODE_REMOTE;
    server.start_udp_status_pump(10);
    auto controller = std::make_shared<robot::YaskawaController>(io_ctx, config);
    controller->connect().get();
    test::wait_for_connected(controller);
    // Drive the mock into a state the FSM will see as ready (mock starts with
    // drives_powered=0, motion_possible=0, so the FSM would otherwise stick in
    // independent(servo_off, motion_blocked)).
    test::drive_mock_to_ready(controller);
    BOOST_REQUIRE_EQUAL(controller->describe_state(), "ready");

    // Drop the arm into in_error and wait for the FSM to land in independent(in_error).
    server.robot().in_error = true;
    server.robot().error_count = 1;
    server.robot().error_codes[0] = 42;
    test::wait_for_state(controller, "independent(in_error");
    BOOST_TEST_INFO("state before move: " << controller->describe_state());
    BOOST_REQUIRE(controller->describe_state().starts_with("independent(in_error"));

    // Enqueue a move via the FSM. The wake-up path should fire, clear in_error
    // via reset_errors, transition to ready, and dispatch.
    std::vector<trajectory_point_t> samples;
    {
        trajectory_point_t start{};
        start.time_from_start = {0, 0};
        samples.push_back(start);
        trajectory_point_t end{};
        for (int i = 0; i < k_dof; ++i) {
            end.positions[i] = 0.1;
        }
        end.time_from_start = {1, 0};
        samples.push_back(end);
    }
    auto fut = controller->enqueue_move_request(0, k_dof, std::move(samples), {}, 3.0);
    BOOST_REQUIRE(fut.wait_for(std::chrono::seconds(5)) == std::future_status::ready);
    BOOST_CHECK_NO_THROW(fut.get());
    BOOST_CHECK_EQUAL(server.robot().in_error, false);

    controller->disconnect();
    io_ctx.stop();
    if (io_thread.joinable()) {
        io_thread.join();
    }
}

BOOST_AUTO_TEST_SUITE_END()

// ==================== Suite 6: Multi-Group Support ====================

BOOST_AUTO_TEST_SUITE(multi_group_support)

BOOST_FIXTURE_TEST_CASE(multi_group_server_construction, GantryFixture, *boost::unit_test::timeout(5)) {
    // Verify multi-group FakeServer stores correct group configuration
    BOOST_CHECK_EQUAL(server.num_groups(), 2);
    BOOST_CHECK_EQUAL(server.group_config(0).num_axes, 6);
    BOOST_CHECK_EQUAL(server.group_config(0).group_type, GROUP_TYPE_ROBOT);
    BOOST_CHECK_EQUAL(server.group_config(1).num_axes, 1);
    BOOST_CHECK_EQUAL(server.group_config(1).group_type, GROUP_TYPE_BASE);
}

BOOST_FIXTURE_TEST_CASE(capabilities_single_group, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    auto caps = controller->get_capabilities();
    BOOST_CHECK_EQUAL(caps.protocol_version, PROTOCOL_VERSION);
    BOOST_CHECK_EQUAL(caps.num_groups, 1);
    BOOST_CHECK_EQUAL(caps.groups[0].num_axes, 6);
    BOOST_CHECK_EQUAL(caps.groups[0].group_type, GROUP_TYPE_ROBOT);
}

BOOST_FIXTURE_TEST_CASE(capabilities_multi_group, GantryFixture, *boost::unit_test::timeout(15)) {
    connect();
    auto caps = controller->get_capabilities();
    BOOST_CHECK_EQUAL(caps.protocol_version, PROTOCOL_VERSION);
    BOOST_CHECK_EQUAL(caps.num_groups, 2);
    BOOST_CHECK_EQUAL(caps.groups[0].num_axes, 6);
    BOOST_CHECK_EQUAL(caps.groups[0].group_type, GROUP_TYPE_ROBOT);
    BOOST_CHECK_EQUAL(caps.groups[1].num_axes, 1);
    BOOST_CHECK_EQUAL(caps.groups[1].group_type, GROUP_TYPE_BASE);
}

BOOST_FIXTURE_TEST_CASE(multi_group_status_positions, GantryFixture, *boost::unit_test::timeout(15)) {
    // Set distinguishable positions per group
    for (int i = 0; i < 6; ++i) {
        server.robot().groups[0].positions[i] = (i + 1) * 1.0;
    }
    server.robot().groups[1].positions[0] = 10.0;

    connect();
    // Wait for pump to push status
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    auto pvt = controller->get_group_position_velocity_torque(0);
    BOOST_CHECK_GT(pvt.num_axes, static_cast<uint8_t>(0));
}

BOOST_FIXTURE_TEST_CASE(multi_group_robot_status, GantryFixture, *boost::unit_test::timeout(15)) {
    connect();
    // Override the fixture's mode=REMOTE so we can observe the mock's default.
    server.robot().mode = 1;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto status = controller->get_robot_status();
    // Controller-wide robot status (not per-group)
    BOOST_CHECK_EQUAL(status.mode, 1);
    BOOST_CHECK(!status.e_stopped);
}

BOOST_AUTO_TEST_SUITE_END()

// ==================== Suite 6.5: Group Validation Cache (RSDK-13930) ====================

BOOST_AUTO_TEST_SUITE(group_validation_cache)

// Known group_index passes validation: the cache was populated from the capabilities response
// at connect time, and the lookup finds the entry without a roundtrip. `stop` is used as a
// probe because it's the simplest group-keyed call the mock fully supports; the test only
// cares that validate_group_ doesn't throw.
BOOST_FIXTURE_TEST_CASE(known_group_passes, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    BOOST_CHECK_NO_THROW(controller->stop(0));
}

// Unknown group_index throws a structured error including the known-groups list. Critically
// the message comes from the client-side validate_group_ before any TCP request fires — so
// the server's generic VIAM_ERROR_INVALID_PAYLOAD doesn't surface.
BOOST_FIXTURE_TEST_CASE(unknown_group_throws_structured, ControllerFixture, *boost::unit_test::timeout(15)) {
    connect();
    try {
        controller->stop(99);
        BOOST_FAIL("expected validate_group_ to throw on unknown group_index");
    } catch (const std::runtime_error& ex) {
        const std::string what = ex.what();
        BOOST_CHECK(what.find("group_index 99 is not configured") != std::string::npos);
        BOOST_CHECK(what.find("known groups: 0") != std::string::npos);
    }
}

// Multi-group cache: gantry fixture advertises group 0 (robot) and group 1 (base track).
// Both should pass validation; anything else should not.
BOOST_FIXTURE_TEST_CASE(multi_group_cache, GantryFixture, *boost::unit_test::timeout(15)) {
    connect();
    BOOST_CHECK_NO_THROW(controller->stop(0));
    BOOST_CHECK_NO_THROW(controller->stop(1));
    BOOST_CHECK_THROW(controller->stop(2), std::runtime_error);
}

// Cache-clearing-on-disconnect and cache-repopulate-on-reconnect were here, but neither
// can observe the property via the public API now that `check_connected_()` gates first:
// after disconnect the connection gate throws before any cache lookup, and the populate
// path is verified transitively by `known_group_passes` (which would fail if connect's
// cache populate were broken). The clear-on-disconnect invariant is enforced structurally
// in `disconnect()`; a runtime test that could observe it would have to either lean on
// the deprecated `checkGroupIndex` shim or set up a reconnect-with-different-server
// scenario — both larger than the property warrants.

BOOST_AUTO_TEST_SUITE_END()

// ==================== Suite 7: Dual-Arm Motion ====================

BOOST_AUTO_TEST_SUITE(dual_arm_motion)

// Scenario 1: Sequential moves on two different groups
BOOST_FIXTURE_TEST_CASE(sequential_dual_group_moves, DualArmFixture, *boost::unit_test::timeout(30)) {
    connect();

    // Move group 0
    auto handle0 = do_move(0, 0.1);
    BOOST_CHECK_EQUAL(handle0->wait(), GOAL_STATE_SUCCEEDED);

    // Move group 1
    auto handle1 = do_move(1, 0.1);
    BOOST_CHECK_EQUAL(handle1->wait(), GOAL_STATE_SUCCEEDED);
}

// Scenario 2: Parallel moves on two different groups (must NOT throw "move in progress")
BOOST_FIXTURE_TEST_CASE(parallel_dual_group_moves, DualArmFixture, *boost::unit_test::timeout(30)) {
    connect();
    controller->turn_servo_power_on();
    controller->setMotionMode(1);

    auto handle0 = do_move(0, 0.2);
    auto handle1 = do_move(1, 0.2);

    BOOST_CHECK_EQUAL(handle0->wait(), GOAL_STATE_SUCCEEDED);
    BOOST_CHECK_EQUAL(handle1->wait(), GOAL_STATE_SUCCEEDED);
}

// Scenario 3: Partial cancel — cancel group 0, group 1 continues
BOOST_FIXTURE_TEST_CASE(partial_cancel_dual_group, DualArmFixture, *boost::unit_test::timeout(30)) {
    connect();
    controller->turn_servo_power_on();
    controller->setMotionMode(1);

    // Use larger offsets so trajectories take longer, giving us time to cancel
    auto handle0 = do_move(0, 1.0);
    auto handle1 = do_move(1, 1.0);

    // Wait briefly then cancel group 0
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    handle0->cancel();

    auto state0 = handle0->wait();
    BOOST_CHECK(state0 == GOAL_STATE_CANCELLED || state0 == GOAL_STATE_ABORTED);

    // Group 1 should complete successfully
    BOOST_CHECK_EQUAL(handle1->wait(), GOAL_STATE_SUCCEEDED);
}

// Scenario 4: Full disconnect during dual-group motion
BOOST_FIXTURE_TEST_CASE(disconnect_during_dual_group_motion, DualArmFixture, *boost::unit_test::timeout(30)) {
    connect();
    controller->turn_servo_power_on();
    controller->setMotionMode(1);

    auto handle0 = do_move(0, 1.0);
    auto handle1 = do_move(1, 1.0);

    // Let both goals start
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Disconnect — should not crash or hang
    controller->disconnect();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Reconnect and verify clean state
    reconnect();
    controller->turn_servo_power_on();
    BOOST_CHECK_EQUAL(server.robot().servo_power_on, 1);

    // New move should work
    auto handle_new = do_move(0, 0.1);
    BOOST_CHECK_EQUAL(handle_new->wait(), GOAL_STATE_SUCCEEDED);
}

BOOST_AUTO_TEST_SUITE_END()
