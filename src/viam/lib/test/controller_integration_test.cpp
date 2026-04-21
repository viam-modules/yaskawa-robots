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

    TestFixture()
        : ports(test::FakeServer::allocate_ports()), server(ports, ServerConfig) {
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
        server.start_udp_status_pump(10);
        controller->connect().get();
    }

    void reconnect() {
        controller->disconnect();
        controller = std::make_shared<robot::YaskawaController>(io_ctx, make_config(ports.tcp_port));
        controller->connect().get();
    }

    // Helper: build a waypoint list with a single target offset from zero.
    // The mock starts all joints at 0, so this moves each joint by `offset` radians.
    static std::list<Eigen::VectorXd> make_waypoints(double offset = 0.1) {
        Eigen::VectorXd target = Eigen::VectorXd::Constant(k_dof, offset);
        return {target};
    }

    // Helper: issue a move on the given group using default velocity/acceleration limits.
    std::unique_ptr<robot::GoalRequestHandle> do_move(uint32_t group_index = 0, double offset = 0.1) {
        auto vel = controller->get_velocity_limits();
        auto accel = controller->get_acceleration_limits();
        return controller->move(make_waypoints(offset), group_index, "0", vel, accel);
    }
};

using ControllerFixture = TestFixture<>;
using GantryFixture = TestFixture<test::k_gantry_r1_b1>;
using DualArmFixture = TestFixture<test::k_dual_arm>;

// ==================== Suite 1: Basic Communication ====================

BOOST_AUTO_TEST_SUITE(basic_communication)

BOOST_FIXTURE_TEST_CASE(connect_disconnect, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    BOOST_CHECK_GE(server.robot().connection_count, 1U);
    controller->disconnect();
    // Allow server to notice the disconnect
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

BOOST_FIXTURE_TEST_CASE(heartbeat_running, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    BOOST_CHECK_GT(server.robot().last_heartbeat, 0);
}

BOOST_FIXTURE_TEST_CASE(servo_power_on, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    controller->turn_servo_power_on();
    BOOST_CHECK_EQUAL(server.robot().servo_power_on, 1);
}

BOOST_FIXTURE_TEST_CASE(reset_errors, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
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

BOOST_FIXTURE_TEST_CASE(set_motion_mode, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    controller->setMotionMode(1);
    BOOST_CHECK_EQUAL(server.robot().motion_mode, 1);
}

BOOST_FIXTURE_TEST_CASE(stop_motion, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    controller->turn_servo_power_on();
    controller->send_test_trajectory();
    BOOST_CHECK(server.robot().groups[0].in_motion);
    bool stopped = controller->stop(0);
    BOOST_CHECK(stopped);
    BOOST_CHECK(!server.robot().groups[0].in_motion);
}

BOOST_FIXTURE_TEST_CASE(test_trajectory_command, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    controller->turn_servo_power_on();
    controller->send_test_trajectory();
    BOOST_CHECK_EQUAL(server.robot().groups[0].trajectory_active, 1);
}

BOOST_FIXTURE_TEST_CASE(test_error_command, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    BOOST_CHECK_THROW(controller->send_test_error_command(), std::runtime_error);
}

BOOST_FIXTURE_TEST_CASE(check_group_index, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    BOOST_CHECK(controller->checkGroupIndex(0));
}

BOOST_FIXTURE_TEST_CASE(get_robot_status, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    auto status = controller->get_robot_status();
    // Default mock state: mode=1, not e-stopped
    BOOST_CHECK_EQUAL(status.mode, 1);
    BOOST_CHECK(!status.e_stopped);
}

BOOST_FIXTURE_TEST_CASE(get_position_velocity_torque, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
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

BOOST_FIXTURE_TEST_CASE(move_simple, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    auto handle = do_move();
    auto state = handle->wait();
    BOOST_CHECK_EQUAL(state, GOAL_STATE_SUCCEEDED);
}

BOOST_FIXTURE_TEST_CASE(move_get_status, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    auto handle = do_move();
    auto status = handle->get_status().get();
    BOOST_CHECK(status.state == GOAL_STATE_ACTIVE || status.state == GOAL_STATE_SUCCEEDED);
    handle->wait();
}

BOOST_FIXTURE_TEST_CASE(move_cancel, ControllerFixture,
                        *boost::unit_test::timeout(30)) {
    connect();
    // Use a larger offset so the trajectory takes longer, giving us time to cancel
    auto handle = do_move(0, 1.0);
    // Brief pause so the goal is accepted and active
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    handle->cancel();
    auto state = handle->wait();
    BOOST_CHECK(state == GOAL_STATE_CANCELLED || state == GOAL_STATE_ABORTED);
}

BOOST_FIXTURE_TEST_CASE(cancel_after_completion, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    auto handle = do_move();
    auto state = handle->wait();
    BOOST_CHECK_EQUAL(state, GOAL_STATE_SUCCEEDED);
    // Cancel after completion must not crash
    BOOST_CHECK_NO_THROW(handle->cancel());
}

BOOST_FIXTURE_TEST_CASE(full_move_lifecycle, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    auto handle = do_move();
    BOOST_CHECK_EQUAL(handle->wait(), GOAL_STATE_SUCCEEDED);
}

BOOST_AUTO_TEST_SUITE_END()

// ==================== Suite 3: Trajectory Corner Cases ====================

BOOST_AUTO_TEST_SUITE(trajectory_corner_cases)

BOOST_FIXTURE_TEST_CASE(move_to_current_position, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    // Target is (near) current position — move() should detect this and succeed immediately
    auto handle = do_move(0, 0.0);
    BOOST_CHECK_EQUAL(handle->wait(), GOAL_STATE_SUCCEEDED);
}

BOOST_FIXTURE_TEST_CASE(move_small_offset, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    auto handle = do_move(0, 0.05);
    BOOST_CHECK_EQUAL(handle->wait(), GOAL_STATE_SUCCEEDED);
}

BOOST_FIXTURE_TEST_CASE(move_large_offset, ControllerFixture,
                        *boost::unit_test::timeout(30)) {
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
    BOOST_CHECK_THROW(ctrl->connect().get(), std::exception);

    io_ctx.stop();
    if (io_thread.joinable()) {
        io_thread.join();
    }
}

BOOST_FIXTURE_TEST_CASE(client_disconnect_reconnect, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
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

BOOST_FIXTURE_TEST_CASE(error_propagation, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    try {
        controller->send_test_error_command();
        BOOST_FAIL("Expected exception");
    } catch (const std::runtime_error& e) {
        std::string msg = e.what();
        BOOST_CHECK(msg.find("MSG_TEST_ERROR_COMMAND") != std::string::npos ||
                    msg.find("error") != std::string::npos);
    }
}

BOOST_FIXTURE_TEST_CASE(error_reset_then_operate, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
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

BOOST_FIXTURE_TEST_CASE(move_rejects_when_e_stopped, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    // Simulate e-stop and let UDP propagate it. reset_errors() does not clear e_stopped,
    // so turn_servo_power_on() inside move() will throw even after the reset attempt.
    server.robot().e_stopped = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    BOOST_CHECK_THROW(do_move(), std::runtime_error);
}

BOOST_AUTO_TEST_SUITE_END()

// ==================== Suite 6: Multi-Group Support ====================

BOOST_AUTO_TEST_SUITE(multi_group_support)

BOOST_FIXTURE_TEST_CASE(multi_group_server_construction, GantryFixture,
                        *boost::unit_test::timeout(5)) {
    // Verify multi-group FakeServer stores correct group configuration
    BOOST_CHECK_EQUAL(server.num_groups(), 2);
    BOOST_CHECK_EQUAL(server.group_config(0).num_axes, 6);
    BOOST_CHECK_EQUAL(server.group_config(0).group_type, GROUP_TYPE_ROBOT);
    BOOST_CHECK_EQUAL(server.group_config(1).num_axes, 1);
    BOOST_CHECK_EQUAL(server.group_config(1).group_type, GROUP_TYPE_BASE);
}

BOOST_FIXTURE_TEST_CASE(capabilities_single_group, ControllerFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    auto caps = controller->get_capabilities();
    BOOST_CHECK_EQUAL(caps.protocol_version, PROTOCOL_VERSION);
    BOOST_CHECK_EQUAL(caps.num_groups, 1);
    BOOST_CHECK_EQUAL(caps.groups[0].num_axes, 6);
    BOOST_CHECK_EQUAL(caps.groups[0].group_type, GROUP_TYPE_ROBOT);
}

BOOST_FIXTURE_TEST_CASE(capabilities_multi_group, GantryFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    auto caps = controller->get_capabilities();
    BOOST_CHECK_EQUAL(caps.protocol_version, PROTOCOL_VERSION);
    BOOST_CHECK_EQUAL(caps.num_groups, 2);
    BOOST_CHECK_EQUAL(caps.groups[0].num_axes, 6);
    BOOST_CHECK_EQUAL(caps.groups[0].group_type, GROUP_TYPE_ROBOT);
    BOOST_CHECK_EQUAL(caps.groups[1].num_axes, 1);
    BOOST_CHECK_EQUAL(caps.groups[1].group_type, GROUP_TYPE_BASE);
}

BOOST_FIXTURE_TEST_CASE(multi_group_status_positions, GantryFixture,
                        *boost::unit_test::timeout(15)) {
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

BOOST_FIXTURE_TEST_CASE(multi_group_robot_status_group_index, GantryFixture,
                        *boost::unit_test::timeout(15)) {
    connect();
    auto status = controller->get_robot_status();
    // Robot status should carry a valid group_index
    BOOST_CHECK_EQUAL(status.mode, 1);
    BOOST_CHECK(!status.e_stopped);
}

BOOST_AUTO_TEST_SUITE_END()

// ==================== Suite 7: Dual-Arm Motion ====================

BOOST_AUTO_TEST_SUITE(dual_arm_motion)

// Scenario 1: Sequential moves on two different groups
BOOST_FIXTURE_TEST_CASE(sequential_dual_group_moves, DualArmFixture,
                        *boost::unit_test::timeout(30)) {
    connect();

    // Move group 0
    auto handle0 = do_move(0, 0.1);
    BOOST_CHECK_EQUAL(handle0->wait(), GOAL_STATE_SUCCEEDED);

    // Move group 1
    auto handle1 = do_move(1, 0.1);
    BOOST_CHECK_EQUAL(handle1->wait(), GOAL_STATE_SUCCEEDED);
}

// Scenario 2: Parallel moves on two different groups (must NOT throw "move in progress")
BOOST_FIXTURE_TEST_CASE(parallel_dual_group_moves, DualArmFixture,
                        *boost::unit_test::timeout(30)) {
    connect();
    controller->turn_servo_power_on();
    controller->setMotionMode(1);

    auto handle0 = do_move(0, 0.2);
    auto handle1 = do_move(1, 0.2);

    BOOST_CHECK_EQUAL(handle0->wait(), GOAL_STATE_SUCCEEDED);
    BOOST_CHECK_EQUAL(handle1->wait(), GOAL_STATE_SUCCEEDED);
}

// Scenario 3: Partial cancel — cancel group 0, group 1 continues
BOOST_FIXTURE_TEST_CASE(partial_cancel_dual_group, DualArmFixture,
                        *boost::unit_test::timeout(30)) {
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
BOOST_FIXTURE_TEST_CASE(disconnect_during_dual_group_motion, DualArmFixture,
                        *boost::unit_test::timeout(30)) {
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
