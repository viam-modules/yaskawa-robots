#define BOOST_TEST_MODULE FaultInjectionTest

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>
#pragma GCC diagnostic pop
#include <chrono>
#include <csignal>
#include <memory>
#include <thread>

#include <viam/sdk/config/resource.hpp>

// Ignore SIGPIPE — writes to closed sockets return EPIPE instead of killing the process
struct SigpipeIgnorer {
    SigpipeIgnorer() {
        std::signal(SIGPIPE, SIG_IGN);
    }
};
static SigpipeIgnorer s_ignore_sigpipe;

#include "../robot_socket.hpp"
#include "fake_server.hpp"

#include "protocol.h"

viam::sdk::ResourceConfig make_test_config(uint16_t tcp_port) {
    viam::sdk::ProtoStruct attrs;
    attrs["host"] = viam::sdk::ProtoValue(std::string("127.0.0.1"));
    attrs["tcp_port"] = viam::sdk::ProtoValue(static_cast<double>(tcp_port));
    attrs["speed_rad_per_sec"] = viam::sdk::ProtoValue(1.0);
    attrs["acceleration_rad_per_sec2"] = viam::sdk::ProtoValue(1.0);
    return viam::sdk::ResourceConfig(
        "arm", "test-arm", "", std::move(attrs), "rdk:component:arm", viam::sdk::Model("test", "test", "test"));
}

struct FaultFixture {
    boost::asio::io_context io_ctx;
    std::thread io_thread;
    test::ServerPorts ports;
    test::FakeServer server;
    std::shared_ptr<robot::YaskawaController> controller;

    FaultFixture() : ports(test::FakeServer::allocate_ports()), server(ports) {
        io_thread = std::thread([this]() {
            auto guard = boost::asio::make_work_guard(io_ctx);
            io_ctx.run();
        });
        controller = std::make_shared<robot::YaskawaController>(io_ctx, make_test_config(ports.tcp_port));
    }

    ~FaultFixture() {
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

    void make_new_controller() {
        controller = std::make_shared<robot::YaskawaController>(io_ctx, make_test_config(ports.tcp_port));
    }
};

BOOST_AUTO_TEST_SUITE(fault_injection)

// Test 1: Server disconnects during idle. The controller's heartbeat thread
// detects the failure and auto-reconnects. Verify the server observes the
// disconnect/reconnect cycle and that the controller remains operational.
BOOST_FIXTURE_TEST_CASE(server_disconnect_during_idle, FaultFixture, *boost::unit_test::timeout(15)) {
    connect();
    uint32_t initial_connections = server.robot().connection_count;

    // One-shot disconnect on the next heartbeat
    fault_rule_t rule{};
    rule.fault = FAULT_DISCONNECT;
    rule.trigger = TRIGGER_ON_MESSAGE_TYPE;
    rule.trigger_message_type = MSG_HEARTBEAT;
    rule.one_shot = true;
    server.inject_rule(rule);

    // Wait for heartbeat to trigger the fault and for auto-reconnect
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Server should have seen at least one disconnection and a reconnection
    BOOST_CHECK_GE(server.robot().disconnection_count, 1U);
    BOOST_CHECK_GT(server.robot().connection_count, initial_connections);

    // Controller should still be operational after auto-reconnect
    BOOST_CHECK_NO_THROW(controller->turn_servo_power_on());
    BOOST_CHECK_EQUAL(server.robot().servo_power_on, 1);
}

// Test 2: Server becomes unresponsive when it receives MSG_TEST_TRAJECTORY_COMMAND. The fault
// blocks the server thread for ~5s, which is as long as we are willing to wait for any request, so
// the caller gives up instead of sitting there. What matters is that the wait is bounded by us and
// not by the controller: a controller that never answers used to block the caller forever, and
// when that caller was the FSM worker thread it took teardown down with it.
BOOST_FIXTURE_TEST_CASE(server_unresponsive_during_move, FaultFixture, *boost::unit_test::timeout(30)) {
    connect();
    controller->turn_servo_power_on();

    // Server will block for ~5s when it receives the trajectory command
    server.inject_unresponsive_on(MSG_TEST_TRAJECTORY_COMMAND);

    auto start = std::chrono::steady_clock::now();
    BOOST_CHECK_THROW(controller->send_test_trajectory(), std::runtime_error);
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);

    // It waited for the timeout, and no longer than that.
    BOOST_CHECK_GE(elapsed.count(), 4);
    BOOST_CHECK_LE(elapsed.count(), 10);
}

// Test 3: Same, on MSG_TURN_SERVO_POWER_ON.
BOOST_FIXTURE_TEST_CASE(server_unresponsive, FaultFixture, *boost::unit_test::timeout(30)) {
    connect();

    server.inject_unresponsive_on(MSG_TURN_SERVO_POWER_ON);

    auto start = std::chrono::steady_clock::now();
    BOOST_CHECK_THROW(controller->turn_servo_power_on(), std::runtime_error);
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);

    BOOST_CHECK_GE(elapsed.count(), 4);
    BOOST_CHECK_LE(elapsed.count(), 10);
    // servo_power_on is NOT set — the fault intercepted the command without delegating to the mock
    // robot handler. Verify the interception worked.
    BOOST_CHECK_EQUAL(server.robot().servo_power_on, 0);
}

// Test 4: Server abruptly closes the connection. The heartbeat thread detects
// the failure and auto-reconnects. Verify the server sees the cycle.
BOOST_FIXTURE_TEST_CASE(server_abrupt_close, FaultFixture, *boost::unit_test::timeout(15)) {
    connect();
    uint32_t initial_connections = server.robot().connection_count;

    // One-shot close on next heartbeat
    fault_rule_t rule{};
    rule.fault = FAULT_CLOSE_CONNECTION;
    rule.trigger = TRIGGER_ON_MESSAGE_TYPE;
    rule.trigger_message_type = MSG_HEARTBEAT;
    rule.one_shot = true;
    server.inject_rule(rule);

    // Wait for heartbeat to trigger the fault and for auto-reconnect
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Server should have seen disconnect + reconnect
    BOOST_CHECK_GE(server.robot().disconnection_count, 1U);
    BOOST_CHECK_GT(server.robot().connection_count, initial_connections);

    // Controller should still work after auto-reconnect
    BOOST_CHECK_NO_THROW(controller->turn_servo_power_on());
    BOOST_CHECK_EQUAL(server.robot().servo_power_on, 1);
}

// Test 5: After a server fault fires and the controller auto-reconnects,
// verify the server accepts the new connection and operations work normally.
BOOST_FIXTURE_TEST_CASE(reconnect_after_server_fault, FaultFixture, *boost::unit_test::timeout(15)) {
    connect();

    // One-shot disconnect
    fault_rule_t rule{};
    rule.fault = FAULT_DISCONNECT;
    rule.trigger = TRIGGER_IMMEDIATE;
    rule.one_shot = true;
    server.inject_rule(rule);

    // Wait for it to fire and auto-reconnect
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Verify the controller auto-reconnected and works
    BOOST_CHECK_GE(server.robot().disconnection_count, 1U);
    BOOST_CHECK_GE(server.robot().connection_count, 2U);
    BOOST_CHECK_NO_THROW(controller->turn_servo_power_on());
    BOOST_CHECK_EQUAL(server.robot().servo_power_on, 1);
}

// Teardown must not wait on the controller. We stall the server inside the request the FSM worker
// thread makes every cycle, then disconnect. Closing the socket is what fails that request, so it
// has to happen before we join the worker, otherwise we are joining a thread that is waiting on a
// socket we have not closed yet. A controller that answers nothing at all makes that wait
// permanent, which is how a reconfigure used to hang the whole module.
BOOST_FIXTURE_TEST_CASE(disconnect_does_not_wait_on_a_stalled_controller, FaultFixture, *boost::unit_test::timeout(30)) {
    connect();

    // The fault blocks the server's TCP thread for ~5s once the heartbeat arrives.
    server.inject_unresponsive_on(MSG_HEARTBEAT);
    // The connected states run a cycle every 100ms, so this lands the worker inside the stall.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    const auto start = std::chrono::steady_clock::now();
    controller->disconnect();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);

    BOOST_TEST_MESSAGE("disconnect took " << elapsed.count() << "ms");
    BOOST_CHECK_LT(elapsed.count(), 2000);
    BOOST_CHECK(controller->is_disconnected());
}

// Test 6: Client disconnects while a trajectory is active. Verify the server
// detects the disconnect and accepts a new connection.
BOOST_FIXTURE_TEST_CASE(client_disconnect_during_active_goal, FaultFixture, *boost::unit_test::timeout(15)) {
    connect();
    controller->turn_servo_power_on();

    controller->send_test_trajectory();
    BOOST_CHECK(server.robot().groups[0].in_motion);

    // Client-initiated disconnect
    controller->disconnect();

    // Wait for server to notice
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    BOOST_CHECK_GE(server.robot().disconnection_count, 1U);

    // Reconnect with a fresh controller — server should accept
    make_new_controller();
    connect();
    BOOST_CHECK_GE(server.robot().connection_count, 2U);
}

BOOST_AUTO_TEST_SUITE_END()
