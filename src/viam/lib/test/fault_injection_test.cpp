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

extern "C" {
#include "protocol.h"
}

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
        // Wait for at least one UDP status to arrive and update State
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

// Test 2: Server becomes unresponsive when it receives MSG_TEST_TRAJECTORY_COMMAND.
// The fault blocks the server thread for ~5s before responding. Verify the
// delay is observable (operation takes >3s instead of being near-instant).
BOOST_FIXTURE_TEST_CASE(server_unresponsive_during_move, FaultFixture, *boost::unit_test::timeout(15)) {
    connect();
    controller->turn_servo_power_on();

    // Server will block for ~5s when it receives the trajectory command
    server.inject_unresponsive_on(MSG_TEST_TRAJECTORY_COMMAND);

    auto start = std::chrono::steady_clock::now();
    controller->send_test_trajectory();
    auto elapsed = std::chrono::steady_clock::now() - start;

    // The operation succeeded but was delayed by the fault
    BOOST_CHECK_GT(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(), 3);
}

// Test 3: Server becomes unresponsive on MSG_TURN_SERVO_POWER_ON.
// The fault blocks the server thread for ~5s before responding. Verify the
// delay is observable.
BOOST_FIXTURE_TEST_CASE(server_unresponsive, FaultFixture, *boost::unit_test::timeout(15)) {
    connect();

    // FSM's auto-recovery may have already turned servo power on during the transition into
    // ready state. Reset the baseline so we can observe whether the fault below intercepts
    // the command.
    server.robot().servo_power_on = 0;

    server.inject_unresponsive_on(MSG_TURN_SERVO_POWER_ON);

    auto start = std::chrono::steady_clock::now();
    controller->turn_servo_power_on();
    auto elapsed = std::chrono::steady_clock::now() - start;

    // The operation completed but was delayed by the fault (~5s instead of instant).
    BOOST_CHECK_GT(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(), 3);
    // servo_power_on is NOT set — the fault intercepted the command and returned OK
    // without delegating to the mock robot handler. Verify the interception worked.
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

// Test 6: Client disconnects while a trajectory is active. Verify the server
// detects the disconnect and accepts a new connection.
BOOST_FIXTURE_TEST_CASE(client_disconnect_during_active_goal, FaultFixture, *boost::unit_test::timeout(15)) {
    connect();
    controller->turn_servo_power_on();

    controller->send_test_trajectory();
    BOOST_CHECK(server.robot().in_motion);

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
