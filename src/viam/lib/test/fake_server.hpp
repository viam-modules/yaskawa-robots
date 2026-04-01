#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

extern "C" {
#include "fault_inject.h"
#include "mock_robot.h"
#include "robot_protocol.h"
}

namespace test {

struct ServerPorts {
    uint16_t tcp_port;
    uint16_t udp_port;
};

// Passed as user_data to the fault-injecting command wrapper so the C callback
// can reach both the fault context and the underlying mock robot.
struct FaultCallbackData {
    fault_inject_ctx_t* fault_ctx;
    mock_robot_t* robot;
};

class FakeServer {
   public:
    explicit FakeServer(ServerPorts ports, uint32_t connection_timeout_ms = 30000);
    ~FakeServer();

    FakeServer(const FakeServer&) = delete;
    FakeServer& operator=(const FakeServer&) = delete;

    mock_robot_t& robot();
    robot_server_ctx_t* ctx();
    ServerPorts ports() const;

    void start_udp_status_pump(uint32_t interval_ms = 10);
    void stop_udp_status_pump();

    // Fault injection convenience wrappers
    void inject_disconnect_after(uint32_t n);
    void inject_unresponsive_on(uint8_t msg_type);
    void inject_close_connection();
    void inject_rule(const fault_rule_t& rule);
    void reset_faults();
    fault_inject_ctx_t& faults();

    static ServerPorts allocate_ports();

   private:
    // C callback that checks fault rules before delegating to mock_robot_handle_command
    static command_response_context_t* fault_injecting_handle_command(
        protocol_header_t* header, void* payload, void* user_data);

    ServerPorts ports_;
    mock_robot_t robot_;
    robot_server_ctx_t* server_ctx_;
    fault_inject_ctx_t fault_ctx_;
    FaultCallbackData cb_data_;
    std::atomic<bool> pumping_{false};
    std::thread pump_thread_;
};

}  // namespace test
