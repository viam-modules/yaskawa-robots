#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <span>
#include <string_view>
#include <thread>
#include <vector>

extern "C" {
#include "fault_inject.h"
#include "mock_robot.h"
#include "robot_protocol.h"
}

// Forward-decl avoids dragging the full robot_socket.hpp into every test that includes
// fake_server.hpp. Call sites that use `wait_for_connected` must include robot_socket.hpp
// themselves (they do already — `is_disconnected()` is a member call).
namespace robot {
class YaskawaController;
}

namespace test {

struct ServerPorts {
    uint16_t tcp_port;
    uint16_t udp_port;
};

// Per-group configuration for multi-group FakeServer construction.
struct GroupConfig {
    uint8_t num_axes;
    uint8_t group_type;  // group_type_t: GROUP_TYPE_ROBOT=0, GROUP_TYPE_BASE=1, GROUP_TYPE_STATION=2
};

// Pre-defined group layouts for common test configurations
inline constexpr GroupConfig k_single_robot[] = {{6, GROUP_TYPE_ROBOT}};
inline constexpr GroupConfig k_gantry_r1_b1[] = {{6, GROUP_TYPE_ROBOT}, {1, GROUP_TYPE_BASE}};
inline constexpr GroupConfig k_dual_arm[] = {{6, GROUP_TYPE_ROBOT}, {6, GROUP_TYPE_ROBOT}};

// Passed as user_data to the fault-injecting command wrapper so the C callback
// can reach both the fault context and the underlying mock robot.
struct FaultCallbackData {
    fault_inject_ctx_t* fault_ctx;
    mock_robot_t* robot;
};

class FakeServer {
   public:
    explicit FakeServer(ServerPorts ports, uint32_t connection_timeout_ms = 30000);
    FakeServer(ServerPorts ports, std::span<const GroupConfig> groups, uint32_t connection_timeout_ms = 30000);
    ~FakeServer();

    FakeServer(const FakeServer&) = delete;
    FakeServer& operator=(const FakeServer&) = delete;

    mock_robot_t& robot();
    robot_server_ctx_t* ctx();
    ServerPorts ports() const;

    uint8_t num_groups() const;
    const GroupConfig& group_config(uint8_t index) const;

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
    void init_server(uint32_t connection_timeout_ms);

    // C callback that checks fault rules before delegating to mock_robot_handle_command
    static command_response_context_t* fault_injecting_handle_command(protocol_header_t* header, void* payload, void* user_data);

    ServerPorts ports_;
    mock_robot_t robot_;
    robot_server_ctx_t* server_ctx_;
    fault_inject_ctx_t fault_ctx_;
    FaultCallbackData cb_data_;
    std::atomic<bool> pumping_{false};
    std::thread pump_thread_;

    // Multi-group configuration (stored for accessors and future pump changes)
    std::vector<GroupConfig> groups_;
};

// Block until the controller's FSM leaves state_disconnected_, or `timeout` elapses.
// Public methods on the controller throw via `check_connected_()` until the FSM
// transition fires; tests setting up a controller need to wait for that before issuing
// any group- or controller-keyed call. A trailing 20ms settle covers the per-group
// status pump being slightly behind the FSM transition itself.
void wait_for_connected(const std::shared_ptr<robot::YaskawaController>& controller,
                        std::chrono::milliseconds timeout = std::chrono::seconds(2));

// Block until `controller->describe_state()` starts with `prefix`, or `timeout` elapses.
// Tests that drive the mock through transitions use this to converge on a state
// deterministically instead of sleeping a fixed duration. No exception on timeout —
// callers should follow up with an assertion if they need to fail loudly.
void wait_for_state(const std::shared_ptr<robot::YaskawaController>& controller,
                    std::string_view prefix,
                    std::chrono::milliseconds timeout = std::chrono::seconds(3));

// Convenience: drive the controller from the mock's default (servo_off + motion_blocked)
// to `state_ready_` by issuing the production setup sequence and waiting for the FSM to
// converge. Used by tests that need a ready arm before dispatching a move via the
// direct `execute_trajectory` path.
void drive_mock_to_ready(const std::shared_ptr<robot::YaskawaController>& controller,
                         std::chrono::milliseconds timeout = std::chrono::seconds(2));

}  // namespace test
