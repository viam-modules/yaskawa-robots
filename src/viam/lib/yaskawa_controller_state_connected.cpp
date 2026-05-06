#include "robot_socket.hpp"

using namespace robot;

// `state_connected_` is *not* a state the FSM ever holds in `current_state_`. It's a shared
// base for the two connected sub-states (state_independent_ and state_ready_), holding logic
// that's identical across them: heartbeat cadence and the heartbeat handler. Each sub-state
// inherits via `using state_connected_::foo;` and overrides only what differs.

// ---------------------------------------------------------------
// state_connected_ cycle
// ---------------------------------------------------------------

// NOLINTBEGIN(readability-convert-member-functions-to-static)
std::chrono::milliseconds YaskawaController::state_::state_connected_::get_timeout() const {
    return std::chrono::milliseconds{100};
}
// NOLINTEND(readability-convert-member-functions-to-static)

std::optional<YaskawaController::state_::event_variant_> YaskawaController::state_::state_connected_::send_heartbeat(state_& state) {
    try {
        state.controller_->send_heartbeat();
    } catch (const std::exception& ex) {
        LOGGING(warning) << "[fsm] heartbeat failed: " << ex.what();
        return event_connection_lost_::heartbeat_failure();
    }
    return std::nullopt;
}
