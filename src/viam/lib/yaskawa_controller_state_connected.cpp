#include "robot_socket.hpp"

using namespace robot;

// See robot_socket.hpp for state_connected_'s role (shared base for the connected sub-states).

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
