#include "robot_socket.hpp"

using namespace robot;

// NOLINTBEGIN(readability-convert-member-functions-to-static)

// ---------------------------------------------------------------
// event_connection_established_
// ---------------------------------------------------------------

std::string_view YaskawaController::state_::event_connection_established_::name() {
    return "connection_established";
}

std::string_view YaskawaController::state_::event_connection_established_::describe() const {
    return name();
}

// ---------------------------------------------------------------
// event_connection_lost_
// ---------------------------------------------------------------

YaskawaController::state_::event_connection_lost_::event_connection_lost_(reason r) : reason_code_(r) {}

YaskawaController::state_::event_connection_lost_ YaskawaController::state_::event_connection_lost_::tcp_failure() {
    return event_connection_lost_{reason::k_tcp_failure};
}

YaskawaController::state_::event_connection_lost_ YaskawaController::state_::event_connection_lost_::heartbeat_failure() {
    return event_connection_lost_{reason::k_heartbeat_failure};
}

YaskawaController::state_::event_connection_lost_ YaskawaController::state_::event_connection_lost_::module_shutdown() {
    return event_connection_lost_{reason::k_module_shutdown};
}

std::string_view YaskawaController::state_::event_connection_lost_::name() {
    return "connection_lost";
}

std::string_view YaskawaController::state_::event_connection_lost_::describe() const {
    switch (reason_code_) {
        case reason::k_tcp_failure:
            return "connection_lost(tcp_failure)";
        case reason::k_heartbeat_failure:
            return "connection_lost(heartbeat_failure)";
        case reason::k_module_shutdown:
            return "connection_lost(module_shutdown)";
    }
    return "connection_lost(unknown)";
}

// ---------------------------------------------------------------
// event_not_ready_detected_
// ---------------------------------------------------------------

std::string_view YaskawaController::state_::event_not_ready_detected_::name() {
    return "not_ready_detected";
}

std::string_view YaskawaController::state_::event_not_ready_detected_::describe() const {
    return name();
}

// ---------------------------------------------------------------
// event_ready_detected_
// ---------------------------------------------------------------

std::string_view YaskawaController::state_::event_ready_detected_::name() {
    return "ready_detected";
}

std::string_view YaskawaController::state_::event_ready_detected_::describe() const {
    return name();
}
// NOLINTEND(readability-convert-member-functions-to-static)
