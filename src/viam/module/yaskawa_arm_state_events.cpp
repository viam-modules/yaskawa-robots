#include "yaskawa_arm_state.hpp"

// ---------------------------------------------------------------
// event_connection_established_
// ---------------------------------------------------------------

std::string_view YaskawaArm::state_::event_connection_established_::name() {
    return "connection_established";
}

std::string_view YaskawaArm::state_::event_connection_established_::describe() const {
    return name();
}

// ---------------------------------------------------------------
// event_connection_lost_
// ---------------------------------------------------------------

YaskawaArm::state_::event_connection_lost_::event_connection_lost_(reason r) : reason_code_(r) {}

YaskawaArm::state_::event_connection_lost_ YaskawaArm::state_::event_connection_lost_::tcp_failure() {
    return event_connection_lost_{reason::k_tcp_failure};
}

YaskawaArm::state_::event_connection_lost_ YaskawaArm::state_::event_connection_lost_::heartbeat_failure() {
    return event_connection_lost_{reason::k_heartbeat_failure};
}

YaskawaArm::state_::event_connection_lost_ YaskawaArm::state_::event_connection_lost_::module_shutdown() {
    return event_connection_lost_{reason::k_module_shutdown};
}

std::string_view YaskawaArm::state_::event_connection_lost_::name() {
    return "connection_lost";
}

std::string_view YaskawaArm::state_::event_connection_lost_::describe() const {
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
// event_blocking_detected_
// ---------------------------------------------------------------

std::string_view YaskawaArm::state_::event_blocking_detected_::name() {
    return "blocking_detected";
}

std::string_view YaskawaArm::state_::event_blocking_detected_::describe() const {
    return name();
}

// ---------------------------------------------------------------
// event_ready_detected_
// ---------------------------------------------------------------

std::string_view YaskawaArm::state_::event_ready_detected_::name() {
    return "ready_detected";
}

std::string_view YaskawaArm::state_::event_ready_detected_::describe() const {
    return name();
}
