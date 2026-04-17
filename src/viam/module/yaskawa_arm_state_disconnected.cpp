#include "yaskawa_arm_state.hpp"

#include <viam/sdk/log/logging.hpp>

using namespace viam::sdk;

// ---------------------------------------------------------------
// state_disconnected_ constructors
// ---------------------------------------------------------------

YaskawaArm::state_::state_disconnected_::state_disconnected_(event_connection_lost_ triggering_event)
    : triggering_event_(std::make_unique<event_connection_lost_>(std::move(triggering_event))) {}

// ---------------------------------------------------------------
// state_disconnected_ identity
// ---------------------------------------------------------------

std::string_view YaskawaArm::state_::state_disconnected_::name() {
    return "disconnected";
}

std::string YaskawaArm::state_::state_disconnected_::describe() const {
    return std::string(name());
}

std::chrono::milliseconds YaskawaArm::state_::state_disconnected_::get_timeout() const {
    if (pending_connection_) {
        return std::chrono::milliseconds{50};
    }
    return std::chrono::milliseconds{1000};
}

// ---------------------------------------------------------------
// state_disconnected_ cycle
// ---------------------------------------------------------------

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_disconnected_::recv_arm_data(state_&) {
    return std::nullopt;
}

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_disconnected_::upgrade_downgrade(state_&) {
    return std::nullopt;
}

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_disconnected_::send_heartbeat(state_&) {
    return std::nullopt;
}

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_disconnected_::handle_move_request(state_& state) const {
    if (state.move_request_) {
        state.move_request_->complete_error("arm is disconnected");
        state.move_request_.reset();
    }
    return std::nullopt;
}

// ---------------------------------------------------------------
// state_disconnected_ transitions
// ---------------------------------------------------------------

std::optional<YaskawaArm::state_::state_variant_> YaskawaArm::state_::state_disconnected_::handle_event(
    event_connection_established_ event) {
    VIAM_SDK_LOG(info) << "connection established, entering independent state";
    const blocking_mask all_bits = blocking_reason::k_in_error | blocking_reason::k_servo_off | blocking_reason::k_motion_blocked |
                                   blocking_reason::k_major_alarm | blocking_reason::k_estop | blocking_reason::k_not_remote;
    return state_independent_{std::move(event.controller), all_bits};
}

std::optional<YaskawaArm::state_::state_variant_> YaskawaArm::state_::state_disconnected_::handle_event(event_connection_lost_ /*event*/) {
    return std::nullopt;
}

// ---------------------------------------------------------------
// state_disconnected_ connect_ (stub — implemented in Step 3)
// ---------------------------------------------------------------

std::shared_ptr<YaskawaController> YaskawaArm::state_::state_disconnected_::connect_(state_&) {
    throw std::runtime_error("not implemented");
}
