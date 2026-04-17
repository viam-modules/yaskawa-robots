#include "yaskawa_arm_state.hpp"

#include <viam/sdk/log/logging.hpp>

using namespace viam::sdk;

// ---------------------------------------------------------------
// state_ready_ constructor
// ---------------------------------------------------------------

YaskawaArm::state_::state_ready_::state_ready_(std::shared_ptr<YaskawaController> controller) : state_connected_(std::move(controller)) {}

// ---------------------------------------------------------------
// state_ready_ identity
// ---------------------------------------------------------------

std::string_view YaskawaArm::state_::state_ready_::name() {
    return "ready";
}

std::string YaskawaArm::state_::state_ready_::describe() const {
    return std::string(name());
}

// ---------------------------------------------------------------
// state_ready_ cycle (stubs — implemented in Step 5)
// ---------------------------------------------------------------

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_ready_::upgrade_downgrade(state_&) {
    return std::nullopt;
}

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_ready_::handle_move_request(state_&) {
    return std::nullopt;
}

// ---------------------------------------------------------------
// state_ready_ transitions
// ---------------------------------------------------------------

std::optional<YaskawaArm::state_::state_variant_> YaskawaArm::state_::state_ready_::handle_event(event_connection_lost_ event) {
    if (controller_) {
        controller_->disconnect();
    }
    return state_disconnected_{std::move(event)};
}

std::optional<YaskawaArm::state_::state_variant_> YaskawaArm::state_::state_ready_::handle_event(event_blocking_detected_ event) {
    VIAM_SDK_LOG(warn) << "blocking condition detected in ready state, entering independent state";
    return state_independent_{std::move(controller_), event.mask};
}
