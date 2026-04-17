#include "yaskawa_arm_state.hpp"

#include <format>

#include <viam/sdk/log/logging.hpp>

using namespace viam::sdk;

// ---------------------------------------------------------------
// state_independent_ constructor
// ---------------------------------------------------------------

YaskawaArm::state_::state_independent_::state_independent_(std::shared_ptr<YaskawaController> controller, blocking_mask reasons)
    : state_connected_(std::move(controller)), reasons_(reasons) {}

// ---------------------------------------------------------------
// state_independent_ identity
// ---------------------------------------------------------------

std::string_view YaskawaArm::state_::state_independent_::name() {
    return "independent";
}

std::string YaskawaArm::state_::state_independent_::describe() const {
    return std::format("independent({})", describe_blocking_mask_(reasons_));
}

// ---------------------------------------------------------------
// state_independent_ cycle (stubs — implemented in Step 4)
// ---------------------------------------------------------------

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_independent_::upgrade_downgrade(state_&) {
    return std::nullopt;
}

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_independent_::handle_move_request(state_& state) const {
    if (state.move_request_) {
        state.move_request_->complete_error(std::format("cannot move: arm is independent({})", describe_blocking_mask_(reasons_)));
        state.move_request_.reset();
    }
    return std::nullopt;
}

// ---------------------------------------------------------------
// state_independent_ transitions
// ---------------------------------------------------------------

std::optional<YaskawaArm::state_::state_variant_> YaskawaArm::state_::state_independent_::handle_event(event_connection_lost_ event) {
    if (controller_) {
        controller_->disconnect();
    }
    return state_disconnected_{std::move(event)};
}

std::optional<YaskawaArm::state_::state_variant_> YaskawaArm::state_::state_independent_::handle_event(event_blocking_detected_ event) {
    reasons_ = event.mask;
    return std::nullopt;
}

std::optional<YaskawaArm::state_::state_variant_> YaskawaArm::state_::state_independent_::handle_event(event_ready_detected_) {
    VIAM_SDK_LOG(info) << "all blocking conditions cleared, entering ready state";
    return state_ready_{std::move(controller_)};
}
