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
    const auto status = controller_->get_robot_status();

    // Compute blocking mask from wire-observable fields.
    blocking_mask mask = 0;
    if (status.e_stopped) {
        mask = mask | blocking_reason::k_estop;
    }
    if (status.mode != ROBOT_MODE_REMOTE) {
        mask = mask | blocking_reason::k_not_remote;
    }
    if (status.in_error) {
        mask = mask | blocking_reason::k_in_error;
    }
    if (!status.drives_powered) {
        mask = mask | blocking_reason::k_servo_off;
    }
    if (!status.motion_possible) {
        mask = mask | blocking_reason::k_motion_blocked;
    }

    // Preserve a previously diagnosed major alarm while in_error persists on the wire.
    // k_major_alarm replaces k_in_error once reset_errors() has been exhausted.
    if (has_reason(reasons_, blocking_reason::k_major_alarm) && has_reason(mask, blocking_reason::k_in_error)) {
        mask = (mask & ~static_cast<blocking_mask>(blocking_reason::k_in_error)) | blocking_reason::k_major_alarm;
    }

    if (mask == 0) {
        return event_ready_detected_{};
    }

    if (mask != reasons_) {
        recovery_attempts_ = 0;
        return event_blocking_detected_{mask};
    }

    // Auto-recovery: skip if any human-required condition is present.
    if (mask & ~k_auto_recoverable_mask) {
        return std::nullopt;
    }

    if (has_reason(mask, blocking_reason::k_in_error)) {
        // Attempt reset; after k_max_reset_attempts failures, diagnose as major alarm.
        constexpr int k_max_reset_attempts = 3;
        if (recovery_attempts_ < k_max_reset_attempts) {
            controller_->reset_errors();
            ++recovery_attempts_;
        } else {
            return event_blocking_detected_{(mask & ~static_cast<blocking_mask>(blocking_reason::k_in_error)) |
                                            blocking_reason::k_major_alarm};
        }
    } else {
        // Not in error — drive servo on and enable motion.
        if (has_reason(mask, blocking_reason::k_servo_off)) {
            controller_->turn_servo_power_on();
        }
        if (has_reason(mask, blocking_reason::k_motion_blocked)) {
            controller_->setMotionMode(1);
        }
    }

    return std::nullopt;
}

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_independent_::handle_move_request(state_& state) const {
    if (state.move_request_) {
        if (state.move_request_->handle && !state.move_request_->handle->is_done()) {
            state.move_request_->handle->cancel();
        }
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
