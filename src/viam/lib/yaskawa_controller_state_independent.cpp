#include "robot_socket.hpp"

#include <format>

#include <viam/sdk/log/logging.hpp>

using namespace robot;
using namespace viam::sdk;

// ---------------------------------------------------------------
// state_independent_ constructor
// ---------------------------------------------------------------

YaskawaController::state_::state_independent_::state_independent_(blocking_mask reasons) : reasons_(reasons) {}

// ---------------------------------------------------------------
// state_independent_ identity
// ---------------------------------------------------------------

std::string_view YaskawaController::state_::state_independent_::name() {
    return "independent";
}

std::string YaskawaController::state_::state_independent_::describe() const {
    return std::format("independent({})", YaskawaController::state_::describe_blocking_mask_(reasons_));
}

// ---------------------------------------------------------------
// state_independent_ cycle
// ---------------------------------------------------------------

std::optional<YaskawaController::state_::event_variant_> YaskawaController::state_::state_independent_::upgrade_downgrade(state_& state) {
    const auto status = state.controller_->get_robot_status();

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
        constexpr int k_max_reset_attempts = 3;
        if (recovery_attempts_ < k_max_reset_attempts) {
            state.controller_->reset_errors();
            ++recovery_attempts_;
        } else {
            return event_blocking_detected_{(mask & ~static_cast<blocking_mask>(blocking_reason::k_in_error)) |
                                            blocking_reason::k_major_alarm};
        }
    } else {
        if (has_reason(mask, blocking_reason::k_servo_off)) {
            state.controller_->turn_servo_power_on();
        }
        if (has_reason(mask, blocking_reason::k_motion_blocked)) {
            state.controller_->setMotionMode(1);
        }
    }

    return std::nullopt;
}

std::optional<YaskawaController::state_::event_variant_> YaskawaController::state_::state_independent_::handle_move_request(
    state_& state) const {
    for (auto& req : state.move_requests_) {
        if (req.handle && !req.handle->is_done()) {
            req.handle->cancel();
        }
        req.complete_error(
            std::format("cannot move: arm is independent({})", YaskawaController::state_::describe_blocking_mask_(reasons_)));
    }
    state.move_requests_.clear();
    return std::nullopt;
}

// ---------------------------------------------------------------
// state_independent_ transitions
// ---------------------------------------------------------------

// NOLINTBEGIN(readability-convert-member-functions-to-static)
std::optional<YaskawaController::state_::state_variant_> YaskawaController::state_::state_independent_::handle_event(
    state_& state, event_connection_lost_ event) {
    state.controller_->disconnect();
    return state_disconnected_{std::move(event)};
}

std::optional<YaskawaController::state_::state_variant_> YaskawaController::state_::state_independent_::handle_event(
    state_&, event_blocking_detected_ event) {
    reasons_ = event.mask;
    return std::nullopt;
}

std::optional<YaskawaController::state_::state_variant_> YaskawaController::state_::state_independent_::handle_event(
    state_&, event_ready_detected_) {
    VIAM_SDK_LOG(info) << "all blocking conditions cleared, entering ready state";
    return state_ready_{};
}
// NOLINTEND(readability-convert-member-functions-to-static)
