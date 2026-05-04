#include "robot_socket.hpp"

#include <format>

#include <viam/sdk/log/logging.hpp>

using namespace robot;
using namespace viam::sdk;

// ---------------------------------------------------------------
// state_independent_ constructor
// ---------------------------------------------------------------

YaskawaController::state_::state_independent_::state_independent_(not_ready_mask reasons) : reasons_(reasons) {}

// ---------------------------------------------------------------
// state_independent_ identity
// ---------------------------------------------------------------

std::string_view YaskawaController::state_::state_independent_::name() {
    return "independent";
}

std::string YaskawaController::state_::state_independent_::describe() const {
    return std::format("independent({})", YaskawaController::state_::describe_not_ready_mask_(reasons_));
}

// ---------------------------------------------------------------
// state_independent_ cycle
// ---------------------------------------------------------------

std::optional<YaskawaController::state_::event_variant_> YaskawaController::state_::state_independent_::upgrade_downgrade(state_& state) {
    const auto status = state.controller_->get_robot_status();

    not_ready_mask mask = 0;
    if (status.e_stopped) {
        mask = mask | k_estop;
    }
    if (status.mode != ROBOT_MODE_REMOTE) {
        mask = mask | k_not_remote;
    }
    if (status.in_error) {
        mask = mask | k_in_error;
    }
    if (!status.drives_powered) {
        mask = mask | k_servo_off;
    }
    if (!status.motion_possible) {
        mask = mask | k_motion_blocked;
    }

    // Preserve a previously diagnosed major alarm while in_error persists on the wire.
    if ((reasons_ & k_major_alarm) && (mask & k_in_error)) {
        mask = (mask & ~k_in_error) | k_major_alarm;
    }

    if (mask == 0) {
        return event_ready_detected_{};
    }

    if (mask != reasons_) {
        recovery_attempts_ = 0;
        return event_not_ready_detected_{mask};
    }

    // Auto-recovery: skip if any human-required condition is present.
    if (mask & ~k_auto_recoverable_mask) {
        return std::nullopt;
    }

    if (mask & k_in_error) {
        constexpr int k_max_reset_attempts = 3;
        if (recovery_attempts_ < k_max_reset_attempts) {
            state.controller_->reset_errors();
            ++recovery_attempts_;
        } else {
            return event_not_ready_detected_{static_cast<not_ready_mask>((mask & ~k_in_error) | k_major_alarm)};
        }
    } else {
        if (mask & k_servo_off) {
            state.controller_->turn_servo_power_on();
        }
        if (mask & k_motion_blocked) {
            state.controller_->setMotionMode(1);
        }
    }

    return std::nullopt;
}

std::optional<YaskawaController::state_::event_variant_> YaskawaController::state_::state_independent_::handle_move_request(
    state_& state) const {
    auto requests = std::exchange(state.move_requests_, {});
    for (auto& req : requests) {
        if (req.handle && !req.handle->is_done()) {
            try {
                req.handle->cancel();
            } catch (...) {
                VIAM_SDK_LOG(warn) << "[fsm] exception while cancelling move request on entering independent state";
            }
        }
        req.complete_error(
            std::format("cannot move: arm is independent({})", YaskawaController::state_::describe_not_ready_mask_(reasons_)));
    }
    return std::nullopt;
}

// ---------------------------------------------------------------
// state_independent_ transitions
// ---------------------------------------------------------------

// NOLINTBEGIN(readability-convert-member-functions-to-static)
std::optional<YaskawaController::state_::state_variant_> YaskawaController::state_::state_independent_::handle_event(
    state_&, event_connection_lost_ event) {
    // state_disconnected_::connect_() tears down stale sockets before reconnecting.
    return state_disconnected_{std::move(event)};
}

std::optional<YaskawaController::state_::state_variant_> YaskawaController::state_::state_independent_::handle_event(
    state_&, event_not_ready_detected_ event) {
    reasons_ = event.mask;
    return std::nullopt;
}

std::optional<YaskawaController::state_::state_variant_> YaskawaController::state_::state_independent_::handle_event(
    state_&, event_ready_detected_) {
    VIAM_SDK_LOG(info) << "[fsm] all not-ready conditions cleared, entering ready state";
    return state_ready_{};
}
// NOLINTEND(readability-convert-member-functions-to-static)
