#include "robot_socket.hpp"

#include <format>

using namespace robot;

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
        LOGGING(info) << "[fsm] not-ready mask changed: `" << describe() << "` -> `independent("
                      << YaskawaController::state_::describe_not_ready_mask_(mask) << ")`";
        recovery_attempts_ = 0;
        wakeup_attempted_ = false;
        wakeup_grace_cycles_remaining_ = 0;
        return event_not_ready_detected_{mask};
    }

    // Auto-recovery: skip if any human-required condition is present.
    //
    // Only `in_error` is recovered automatically — `reset_errors` clears software state on the
    // controller without physically energizing anything. `servo_off` and `motion_blocked` are
    // left for explicit user action (a move request) to wake the arm. Auto-toggling servo power
    // would fight an operator who deliberately powered the arm down (panel button, idle timeout
    // after `stop()`, etc.).
    if (mask & ~k_auto_recoverable_mask) {
        return std::nullopt;
    }

    if (mask & k_in_error && state.controller_->enable_auto_error_recovery_) {
        constexpr int k_max_reset_attempts = 3;
        if (recovery_attempts_ < k_max_reset_attempts) {
            state.controller_->reset_errors();
            ++recovery_attempts_;
        } else {
            return event_not_ready_detected_{static_cast<not_ready_mask>((mask & ~k_in_error) | k_major_alarm)};
        }
    }

    return std::nullopt;
}

std::optional<YaskawaController::state_::event_variant_> YaskawaController::state_::state_independent_::handle_move_request(state_& state) {
    // Any in-flight handle was started when we were in state_ready_ and got interrupted by the
    // transition into independent. Cancel and complete-error each one regardless of `reasons_`
    // — the move can't survive the transition either way.
    for (auto it = state.move_requests_.begin(); it != state.move_requests_.end();) {
        auto& req = *it;
        if (!req.handle) {
            ++it;
            continue;
        }
        if (!req.handle->is_done()) {
            try {
                req.handle->cancel();
            } catch (const std::exception& ex) {
                LOGGING(warning) << "[fsm] exception while cancelling move request on entering independent state: " << ex.what();
            } catch (...) {
                LOGGING(warning) << "[fsm] unknown exception while cancelling move request on entering independent state";
            }
        }
        req.complete_error(std::format("move interrupted: arm transitioned to independent({})",
                                       YaskawaController::state_::describe_not_ready_mask_(reasons_)));
        it = state.move_requests_.erase(it);
    }

    // Any pending (un-dispatched) requests with human-required bits set cannot be recovered
    // here — fail them. (enqueue_move_request rejects up-front, but reasons_ can have gained a
    // human-required bit between enqueue and dispatch.)
    if (reasons_ & ~k_auto_recoverable_mask) {
        auto requests = std::exchange(state.move_requests_, {});
        for (auto& req : requests) {
            req.complete_error(
                std::format("cannot move: arm is independent({})", YaskawaController::state_::describe_not_ready_mask_(reasons_)));
        }
        return std::nullopt;
    }

    // Pending requests with only auto-recoverable bits remaining — wake the arm. A move request
    // is explicit user intent, so we run the recovery steps regardless of the controller's
    // `enable_auto_error_recovery` config (which only gates the passive path in upgrade_downgrade).
    // Bits clear asynchronously via UDP status updates; on the next cycle, upgrade_downgrade
    // transitions to ready and state_ready_::handle_move_request dispatches the move.
    if (state.move_requests_.empty()) {
        return std::nullopt;
    }
    // Fire each recovery call exactly once, then wait up to `k_wakeup_grace_cycles` FSM cycles
    // for the controller's status to reflect the cleared bits (propagation is over UDP, so the
    // updated mask may not arrive on the very next cycle). If the grace period elapses with any
    // auto-recoverable bit still set, fail the pending requests instead of retrying — callers
    // shouldn't hang on an arm that won't recover (e.g. servo refuses to stay on, motion mode
    // can't be set). The wake-up state resets in upgrade_downgrade when `reasons_` changes.
    constexpr int k_wakeup_grace_cycles = 10;  // ~1s at the 100ms FSM tick
    if (!wakeup_attempted_) {
        // Set attempted BEFORE the calls so a throw partway through (e.g. a TCP error mid-
        // sequence) doesn't cause earlier successful calls to re-fire on the next tick. The
        // FSM run loop catches the exception; the next cycle then enters the grace-wait
        // branch and either the bits clear or we fail the move cleanly.
        wakeup_attempted_ = true;
        wakeup_grace_cycles_remaining_ = k_wakeup_grace_cycles;
        if (reasons_ & k_in_error) {
            state.controller_->reset_errors();
        }
        if (reasons_ & k_servo_off) {
            state.controller_->turn_servo_power_on();
        }
        if (reasons_ & k_motion_blocked) {
            state.controller_->setMotionMode(1);
        }
    } else if (wakeup_grace_cycles_remaining_ > 0) {
        --wakeup_grace_cycles_remaining_;
    } else {
        auto requests = std::exchange(state.move_requests_, {});
        for (auto& req : requests) {
            req.complete_error(
                std::format("wake-up failed: not-ready bits did not clear within grace period; "
                            "arm is independent({})",
                            YaskawaController::state_::describe_not_ready_mask_(reasons_)));
        }
        return std::nullopt;
    }
    return std::nullopt;
}

// ---------------------------------------------------------------
// state_independent_ transitions
// ---------------------------------------------------------------

// NOLINTBEGIN(readability-convert-member-functions-to-static)
std::optional<YaskawaController::state_::state_variant_> YaskawaController::state_::state_independent_::handle_event(
    state_&, event_connection_lost_ event) {
    return state_disconnected_{std::move(event)};
}

std::optional<YaskawaController::state_::state_variant_> YaskawaController::state_::state_independent_::handle_event(
    state_&, event_not_ready_detected_ event) {
    reasons_ = event.mask;
    return std::nullopt;
}

std::optional<YaskawaController::state_::state_variant_> YaskawaController::state_::state_independent_::handle_event(
    state_&, event_ready_detected_) {
    LOGGING(info) << "[fsm] all not-ready conditions cleared, entering ready state";
    return state_ready_{};
}
// NOLINTEND(readability-convert-member-functions-to-static)
