#include "robot_socket.hpp"

#include <viam/sdk/log/logging.hpp>

using namespace robot;
using namespace viam::sdk;

// ---------------------------------------------------------------
// state_ready_ identity
// ---------------------------------------------------------------

std::string_view YaskawaController::state_::state_ready_::name() {
    return "ready";
}

// NOLINTBEGIN(readability-convert-member-functions-to-static)
std::string YaskawaController::state_::state_ready_::describe() const {
    return std::string(name());
}
// NOLINTEND(readability-convert-member-functions-to-static)

// ---------------------------------------------------------------
// state_ready_ cycle
// ---------------------------------------------------------------

// NOLINTBEGIN(readability-convert-member-functions-to-static)
std::optional<YaskawaController::state_::event_variant_> YaskawaController::state_::state_ready_::upgrade_downgrade(state_& state) {
    const auto status = state.controller_->get_robot_status();

    not_ready_mask mask = 0;
    if (status.e_stopped) {
        mask = mask | not_ready_reason::k_estop;
    }
    if (status.mode != ROBOT_MODE_REMOTE) {
        mask = mask | not_ready_reason::k_not_remote;
    }
    if (status.in_error) {
        mask = mask | not_ready_reason::k_in_error;
    }
    if (!status.drives_powered) {
        mask = mask | not_ready_reason::k_servo_off;
    }
    if (!status.motion_possible) {
        mask = mask | not_ready_reason::k_motion_blocked;
    }

    if (mask != 0) {
        return event_not_ready_detected_{mask};
    }

    return std::nullopt;
}

std::optional<YaskawaController::state_::event_variant_> YaskawaController::state_::state_ready_::handle_move_request(state_& state) {
    // TODO(PR #54): pass req.group_index to move() once the API accepts it.
    for (auto it = state.move_requests_.begin(); it != state.move_requests_.end();) {
        auto& req = *it;
        if (!req.handle) {
            try {
                req.handle = state.controller_->move(req.waypoints, req.unix_time, req.velocity, req.acceleration);
            } catch (const std::exception& ex) {
                req.complete_error(ex.what());
                it = state.move_requests_.erase(it);
                continue;
            }
            ++it;
        } else if (req.handle->is_done()) {
            try {
                std::ignore = req.handle->wait_for(std::chrono::milliseconds(0));
                req.complete_success();
            } catch (const std::exception& ex) {
                req.complete_error(ex.what());
            }
            it = state.move_requests_.erase(it);
        } else {
            ++it;
        }
    }
    return std::nullopt;
}

// ---------------------------------------------------------------
// state_ready_ transitions
// ---------------------------------------------------------------

std::optional<YaskawaController::state_::state_variant_> YaskawaController::state_::state_ready_::handle_event(
    state_& state, event_connection_lost_ event) {
    state.controller_->disconnect();
    return state_disconnected_{std::move(event)};
}

std::optional<YaskawaController::state_::state_variant_> YaskawaController::state_::state_ready_::handle_event(
    state_&, event_not_ready_detected_ event) {
    VIAM_SDK_LOG(warn) << "not-ready condition detected in ready state, entering independent state";
    // In-flight move_requests_ are not cancelled here. They are cancelled on the next
    // handle_move_request_() call, which runs after upgrade_downgrade_() in the same cycle.
    return state_independent_{event.mask};
}
// NOLINTEND(readability-convert-member-functions-to-static)
