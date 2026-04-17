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

// NOLINTBEGIN(readability-convert-member-functions-to-static)
std::string YaskawaArm::state_::state_ready_::describe() const {
    return std::string(name());
}
// NOLINTEND(readability-convert-member-functions-to-static)

// ---------------------------------------------------------------
// state_ready_ cycle (stubs — implemented in Step 5)
// ---------------------------------------------------------------

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_ready_::upgrade_downgrade(state_&) {
    const auto status = controller_->get_robot_status();

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

    if (mask != 0) {
        return event_blocking_detected_{mask};
    }

    return std::nullopt;
}

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_ready_::handle_move_request(state_& state) {
    if (!state.move_request_) {
        return std::nullopt;
    }

    auto& req = *state.move_request_;

    if (!req.handle) {
        req.handle = controller_->move(req.waypoints, req.unix_time, req.velocity, req.acceleration);
        return std::nullopt;
    }

    if (!req.handle->is_done()) {
        return std::nullopt;
    }

    try {
        req.handle->wait_for(std::chrono::milliseconds(0));
        req.complete_success();
    } catch (const std::exception& ex) {
        req.complete_error(ex.what());
    }
    state.move_request_.reset();

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
