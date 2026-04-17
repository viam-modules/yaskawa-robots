#include "yaskawa_arm_state.hpp"

#include <utility>

#include <viam/sdk/log/logging.hpp>

using namespace viam::sdk;

// NOLINTBEGIN(readability-convert-member-functions-to-static)

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

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_disconnected_::upgrade_downgrade(state_& state) {
    if (pending_connection_) {
        switch (pending_connection_->wait_for(std::chrono::seconds(0))) {
            case std::future_status::ready:
                return event_connection_established_{std::exchange(pending_connection_, std::nullopt)->get()};
            case std::future_status::timeout:
                break;
            case std::future_status::deferred:
                std::abort();
        }
    } else {
        pending_connection_.emplace(std::async(std::launch::async, [this, &state] { return connect_(state); }));
    }
    return std::nullopt;
}

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_disconnected_::send_heartbeat(state_&) {
    return std::nullopt;
}

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_disconnected_::handle_move_request(state_& state) const {
    if (state.move_request_) {
        if (state.move_request_->handle && !state.move_request_->handle->is_done()) {
            state.move_request_->handle->cancel();
        }
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
    // Start with all wire-observable blocking bits set; k_major_alarm is diagnosed lazily.
    const blocking_mask all_bits = blocking_reason::k_in_error | blocking_reason::k_servo_off | blocking_reason::k_motion_blocked |
                                   blocking_reason::k_estop | blocking_reason::k_not_remote;
    return state_independent_{std::move(event.controller), all_bits};
}

std::optional<YaskawaArm::state_::state_variant_> YaskawaArm::state_::state_disconnected_::handle_event(event_connection_lost_ /*event*/) {
    return std::nullopt;
}

// ---------------------------------------------------------------
// state_disconnected_ connect_
// ---------------------------------------------------------------

std::shared_ptr<YaskawaController> YaskawaArm::state_::state_disconnected_::connect_(state_& state) {
    constexpr int k_log_at_n_attempts = 100;
    if (++reconnect_attempts_ % k_log_at_n_attempts == 0) {
        if (triggering_event_) {
            VIAM_SDK_LOG(warn) << "disconnected: connection was lost due to " << triggering_event_->describe()
                               << "; attempting automatic recovery";
        }
        VIAM_SDK_LOG(info) << "reconnect attempt " << reconnect_attempts_;
    }

    auto controller = std::make_shared<YaskawaController>(state.io_context_, state.config_);
    controller->connect().get();
    if (!controller->checkGroupIndex()) {
        throw std::runtime_error("group index check failed after connecting");
    }
    return controller;
}
// NOLINTEND(readability-convert-member-functions-to-static)
