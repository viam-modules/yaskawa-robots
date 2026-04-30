#include "robot_socket.hpp"

#include <utility>

#include <viam/sdk/log/logging.hpp>

using namespace robot;
using namespace viam::sdk;

// NOLINTBEGIN(readability-convert-member-functions-to-static)

// ---------------------------------------------------------------
// state_disconnected_ constructors
// ---------------------------------------------------------------

YaskawaController::state_::state_disconnected_::state_disconnected_(event_connection_lost_ triggering_event)
    : triggering_event_(std::move(triggering_event)) {}

// ---------------------------------------------------------------
// state_disconnected_ identity
// ---------------------------------------------------------------

std::string_view YaskawaController::state_::state_disconnected_::name() {
    return "disconnected";
}

std::string YaskawaController::state_::state_disconnected_::describe() const {
    return std::string(name());
}

std::chrono::milliseconds YaskawaController::state_::state_disconnected_::get_timeout() const {
    if (pending_connection_.valid()) {
        return std::chrono::milliseconds{50};
    }
    return std::chrono::milliseconds{1000};
}

// ---------------------------------------------------------------
// state_disconnected_ cycle
// ---------------------------------------------------------------

std::optional<YaskawaController::state_::event_variant_> YaskawaController::state_::state_disconnected_::recv_robot_data(state_&) {
    return std::nullopt;
}

std::optional<YaskawaController::state_::event_variant_> YaskawaController::state_::state_disconnected_::upgrade_downgrade(state_& state) {
    if (!pending_connection_.valid()) {
        pending_connection_ = std::async(std::launch::async, [this, &state] { connect_(state); });
        return std::nullopt;
    }

    switch (pending_connection_.wait_for(std::chrono::seconds(0))) {
        case std::future_status::ready:
            pending_connection_.get();
            return event_connection_established_{};
        case std::future_status::timeout:
            break;
        case std::future_status::deferred:
            std::abort();
    }
    return std::nullopt;
}

std::optional<YaskawaController::state_::event_variant_> YaskawaController::state_::state_disconnected_::send_heartbeat(state_&) {
    return std::nullopt;
}

std::optional<YaskawaController::state_::event_variant_> YaskawaController::state_::state_disconnected_::handle_move_request(
    state_& state) const {
    auto requests = std::exchange(state.move_requests_, {});
    for (auto& req : requests) {
        if (req.handle && !req.handle->is_done()) {
            try {
                req.handle->cancel();
            } catch (...) {
                VIAM_SDK_LOG(warn) << "exception while cancelling move request on disconnect";
            }
        }
        req.complete_error("arm is disconnected");
    }
    return std::nullopt;
}

// ---------------------------------------------------------------
// state_disconnected_ transitions
// ---------------------------------------------------------------

std::optional<YaskawaController::state_::state_variant_> YaskawaController::state_::state_disconnected_::handle_event(
    state_&, event_connection_established_) {
    VIAM_SDK_LOG(info) << "connection established, entering independent state";
    const not_ready_mask all_bits = k_in_error | k_servo_off | k_motion_blocked | k_estop | k_not_remote;
    return state_independent_{all_bits};
}

std::optional<YaskawaController::state_::state_variant_> YaskawaController::state_::state_disconnected_::handle_event(
    state_&, event_connection_lost_) {
    return std::nullopt;
}

// ---------------------------------------------------------------
// state_disconnected_ connect_
// ---------------------------------------------------------------

void YaskawaController::state_::state_disconnected_::connect_(state_& state) {
    constexpr int k_log_at_n_attempts = 100;
    if (++reconnect_attempts_ % k_log_at_n_attempts == 0) {
        if (triggering_event_) {
            VIAM_SDK_LOG(warn) << "disconnected: connection was lost due to " << triggering_event_->describe()
                               << "; attempting automatic recovery";
        }
        VIAM_SDK_LOG(info) << "reconnect attempt " << reconnect_attempts_;
    }

    state.controller_->connect().get();
    if (!state.controller_->checkGroupIndex()) {
        throw std::runtime_error("group index check failed after connecting");
    }
}
// NOLINTEND(readability-convert-member-functions-to-static)
