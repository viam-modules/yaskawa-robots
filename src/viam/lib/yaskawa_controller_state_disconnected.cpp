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
    if (triggering_event_) {
        return std::string(name()) + "(" + std::string(triggering_event_->describe()) + ")";
    }
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
            try {
                pending_connection_.get();
            } catch (const std::exception& ex) {
                // Reconnect failed; log deduped to avoid spamming when the controller is unreachable.
                // The next worker cycle will spawn a fresh attempt (future is now invalid).
                constexpr int k_log_at_n_attempts = 100;
                ++reconnect_attempts_;
                if (reconnect_attempts_ == 1 || reconnect_attempts_ % k_log_at_n_attempts == 0) {
                    VIAM_SDK_LOG(warn) << "[fsm] reconnect attempt " << reconnect_attempts_ << " failed: " << ex.what();
                }
                return std::nullopt;
            }
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
                VIAM_SDK_LOG(warn) << "[fsm] exception while cancelling move request on disconnect";
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
    VIAM_SDK_LOG(info) << "[fsm] connection established, entering independent state";
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
    auto* controller = state.controller_;
    // Tear down any stale connections before re-establishing. Idempotent on the initial
    // attempt (sockets have not connected yet); on reconnect this drops the dead session.
    // TcpRobotSocket can't be reused after disconnect, so replace with a fresh instance;
    // establish_connections_() replaces udp_socket_ internally.
    if (controller->udp_socket_) {
        controller->udp_socket_->disconnect();
    }
    if (controller->tcp_socket_) {
        controller->tcp_socket_->disconnect();
    }
    controller->tcp_socket_ = std::make_unique<TcpRobotSocket>(controller->io_context_, controller->host_, controller->tcp_port_);

    controller->establish_connections_();
    if (!controller->checkGroupIndex()) {
        throw std::runtime_error("group index check failed after connecting");
    }
}
// NOLINTEND(readability-convert-member-functions-to-static)
