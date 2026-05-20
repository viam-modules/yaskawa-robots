#include "robot_socket.hpp"

#include <utility>

using namespace robot;

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

std::optional<YaskawaController::state_::event_variant_> YaskawaController::state_::state_disconnected_::upgrade_downgrade(state_& state) {
    if (!pending_connection_.valid()) {
        // Run the connect on a jthread so state_::shutdown() can request cancellation and the
        // future is fulfilled by a packaged_task (whose destructor doesn't block, unlike one
        // returned from std::async). establish_connections_ polls the stop_token between
        // blocking ops, so cancellation is bounded by one in-flight op (~k_socket_timeout).
        std::packaged_task<void(std::stop_token)> task([this, &state](std::stop_token st) { connect_(state, std::move(st)); });
        pending_connection_ = task.get_future();
        pending_thread_ = std::jthread(std::move(task));
        return std::nullopt;
    }

    switch (pending_connection_.wait_for(std::chrono::seconds(0))) {
        case std::future_status::ready:
            try {
                pending_connection_.get();
            } catch (const std::exception& ex) {
                // Reconnect failed; log on attempt 1, whenever the failure message changes
                // (so new failure modes surface immediately), and every Nth attempt as a
                // heartbeat for long outages. The next worker cycle will spawn a fresh
                // attempt (future is now invalid).
                constexpr int k_heartbeat_attempts = 10;
                ++reconnect_attempts_;
                const std::string current_error = ex.what();
                const bool first_attempt = (reconnect_attempts_ == 1);
                const bool message_changed = (current_error != last_logged_error_);
                const bool heartbeat = (reconnect_attempts_ % k_heartbeat_attempts == 0);
                if (first_attempt || message_changed || heartbeat) {
                    LOGGING(warning) << "[fsm] reconnect attempt " << reconnect_attempts_ << " failed: " << current_error;
                    last_logged_error_ = current_error;
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
            } catch (const std::exception& ex) {
                LOGGING(warning) << "[fsm] exception while cancelling move request on disconnect: " << ex.what();
            } catch (...) {
                LOGGING(warning) << "[fsm] unknown exception while cancelling move request on disconnect";
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
    LOGGING(info) << "[fsm] connection established, entering independent state";
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

void YaskawaController::state_::state_disconnected_::connect_(state_& state, std::stop_token token) {
    // establish_connections_() handles both initial connect and reconnect — on reconnect it
    // tears down the stale session and replaces tcp_socket_ before establishing.
    state.controller_->establish_connections_(std::move(token));
}
// NOLINTEND(readability-convert-member-functions-to-static)
