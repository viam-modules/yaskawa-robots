#include "robot_socket.hpp"

#include <format>
#include <stdexcept>

#include <viam/sdk/log/logging.hpp>

using namespace robot;
using namespace viam::sdk;

// ---------------------------------------------------------------
// state_::constructor / destructor / create / shutdown
// ---------------------------------------------------------------

YaskawaController::state_::state_(private_, YaskawaController* controller) : controller_(controller) {}

YaskawaController::state_::~state_() {
    shutdown();
}

std::unique_ptr<YaskawaController::state_> YaskawaController::state_::create(YaskawaController* controller) {
    auto state = std::make_unique<state_>(private_{}, controller);
    state->worker_thread_ = std::thread{&state_::run_, state.get()};
    return state;
}

void YaskawaController::state_::shutdown() {
    {
        const std::lock_guard lock{mutex_};
        shutdown_requested_ = true;
    }
    worker_wakeup_cv_.notify_all();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

// ---------------------------------------------------------------
// Worker thread
// ---------------------------------------------------------------

void YaskawaController::state_::run_() {
    while (true) {
        std::unique_lock lock{mutex_};
        worker_wakeup_cv_.wait_for(lock, get_timeout_(), [this] { return shutdown_requested_; });
        if (shutdown_requested_) {
            break;
        }
        try {
            recv_arm_data_();
            upgrade_downgrade_();
            handle_move_request_();
            send_heartbeat_();
        } catch (const std::exception& ex) {
            VIAM_SDK_LOG(warn) << controller_->host() << ": exception in worker thread: " << ex.what();
        } catch (...) {
            VIAM_SDK_LOG(warn) << controller_->host() << ": unknown exception in worker thread";
        }
    }
}

// ---------------------------------------------------------------
// FSM core helpers
// ---------------------------------------------------------------

void YaskawaController::state_::emit_event_(event_variant_&& event) {
    auto new_state = std::visit(
        [&](auto& current_state) -> std::optional<state_variant_> {
            const auto pre = describe_state_(current_state_);
            auto next = std::visit([&](auto&& ev) { return current_state.handle_event(*this, std::forward<decltype(ev)>(ev)); },
                                   std::move(event));
            if (next) {
                VIAM_SDK_LOG(info) << controller_->host() << ": state transition `" << pre << "` -> `"
                                   << describe_state_(*next) << "`";
            }
            return next;
        },
        current_state_);

    if (new_state) {
        current_state_ = std::move(*new_state);
    }
}

template <typename Event>
void YaskawaController::state_::emit_event_(Event&& event) {
    emit_event_(event_variant_{std::forward<Event>(event)});
}

std::chrono::milliseconds YaskawaController::state_::get_timeout_() const {
    return std::visit([](const auto& s) { return s.get_timeout(); }, current_state_);
}

std::string YaskawaController::state_::describe_state_(const state_variant_& sv) {
    return std::visit([](const auto& s) { return s.describe(); }, sv);
}

std::string YaskawaController::state_::describe_blocking_mask_(blocking_mask mask) {
    if (mask == 0) {
        return "none";
    }
    std::string result;
    auto append = [&](blocking_reason r, std::string_view label) {
        if (has_reason(mask, r)) {
            if (!result.empty()) {
                result += ", ";
            }
            result += label;
        }
    };
    append(blocking_reason::k_in_error, "in_error");
    append(blocking_reason::k_servo_off, "servo_off");
    append(blocking_reason::k_motion_blocked, "motion_blocked");
    append(blocking_reason::k_major_alarm, "major_alarm");
    append(blocking_reason::k_estop, "estop");
    append(blocking_reason::k_not_remote, "not_remote");
    return result;
}

// ---------------------------------------------------------------
// Cycle dispatch (called from run_() under mutex_)
// ---------------------------------------------------------------

void YaskawaController::state_::recv_arm_data_() {
    auto ev = std::visit([this](auto& s) { return s.recv_arm_data(*this); }, current_state_);
    if (ev) {
        emit_event_(std::move(*ev));
    }
}

void YaskawaController::state_::upgrade_downgrade_() {
    auto ev = std::visit([this](auto& s) { return s.upgrade_downgrade(*this); }, current_state_);
    if (ev) {
        emit_event_(std::move(*ev));
    }
}

void YaskawaController::state_::handle_move_request_() {
    auto ev = std::visit([this](auto& s) { return s.handle_move_request(*this); }, current_state_);
    if (ev) {
        emit_event_(std::move(*ev));
    }
}

void YaskawaController::state_::send_heartbeat_() {
    auto ev = std::visit([this](auto& s) { return s.send_heartbeat(*this); }, current_state_);
    if (ev) {
        emit_event_(std::move(*ev));
    }
}

// ---------------------------------------------------------------
// Public accessors
// ---------------------------------------------------------------

std::string YaskawaController::state_::describe() const {
    const std::lock_guard lock{mutex_};
    return describe_state_(current_state_);
}

bool YaskawaController::state_::is_any_moving() const {
    const std::lock_guard lock{mutex_};
    return std::any_of(move_requests_.begin(), move_requests_.end(),
                       [](const move_request& req) { return req.handle && !req.handle->is_done(); });
}

std::future<void> YaskawaController::state_::enqueue_move_request(uint32_t group_index,
                                                                   std::list<Eigen::VectorXd> waypoints,
                                                                   std::string unix_time,
                                                                   Eigen::VectorXd velocity,
                                                                   Eigen::VectorXd acceleration) {
    std::future<void> future;
    {
        const std::lock_guard lock{mutex_};
        if (!std::holds_alternative<state_ready_>(current_state_)) {
            throw std::runtime_error(std::format("cannot move: arm is in state `{}`", describe_state_(current_state_)));
        }
        auto& req = move_requests_.emplace_back();
        req.group_index = group_index;
        req.waypoints = std::move(waypoints);
        req.unix_time = std::move(unix_time);
        req.velocity = std::move(velocity);
        req.acceleration = std::move(acceleration);
        future = req.completion.get_future();
    }
    worker_wakeup_cv_.notify_one();
    return future;
}

// ---------------------------------------------------------------
// YaskawaController FSM delegation
// ---------------------------------------------------------------

std::string YaskawaController::describe_state() const {
    if (!fsm_) {
        return "uninitialized";
    }
    return fsm_->describe();
}

bool YaskawaController::is_any_moving() const {
    if (!fsm_) {
        return false;
    }
    return fsm_->is_any_moving();
}

std::future<void> YaskawaController::enqueue_move_request(uint32_t group_index,
                                                          std::list<Eigen::VectorXd> waypoints,
                                                          std::string unix_time,
                                                          Eigen::VectorXd velocity,
                                                          Eigen::VectorXd acceleration) {
    if (!fsm_) {
        throw std::runtime_error("controller FSM not initialized");
    }
    return fsm_->enqueue_move_request(group_index, std::move(waypoints), std::move(unix_time), std::move(velocity),
                                      std::move(acceleration));
}

// ---------------------------------------------------------------
// state_connected_
// ---------------------------------------------------------------

// NOLINTBEGIN(readability-convert-member-functions-to-static)
std::chrono::milliseconds YaskawaController::state_::state_connected_::get_timeout() const {
    return std::chrono::milliseconds{100};
}

std::optional<YaskawaController::state_::event_variant_> YaskawaController::state_::state_connected_::recv_arm_data(state_&) {
    return std::nullopt;
}

std::optional<YaskawaController::state_::event_variant_> YaskawaController::state_::state_connected_::send_heartbeat(state_&) {
    return std::nullopt;
}
// NOLINTEND(readability-convert-member-functions-to-static)

// ---------------------------------------------------------------
// move_request
// ---------------------------------------------------------------

void YaskawaController::state_::move_request::complete_success() {
    completion.set_value();
}

void YaskawaController::state_::move_request::complete_error(std::string_view message) {
    completion.set_exception(std::make_exception_ptr(std::runtime_error(std::string(message))));
}
