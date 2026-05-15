#include "robot_socket.hpp"

#include <format>
#include <stdexcept>

using namespace robot;

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
    // TODO(RSDK-13808) try jthread and stop_token
    worker_wakeup_cv_.notify_all();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

// ---------------------------------------------------------------
// Worker thread
// ---------------------------------------------------------------

void YaskawaController::state_::run_() {
    std::unique_lock lock{mutex_};
    while (!shutdown_requested_) {
        try {
            upgrade_downgrade_();
            handle_move_request_();
            send_heartbeat_();
        } catch (const std::exception& ex) {
            LOGGING(warning) << "[fsm] " << controller_->host() << ": exception in worker thread: " << ex.what();
        } catch (...) {
            LOGGING(warning) << "[fsm] " << controller_->host() << ": unknown exception in worker thread";
        }
        // No predicate: any notify (e.g., from enqueue_move_request) returns control to the
        // loop, the cycle re-runs, and shutdown is observed at the top of the next iteration.
        worker_wakeup_cv_.wait_for(lock, get_timeout_());
    }
}

// ---------------------------------------------------------------
// FSM core helpers
// ---------------------------------------------------------------

void YaskawaController::state_::emit_event_(event_variant_&& event) {
    auto new_state = std::visit(
        [&](auto& current_state) -> std::optional<state_variant_> {
            const auto pre = describe_state_(current_state_);
            auto next =
                std::visit([&](auto&& ev) { return current_state.handle_event(*this, std::forward<decltype(ev)>(ev)); }, std::move(event));
            if (next) {
                LOGGING(info) << "[fsm] " << controller_->host() << ": state transition `" << pre << "` -> `" << describe_state_(*next)
                              << "`";
            }
            return next;
        },
        current_state_);

    if (new_state) {
        current_state_ = std::move(*new_state);
        is_disconnected_atomic_.store(std::holds_alternative<state_disconnected_>(current_state_), std::memory_order_release);
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

std::string YaskawaController::state_::describe_not_ready_mask_(not_ready_mask mask) {
    if (mask == 0) {
        return "none";
    }
    std::string result;
    auto append = [&](not_ready_mask r, std::string_view label) {
        if (mask & r) {
            if (!result.empty()) {
                result += ", ";
            }
            result += label;
        }
    };
    append(k_in_error, "in_error");
    append(k_servo_off, "servo_off");
    append(k_motion_blocked, "motion_blocked");
    append(k_major_alarm, "major_alarm");
    append(k_estop, "estop");
    append(k_not_remote, "not_remote");
    return result;
}

// ---------------------------------------------------------------
// Cycle dispatch (called from run_() under mutex_)
// ---------------------------------------------------------------

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
    return std::any_of(
        move_requests_.begin(), move_requests_.end(), [](const move_request& req) { return req.handle && !req.handle->is_done(); });
}

bool YaskawaController::state_::is_disconnected() const {
    // Lock-free: the atomic mirror is updated under mutex_ in emit_event_ on every state
    // transition, but reads don't need the lock. This is the load-bearing reason callers
    // inside the FSM cycle (e.g. wake-up steps) can ask "are we disconnected?" without
    // deadlocking against the worker thread that holds mutex_ for the whole cycle.
    return is_disconnected_atomic_.load(std::memory_order_acquire);
}

std::future<void> YaskawaController::state_::enqueue_move_request(uint32_t group_index,
                                                                  uint32_t axes_controlled,
                                                                  std::vector<trajectory_point_t> samples,
                                                                  std::vector<tolerance_t> tolerance,
                                                                  double trajectory_sampling_freq,
                                                                  std::optional<RealtimeTrajectoryLogger> logger) {
    std::future<void> future;
    {
        const std::lock_guard lock{mutex_};
        // Accept from ready (immediate dispatch) and from independent if all set reasons are
        // auto-recoverable (state_independent_::handle_move_request will wake the arm). Reject
        // from disconnected, or from independent with human-required bits set.
        if (std::holds_alternative<state_disconnected_>(current_state_)) {
            throw std::runtime_error(std::format("cannot move: arm is in state `{}`", describe_state_(current_state_)));
        }
        if (const auto* indep = std::get_if<state_independent_>(&current_state_); indep && (indep->reasons_ & ~k_auto_recoverable_mask)) {
            throw std::runtime_error(std::format("cannot move: arm is in state `{}`", describe_state_(current_state_)));
        }
        auto& req = move_requests_.emplace_back(move_request{
            .group_index = group_index,
            .axes_controlled = axes_controlled,
            .samples = std::move(samples),
            .tolerance = std::move(tolerance),
            .trajectory_sampling_freq = trajectory_sampling_freq,
            .logger = std::move(logger),
            .handle = {},
            .completion = {},
        });
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

bool YaskawaController::is_disconnected() const {
    // No FSM means we've never finished constructing, which is functionally equivalent to
    // "disconnected" from the caller's perspective — we can't reach the controller.
    if (!fsm_) {
        return true;
    }
    return fsm_->is_disconnected();
}

std::future<void> YaskawaController::enqueue_move_request(uint32_t group_index,
                                                          uint32_t axes_controlled,
                                                          std::vector<trajectory_point_t> samples,
                                                          std::vector<tolerance_t> tolerance,
                                                          double trajectory_sampling_freq,
                                                          std::optional<RealtimeTrajectoryLogger> logger) {
    if (!fsm_) {
        throw std::runtime_error("controller FSM not initialized");
    }
    return fsm_->enqueue_move_request(
        group_index, axes_controlled, std::move(samples), std::move(tolerance), trajectory_sampling_freq, std::move(logger));
}

// ---------------------------------------------------------------
// move_request
// ---------------------------------------------------------------

void YaskawaController::state_::move_request::complete_success() {
    completion.set_value();
}

void YaskawaController::state_::move_request::complete_error(std::string_view message) {
    completion.set_exception(std::make_exception_ptr(std::runtime_error(std::string(message))));
}
