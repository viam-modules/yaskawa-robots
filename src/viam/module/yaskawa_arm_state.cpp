#include "yaskawa_arm_state.hpp"

#include <format>
#include <stdexcept>

#include <viam/lib/robot_socket.hpp>
#include <viam/sdk/log/logging.hpp>

#include "utils.hpp"

using namespace viam::sdk;

// ---------------------------------------------------------------
// state_::constructor / destructor / create / shutdown
// ---------------------------------------------------------------

YaskawaArm::state_::state_(private_, std::string resource_name, const ResourceConfig& config, boost::asio::io_context& io_context)
    : resource_name_(std::move(resource_name)),
      reject_move_request_threshold_rad_(find_config_attribute<double>(config, "reject_move_request_threshold_rad")),
      io_context_(io_context),
      config_(config) {}

YaskawaArm::state_::~state_() {
    shutdown();
}

std::unique_ptr<YaskawaArm::state_> YaskawaArm::state_::create(const std::string& resource_name,
                                                               const ResourceConfig& config,
                                                               boost::asio::io_context& io_context) {
    auto state = std::make_unique<state_>(private_{}, resource_name, config, io_context);
    state->worker_thread_ = std::thread{&state_::run_, state.get()};
    return state;
}

void YaskawaArm::state_::shutdown() {
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

void YaskawaArm::state_::run_() {
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
            VIAM_SDK_LOG(warn) << resource_name_ << ": exception in worker thread: " << ex.what();
        } catch (...) {
            VIAM_SDK_LOG(warn) << resource_name_ << ": unknown exception in worker thread";
        }
    }
}

// ---------------------------------------------------------------
// FSM core helpers
// ---------------------------------------------------------------

void YaskawaArm::state_::emit_event_(event_variant_&& event) {
    auto new_state = std::visit(
        [&](auto& current_state) -> std::optional<state_variant_> {
            const auto pre = describe_state_(current_state_);
            auto next = std::visit([&](auto&& ev) { return current_state.handle_event(std::forward<decltype(ev)>(ev)); }, std::move(event));
            if (next) {
                VIAM_SDK_LOG(info) << resource_name_ << ": state transition `" << pre << "` -> `" << describe_state_(*next) << "`";
            }
            return next;
        },
        current_state_);

    if (new_state) {
        current_state_ = std::move(*new_state);
    }
}

template <typename Event>
void YaskawaArm::state_::emit_event_(Event&& event) {
    emit_event_(event_variant_{std::forward<Event>(event)});
}

std::chrono::milliseconds YaskawaArm::state_::get_timeout_() const {
    return std::visit([](const auto& s) { return s.get_timeout(); }, current_state_);
}

std::string YaskawaArm::state_::describe_state_(const state_variant_& sv) {
    return std::visit([](const auto& s) { return s.describe(); }, sv);
}

std::string YaskawaArm::state_::describe_blocking_mask_(blocking_mask mask) {
    if (mask == 0) {
        return "none";
    }
    std::string result;
    auto append = [&](blocking_reason r, std::string_view label) {
        if (has_reason(mask, r)) {
            if (!result.empty())
                result += ", ";
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

void YaskawaArm::state_::recv_arm_data_() {
    auto ev = std::visit([this](auto& s) { return s.recv_arm_data(*this); }, current_state_);
    if (ev)
        emit_event_(std::move(*ev));
}

void YaskawaArm::state_::upgrade_downgrade_() {
    auto ev = std::visit([this](auto& s) { return s.upgrade_downgrade(*this); }, current_state_);
    if (ev)
        emit_event_(std::move(*ev));
}

void YaskawaArm::state_::handle_move_request_() {
    auto ev = std::visit([this](auto& s) { return s.handle_move_request(*this); }, current_state_);
    if (ev)
        emit_event_(std::move(*ev));
}

void YaskawaArm::state_::send_heartbeat_() {
    auto ev = std::visit([this](auto& s) { return s.send_heartbeat(*this); }, current_state_);
    if (ev)
        emit_event_(std::move(*ev));
}

// ---------------------------------------------------------------
// Public accessors
// ---------------------------------------------------------------

std::string YaskawaArm::state_::describe() const {
    const std::lock_guard lock{mutex_};
    return describe_state_(current_state_);
}

bool YaskawaArm::state_::is_moving() const {
    const std::lock_guard lock{mutex_};
    return move_request_.has_value() && move_request_->handle && !move_request_->handle->is_done();
}

size_t YaskawaArm::state_::get_move_epoch() const {
    return move_epoch_.load(std::memory_order_acquire);
}

std::future<void> YaskawaArm::state_::enqueue_move_request(size_t current_move_epoch,
                                                           std::list<Eigen::VectorXd> waypoints,
                                                           std::string unix_time,
                                                           Eigen::VectorXd velocity,
                                                           Eigen::VectorXd acceleration) {
    if (!move_epoch_.compare_exchange_strong(current_move_epoch, current_move_epoch + 1, std::memory_order_acq_rel)) {
        throw std::runtime_error("move operation was superseded by a newer operation");
    }
    const std::lock_guard lock{mutex_};
    if (!std::holds_alternative<state_ready_>(current_state_)) {
        throw std::runtime_error(std::format("cannot move: arm is in state `{}`", describe_state_(current_state_)));
    }
    if (move_request_) {
        throw std::runtime_error("an actuation is already in progress");
    }
    auto& req = move_request_.emplace();
    req.waypoints = std::move(waypoints);
    req.unix_time = std::move(unix_time);
    req.velocity = std::move(velocity);
    req.acceleration = std::move(acceleration);
    return req.completion.get_future();
}

std::vector<double> YaskawaArm::state_::read_joint_positions() const {
    throw std::runtime_error("not implemented");
}

RobotStatusMessage YaskawaArm::state_::read_robot_status() const {
    throw std::runtime_error("not implemented");
}

void YaskawaArm::state_::request_stop() {
    throw std::runtime_error("not implemented");
}

const std::optional<double>& YaskawaArm::state_::get_reject_move_request_threshold_rad() const {
    return reject_move_request_threshold_rad_;
}

const Eigen::VectorXd& YaskawaArm::state_::get_velocity_limits() const {
    throw std::runtime_error("not implemented");
}

const Eigen::VectorXd& YaskawaArm::state_::get_acceleration_limits() const {
    throw std::runtime_error("not implemented");
}

double YaskawaArm::state_::get_waypoint_deduplication_tolerance_rad() const {
    throw std::runtime_error("not implemented");
}

// ---------------------------------------------------------------
// state_connected_
// ---------------------------------------------------------------

YaskawaArm::state_::state_connected_::state_connected_(std::shared_ptr<YaskawaController> controller)
    : controller_(std::move(controller)) {}

std::chrono::milliseconds YaskawaArm::state_::state_connected_::get_timeout() const {
    return std::chrono::milliseconds{100};
}

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_connected_::recv_arm_data(state_&) {
    return std::nullopt;
}

std::optional<YaskawaArm::state_::event_variant_> YaskawaArm::state_::state_connected_::send_heartbeat(state_&) {
    return std::nullopt;
}

// ---------------------------------------------------------------
// move_request
// ---------------------------------------------------------------

void YaskawaArm::state_::move_request::complete_success() {
    completion.set_value();
}

void YaskawaArm::state_::move_request::complete_error(std::string_view message) {
    completion.set_exception(std::make_exception_ptr(std::runtime_error(std::string(message))));
}
