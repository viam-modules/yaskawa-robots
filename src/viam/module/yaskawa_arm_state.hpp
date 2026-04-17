#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <variant>

#include <Eigen/Core>
#include <viam/sdk/log/logging.hpp>

#include "yaskawa_arm.hpp"

class YaskawaArm::state_ {
    struct private_ {};

   public:
    explicit state_(private_, std::string resource_name, const viam::sdk::ResourceConfig& config, boost::asio::io_context& io_context);
    ~state_();

    static std::unique_ptr<state_> create(const std::string& resource_name,
                                          const viam::sdk::ResourceConfig& config,
                                          boost::asio::io_context& io_context);
    void shutdown();

    // Accessors delegated from YaskawaArm public API
    std::vector<double> read_joint_positions() const;
    RobotStatusMessage read_robot_status() const;
    bool is_moving() const;
    std::string describe() const;
    void request_stop();

    // Move enqueueing
    size_t get_move_epoch() const;
    std::future<void> enqueue_move_request(size_t current_move_epoch,
                                           std::list<Eigen::VectorXd> waypoints,
                                           std::string unix_time,
                                           Eigen::VectorXd velocity,
                                           Eigen::VectorXd acceleration);

    // Config accessors
    const std::optional<double>& get_reject_move_request_threshold_rad() const;
    const Eigen::VectorXd& get_velocity_limits() const;
    const Eigen::VectorXd& get_acceleration_limits() const;
    double get_waypoint_deduplication_tolerance_rad() const;

   private:
    // ---------------------------------------------------------------
    // Blocking reason bitmask
    // ---------------------------------------------------------------
    enum class blocking_reason : uint8_t {
        k_in_error = 0x01,        // auto-recoverable: reset_errors()
        k_servo_off = 0x02,       // auto-recoverable: turn_servo_power_on()
        k_motion_blocked = 0x04,  // auto-recoverable: setMotionMode(RUN)
        k_major_alarm = 0x08,     // human-required: physical panel
        k_estop = 0x10,           // human-required: release button
        k_not_remote = 0x20,      // human-required: pendant
    };
    using blocking_mask = uint8_t;

    static constexpr blocking_mask k_auto_recoverable_mask = static_cast<blocking_mask>(blocking_reason::k_in_error) |
                                                             static_cast<blocking_mask>(blocking_reason::k_servo_off) |
                                                             static_cast<blocking_mask>(blocking_reason::k_motion_blocked);

    // Bitwise helpers for blocking_reason
    friend constexpr blocking_mask operator|(blocking_reason a, blocking_reason b) {
        return static_cast<blocking_mask>(a) | static_cast<blocking_mask>(b);
    }
    friend constexpr blocking_mask operator|(blocking_mask a, blocking_reason b) {
        return a | static_cast<blocking_mask>(b);
    }
    friend constexpr bool has_reason(blocking_mask mask, blocking_reason r) {
        return (mask & static_cast<blocking_mask>(r)) != 0;
    }

    static std::string describe_blocking_mask_(blocking_mask mask);

    // ---------------------------------------------------------------
    // State and event forward declarations
    // ---------------------------------------------------------------
    struct state_disconnected_;
    friend struct state_disconnected_;

    struct state_independent_;
    friend struct state_independent_;

    struct state_ready_;
    friend struct state_ready_;

    using state_variant_ = std::variant<state_disconnected_, state_independent_, state_ready_>;

    struct event_connection_established_;
    class event_connection_lost_;
    struct event_blocking_detected_;
    struct event_ready_detected_;

    using event_variant_ =
        std::variant<event_connection_established_, event_connection_lost_, event_blocking_detected_, event_ready_detected_>;

    // ---------------------------------------------------------------
    // Catch-all event handler base (logs unexpected events)
    // ---------------------------------------------------------------
    template <typename T>
    class state_event_handler_base_ {
       public:
        template <typename Event>
        std::optional<state_variant_> handle_event(Event&& event);

       private:
        friend T;
        state_event_handler_base_() = default;
    };

    // ---------------------------------------------------------------
    // Shared base for connected states
    // ---------------------------------------------------------------
    struct state_connected_ {
        explicit state_connected_(std::shared_ptr<YaskawaController> controller);

        std::chrono::milliseconds get_timeout() const;
        std::optional<event_variant_> recv_arm_data(state_&);
        std::optional<event_variant_> send_heartbeat(state_&);

        std::shared_ptr<YaskawaController> controller_;
    };

    // ---------------------------------------------------------------
    // State: DISCONNECTED
    // ---------------------------------------------------------------
    struct state_disconnected_ : public state_event_handler_base_<state_disconnected_> {
        state_disconnected_() = default;
        explicit state_disconnected_(event_connection_lost_ triggering_event);

        static std::string_view name();
        std::string describe() const;
        std::chrono::milliseconds get_timeout() const;

        std::optional<event_variant_> recv_arm_data(state_&);
        std::optional<event_variant_> upgrade_downgrade(state_&);
        std::optional<event_variant_> handle_move_request(state_&) const;
        std::optional<event_variant_> send_heartbeat(state_&);

        std::optional<state_variant_> handle_event(event_connection_established_);
        std::optional<state_variant_> handle_event(event_connection_lost_);
        using state_event_handler_base_<state_disconnected_>::handle_event;

       private:
        std::shared_ptr<YaskawaController> connect_(state_&);

        int reconnect_attempts_{-1};
        std::optional<std::future<std::shared_ptr<YaskawaController>>> pending_connection_;
        std::unique_ptr<event_connection_lost_> triggering_event_;
    };

    // ---------------------------------------------------------------
    // State: INDEPENDENT
    // ---------------------------------------------------------------
    struct state_independent_ : public state_event_handler_base_<state_independent_>, public state_connected_ {
        explicit state_independent_(std::shared_ptr<YaskawaController> controller, blocking_mask reasons);

        static std::string_view name();
        std::string describe() const;
        using state_connected_::get_timeout;

        using state_connected_::recv_arm_data;
        std::optional<event_variant_> upgrade_downgrade(state_&);
        std::optional<event_variant_> handle_move_request(state_&) const;
        using state_connected_::send_heartbeat;

        std::optional<state_variant_> handle_event(event_connection_lost_);
        std::optional<state_variant_> handle_event(event_blocking_detected_);
        std::optional<state_variant_> handle_event(event_ready_detected_);
        using state_event_handler_base_<state_independent_>::handle_event;

        blocking_mask reasons_;
        int recovery_attempts_{0};
    };

    // ---------------------------------------------------------------
    // State: READY
    // ---------------------------------------------------------------
    struct state_ready_ : public state_event_handler_base_<state_ready_>, public state_connected_ {
        explicit state_ready_(std::shared_ptr<YaskawaController> controller);

        static std::string_view name();
        std::string describe() const;
        using state_connected_::get_timeout;

        using state_connected_::recv_arm_data;
        std::optional<event_variant_> upgrade_downgrade(state_&);
        std::optional<event_variant_> handle_move_request(state_&);
        using state_connected_::send_heartbeat;

        std::optional<state_variant_> handle_event(event_connection_lost_);
        std::optional<state_variant_> handle_event(event_blocking_detected_);
        using state_event_handler_base_<state_ready_>::handle_event;
    };

    // ---------------------------------------------------------------
    // Events
    // ---------------------------------------------------------------
    struct event_connection_established_ {
        static std::string_view name();
        std::string_view describe() const;
        std::shared_ptr<YaskawaController> controller;
    };

    class event_connection_lost_ {
       public:
        static event_connection_lost_ tcp_failure();
        static event_connection_lost_ heartbeat_failure();
        static event_connection_lost_ module_shutdown();

        static std::string_view name();
        std::string_view describe() const;

       private:
        enum class reason : uint8_t {
            k_tcp_failure,
            k_heartbeat_failure,
            k_module_shutdown,
        };
        explicit event_connection_lost_(reason r);
        reason reason_code_;
    };

    struct event_blocking_detected_ {
        static std::string_view name();
        std::string_view describe() const;
        blocking_mask mask;
    };

    struct event_ready_detected_ {
        static std::string_view name();
        std::string_view describe() const;
    };

    // ---------------------------------------------------------------
    // Move request (orthogonal to FSM state)
    // ---------------------------------------------------------------
    struct move_request {
        std::list<Eigen::VectorXd> waypoints;
        std::string unix_time;
        Eigen::VectorXd velocity;
        Eigen::VectorXd acceleration;
        std::unique_ptr<GoalRequestHandle> handle;
        std::promise<void> completion;

        void complete_success();
        void complete_error(std::string_view message);
    };

    // ---------------------------------------------------------------
    // Core FSM machinery (called from run_() under mutex_)
    // ---------------------------------------------------------------
    void emit_event_(event_variant_&& event);

    template <typename Event>
    void emit_event_(Event&& event);

    std::chrono::milliseconds get_timeout_() const;
    static std::string describe_state_(const state_variant_& sv);

    void upgrade_downgrade_();
    void recv_arm_data_();
    void handle_move_request_();
    void send_heartbeat_();

    void run_();

    // ---------------------------------------------------------------
    // Fields
    // ---------------------------------------------------------------
    const std::string resource_name_;
    const std::optional<double> reject_move_request_threshold_rad_;
    boost::asio::io_context& io_context_;
    const viam::sdk::ResourceConfig config_;

    mutable std::mutex mutex_;
    state_variant_ current_state_{state_disconnected_{}};
    std::thread worker_thread_;
    std::condition_variable worker_wakeup_cv_;
    bool shutdown_requested_{false};

    std::atomic<size_t> move_epoch_{0};
    std::optional<move_request> move_request_;
};

// ---------------------------------------------------------------
// state_event_handler_base_ catch-all: log and ignore unknown events
// ---------------------------------------------------------------
template <typename T>
template <typename Event>
std::optional<YaskawaArm::state_::state_variant_> YaskawaArm::state_::state_event_handler_base_<T>::handle_event(Event&& /*event*/) {
    VIAM_SDK_LOG(warn) << "state `" << T::name() << "` received unexpected event `" << Event::name() << "`";
    return std::nullopt;
}
