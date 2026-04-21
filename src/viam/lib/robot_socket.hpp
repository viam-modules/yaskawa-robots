#pragma once

#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <boost/asio/any_completion_handler.hpp>
#include <boost/asio/any_io_executor.hpp>
#include <boost/asio/async_result.hpp>
#include <boost/asio/awaitable.hpp>
#include <boost/asio/post.hpp>
#include <boost/system/detail/error_code.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <third_party/trajectories/Path.h>
#include <viam/sdk/config/resource.hpp>

#include "protocol.h"

#include "logger.hpp"
#include "trajectory_logger.hpp"

template <typename T>
[[nodiscard]] constexpr decltype(auto) degrees_to_radians(T&& degrees) {
    return std::forward<T>(degrees) * (M_PI / 180.0);
}

template <typename T>
[[nodiscard]] constexpr decltype(auto) radians_to_degrees(T&& radians) {
    return std::forward<T>(radians) * (180.0 / M_PI);
}

namespace robot {

/// Thread-safe asynchronous queue with Boost.Asio integration
/// Supports async waiting for items to become available
/// @tparam T The type of items stored in the queue
template <typename T>
class AsyncQueue : public std::enable_shared_from_this<AsyncQueue<T>> {
    struct private_ {
        explicit private_() = default;
    };

   private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    boost::asio::any_io_executor executor_;
    std::atomic<bool> closed_;
    struct PendingPopOperation {
        boost::asio::any_completion_handler<void(std::optional<T>, boost::system::error_code)> handler_;
    };
    std::queue<PendingPopOperation> pending_;
    void notify_one() {
        if (!pending_.empty() && !queue_.empty()) {
            auto op = std::move(pending_.front());
            pending_.pop();
            auto item = std::move(queue_.front());
            queue_.pop();
            boost::asio::post(executor_, [handler = std::move(op.handler_), item = std::move(item)]() mutable {
                handler(std::move(item), boost::system::error_code());
            });
        }
    }

   public:
    explicit AsyncQueue(private_, boost::asio::any_io_executor exec) : executor_(std::move(exec)) {};
    static auto create(boost::asio::any_io_executor exec) {
        return std::make_shared<AsyncQueue<T>>(private_{}, std::move(exec));
    };
    // This is somewhat an issue to use standard emplace if there is a pending op,
    // indeed c++17 wants a reference back however we are removing the item
    // immediately so in our case emplace returns void
    template <class... Args>
    void emplace(Args&&... args) {
        if (closed_) {
            throw std::runtime_error("cannot emplace on closed queue");
        }
        {
            std::scoped_lock lock{mutex_};
            queue_.emplace(std::forward<Args>(args)...);
            notify_one();
        }
    }
    void push(T&& value) {
        if (closed_) {
            throw std::runtime_error("cannot push on closed queue");
        }
        {
            const std::scoped_lock lock{mutex_};
            queue_.push(std::move(value));
            notify_one();
        }
    }
    void push(const T& value) {
        if (closed_) {
            throw std::runtime_error("cannot push on closed queue");
        }
        {
            const std::scoped_lock lock{mutex_};
            queue_.push(std::move(value));
            notify_one();
        }
    }

    T pop() {
        if (closed_) {
            throw std::runtime_error("cannot pop on closed queue");
        }
        {
            const std::scoped_lock lock{mutex_};
            T value = std::move(queue_.front());
            queue_.pop();
            return value;
        }
    }

    template <typename CompletionToken>
    auto async_pop(CompletionToken&& token) {
        if (closed_) {
            throw std::runtime_error("cannot pop on closed queue");
        }

        return boost::asio::async_initiate<CompletionToken, void(std::optional<T>, boost::system::error_code ec)>(
            [weak = this->weak_from_this()](auto&& handler) mutable {
                auto self = weak.lock();
                if (!self) {
                    return;
                }
                const std::scoped_lock lock{self->mutex_};
                if (self->closed_) {
                    LOGGING(debug) << "AsyncQueue is closed, cannot pop on closed queue";
                    return;
                }
                if (!self->queue_.empty()) {
                    T item = std::move(self->queue_.front());
                    self->queue_.pop();
                    boost::asio::post(self->executor_,
                                      [handler = std::forward<decltype(handler)>(handler), item = std::move(item)]() mutable {
                                          handler(std::make_optional(std::move(item)), boost::system::error_code());
                                      });
                } else {
                    self->pending_.push(PendingPopOperation{std::forward<decltype(handler)>(handler)});
                }
            },
            token);
    }
    size_t size() const {
        const std::scoped_lock lock{mutex_};
        return queue_.size();
    }
    bool empty() const {
        const std::scoped_lock lock{mutex_};
        return queue_.empty();
    }
    bool is_closed() const {
        return closed_;
    }
    void close() {
        closed_.store(true);
    }
    void clear() {
        const std::scoped_lock lock{mutex_};
        while (!pending_.empty()) {
            auto ops = std::move(pending_.front());
            // probably incorrect, if io_context is already cancelled we might not be
            // able to post todo look for a better implementation
            boost::asio::post(
                executor_, [handler = std::move(ops.handler_)]() mutable { handler(std::nullopt, boost::asio::error::operation_aborted); });
            pending_.pop();
        }
        while (!queue_.empty()) {
            queue_.pop();
        }
    }
};

struct Message {
    protocol_header_t header;
    std::vector<uint8_t> payload;

    Message() = default;
    Message(message_type_t type, std::vector<uint8_t>&& data = {});
    Message(protocol_header_t header, std::vector<uint8_t>&& payload);
    Message(Message&& msg) noexcept;
    Message(const Message&);
    Message& operator=(const Message&);
    std::string get_error(message_type_t expected_type) const;

    friend std::ostream& operator<<(std::ostream& os, const Message& msg);
};

struct CartesianPosition {
    double x, y, z;
    double rx, ry, rz;
    CartesianPosition() = default;
    CartesianPosition(const Message&);
    CartesianPosition(const CartesianPosition&);
    std::string toString() const;
};

struct AnglePosition {
    std::vector<double> pos;
    AnglePosition() = default;
    AnglePosition(const Message&);
    AnglePosition(std::vector<double> posRad);
    std::string toString();
    void toRad();
};

struct StatusMessage {
    uint8_t group_index{0};        // 1 byte - which group this status is for
    uint8_t group_type{0};         // 1 byte - group_type_t
    int64_t timestamp{0};          // 8 bytes
    uint8_t num_axes{0};           // 1 byte
    std::vector<double> position;  // 8 * 8 = 64 bytes
    std::vector<double> velocity;  // 8 * 8 = 64 bytes
    std::vector<double> torque;    // 8 * 8 = 64 bytes
    std::vector<double> position_corrected;

    StatusMessage() = default;
    StatusMessage(const Message&);
};

struct RobotStatusMessage {
    int64_t ts{0};                // 8 bytes - timestamp
    uint8_t group_index{0};       // 1 byte - which group this status is for
    int mode{0};                  // 4 bytes - robot mode
    bool e_stopped{false};        // 1 byte - estop status
    bool drives_powered{false};   // 1 byte - drive power status
    bool motion_possible{false};  // 1 byte - motion enabled
    bool in_motion{false};        // 1 byte - motion status
    bool in_error{false};         // 1 byte - error status
    std::vector<int> error_codes;
    int size{0};  // 4 bytes - number of active error codes

    RobotStatusMessage() = default;
    RobotStatusMessage(const Message&);
};

struct CheckGroupMessage {
    bool is_known_group = false;
    CheckGroupMessage() = default;
    CheckGroupMessage(const Message&);
};

struct GroupCapability {
    uint8_t group_id{0};
    uint8_t group_type{0};
    uint8_t group_sub_index{0};
    uint8_t num_axes{0};
    std::vector<uint8_t> axis_types;
    std::vector<int8_t> base_axis_motion;
    int32_t interpolation_period_us{0};
};

struct CapabilitiesMessage {
    uint8_t protocol_version{0};
    uint8_t num_groups{0};
    std::vector<GroupCapability> groups;

    CapabilitiesMessage() = default;
    CapabilitiesMessage(const Message&);
};

struct State {
    std::atomic<bool> e_stopped;
    std::atomic<bool> in_motion;
    std::atomic<bool> drive_powered;
    std::atomic<bool> in_error;
    explicit State();
    void UpdateState(const RobotStatusMessage& msg);
    bool IsReady() const;
};

struct GoalStatusMessage {
    int32_t goal_id;
    goal_state_t state;
    uint32_t current_queue_size;
    double progress;
    int64_t timestamp_ms;

    GoalStatusMessage() = default;
    GoalStatusMessage(const Message& msg);
};

struct GoalAcceptedMessage {
    int32_t goal_id;
    uint32_t num_trajectory_accepted;
    int64_t timestamp_ms;

    GoalAcceptedMessage() = default;
    GoalAcceptedMessage(const Message& msg);
};

struct MakeGoalResult {
    GoalAcceptedMessage accepted;
    std::vector<trajectory_point_t> remaining_trajectory;
};

class RobotSocketBase {
   public:
    RobotSocketBase(boost::asio::io_context& io_context, const std::string& host, uint16_t port)
        : io_context_(io_context), host_(host), port_(port) {}

    virtual ~RobotSocketBase() = default;

    virtual std::future<void> connect() = 0;
    virtual std::future<Message> send_request(Message request) = 0;
    virtual void disconnect() = 0;

    bool is_connected() const {
        return connected_.load();
    }

   protected:
    // TODO(RSDK-12628) make these variables private
    // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
    boost::asio::io_context& io_context_;
    std::string host_;
    uint16_t port_;
    std::atomic<bool> connected_{false};
    // NOLINTEND(misc-non-private-member-variables-in-classes)

    static protocol_header_t parse_header(const std::vector<uint8_t>& buffer);
};

class TcpRobotSocket : public RobotSocketBase {
   public:
    TcpRobotSocket(boost::asio::io_context& io_context, const std::string& host = "127.0.0.1", uint16_t port = TCP_PORT);
    ~TcpRobotSocket() override;

    std::future<void> connect() override;
    std::future<Message> send_request(Message request) override;
    void disconnect() final;

   private:
    using tcp = boost::asio::ip::tcp;

    struct Session {
        tcp::socket socket_;
        std::shared_ptr<AsyncQueue<std::pair<Message, std::promise<Message>>>> queue_;
        Session(boost::asio::io_context& io_context, boost::asio::any_io_executor exec)
            : socket_(io_context), queue_(AsyncQueue<std::pair<Message, std::promise<Message>>>::create(std::move(exec))) {}
    };

    std::shared_ptr<Session> session_;

    static boost::asio::awaitable<void> process_requests(std::shared_ptr<Session> session);
};

// RAII wrapper around std::promise that prevents std::terminate on unfulfilled destruction.
// If the promise is destroyed without being fulfilled, the destructor sets a descriptive exception.
class SafePromise {
    std::promise<Message> promise_;
    bool fulfilled_ = false;

   public:
    SafePromise() = default;
    SafePromise(SafePromise&& other) noexcept : promise_(std::move(other.promise_)), fulfilled_(other.fulfilled_) {
        other.fulfilled_ = true;  // prevent moved-from destructor from touching the promise
    }
    SafePromise& operator=(SafePromise&& other) noexcept {
        if (this != &other) {
            promise_ = std::move(other.promise_);
            fulfilled_ = other.fulfilled_;
            other.fulfilled_ = true;
        }
        return *this;
    }

    void set_value(Message msg) {
        if (!fulfilled_) {
            promise_.set_value(std::move(msg));
            fulfilled_ = true;
        }
    }

    void set_exception(std::exception_ptr e) {
        if (!fulfilled_) {
            promise_.set_exception(std::move(e));
            fulfilled_ = true;
        }
    }

    std::future<Message> get_future() {
        return promise_.get_future();
    }

    ~SafePromise() {
        if (!fulfilled_) {
            try {
                promise_.set_exception(std::make_exception_ptr(std::runtime_error("status not received before disconnect")));
            } catch (...) {
                LOGGING(debug) << "caught unknown exception";
            }
        }
    }
};

template <class... Ts>
struct cacheVisitor : Ts... {
    using Ts::operator()...;
};

class UdpRobotSocket : public RobotSocketBase {
   public:
    UdpRobotSocket(boost::asio::io_context& io_context, std::shared_ptr<State> state);
    ~UdpRobotSocket() override;

    std::future<void> connect() override;
    std::future<Message> send_request(Message request) override;
    void disconnect() final;

    std::future<Message> get_group_status(uint8_t group_index);
    std::future<Message> get_group_robot_status(uint8_t group_index);
    uint16_t get_local_port() const;

   private:
    using udp = boost::asio::ip::udp;
    using CacheSlot = std::variant<std::monostate, Message, SafePromise>;

    struct Session {
        udp::socket socket_;
        mutable std::mutex status_mutex_;
        std::array<CacheSlot, MAX_GROUPS> group_status_;
        std::array<CacheSlot, MAX_GROUPS> group_robot_status_;
        std::shared_ptr<State> robot_state_;
        Session(boost::asio::io_context& io_context, std::shared_ptr<State> state) : socket_(io_context), robot_state_(std::move(state)) {}
    };

    std::shared_ptr<State> robot_state_;
    std::shared_ptr<Session> session_;

    std::future<Message> request_from_cache_(std::array<CacheSlot, MAX_GROUPS>& cache, uint8_t group_index);
    static void save_to_cache_(std::array<CacheSlot, MAX_GROUPS>& cache, uint8_t group_index, const Message& msg);
    static boost::asio::awaitable<void> receive_messages(std::shared_ptr<Session> session);
    static Message parse_message(const std::vector<uint8_t>& buffer);
};

class UdpBroadcastListener {
   public:
    explicit UdpBroadcastListener(boost::asio::io_context& io_context, uint16_t port = 21789);
    ~UdpBroadcastListener();

    void start();
    void stop();

   private:
    using udp = boost::asio::ip::udp;

    struct Session {
        udp::socket socket_;
        std::array<char, 1024> recv_buffer_{};
        std::unique_ptr<viam::yaskawa::ViamControllerLogParser> log_parser_;
        explicit Session(boost::asio::io_context& io_context)
            : socket_(io_context), log_parser_(std::make_unique<viam::yaskawa::ViamControllerLogParser>()) {}
    };

    boost::asio::io_context& io_context_;
    uint16_t port_;
    bool started_{false};
    std::shared_ptr<Session> session_;

    static boost::asio::awaitable<void> receive_broadcasts(std::shared_ptr<Session> session);
};

class GoalRequestHandle;

class YaskawaController : public std::enable_shared_from_this<YaskawaController> {
   public:
    explicit YaskawaController(boost::asio::io_context& io_context, const viam::sdk::ResourceConfig& config);
    ~YaskawaController();

    std::future<void> connect();
    void disconnect();
    double get_waypoint_deduplication_tolerance_rad() const;

    void send_test_trajectory();
    void turn_servo_power_on();
    void send_heartbeat();
    void send_test_error_command();
    void get_error_info();
    StatusMessage get_group_position_velocity_torque(uint8_t group_index);
    RobotStatusMessage get_robot_status();
    RobotStatusMessage get_group_robot_status(uint8_t group_index);
    void register_udp_port(uint16_t port);
    void reset_errors();
    std::future<Message> echo_trajectory();  // currently unused and unimplemented on the controller
    GoalStatusMessage get_goal_status(int32_t id);
    void cancel_goal(int32_t id);
    bool stop(uint32_t group_index);
    void setMotionMode(uint8_t mode);
    CartesianPosition getCartPosition(uint32_t group_index);
    AnglePosition cartPosToAngle(uint32_t group_index, CartesianPosition& pos);
    CartesianPosition angleToCartPos(uint32_t group_index, AnglePosition& pos);
    bool checkGroupIndex(uint32_t group_index);
    CapabilitiesMessage get_capabilities();

    std::unique_ptr<GoalRequestHandle> move(std::list<Eigen::VectorXd> waypoints,
                                            uint32_t group_index,
                                            const std::string& unix_time,
                                            const Eigen::VectorXd& velocity_limits,
                                            const Eigen::VectorXd& acceleration_limits);

    /// Factory: returns an existing controller for the same host, or creates a new one.
    static std::shared_ptr<YaskawaController> get_or_create(boost::asio::io_context& io_context, const viam::sdk::ResourceConfig& config);

    const Eigen::VectorXd& get_velocity_limits() const {
        return velocity_limits_;
    }
    const Eigen::VectorXd& get_acceleration_limits() const {
        return acceleration_limits_;
    }

    void set_trajectory_loggers(std::string robot_model, std::optional<std::function<std::optional<std::string>()>> telemetry_path_fn);

   private:
    boost::asio::io_context& io_context_;
    std::string host_;
    uint16_t tcp_port_;
    std::shared_ptr<State> robot_state_;

    std::unique_ptr<TcpRobotSocket> tcp_socket_;
    std::unique_ptr<UdpRobotSocket> udp_socket_;
    std::unique_ptr<UdpBroadcastListener> broadcast_listener_;
    Eigen::VectorXd velocity_limits_;
    Eigen::VectorXd acceleration_limits_;
    double trajectory_sampling_freq_;
    double waypoint_dedup_tolerance_rad_;

    // Per-group move locking: prevents concurrent moves on the same group
    // while allowing parallel moves on different groups.
    std::array<std::atomic<bool>, MAX_GROUPS> group_move_in_progress_{};

    // Connection lifecycle: true between connect() and disconnect()
    std::atomic<bool> running_{false};
    std::thread heartbeat_thread_;

    bool use_new_trajectory_planner_{false};
    double path_tolerance_rad_{0.1};
    std::optional<double> collinearization_ratio_;
    double segmentation_threshold_rad_;
    std::string robot_model_;
    std::optional<std::function<std::optional<std::string>()>> telemetry_path_fn_;

    void establish_connections_();
    void reconnect_();

    static bool is_status_command(message_type_t type);
    Message create_status_response_from_cache(message_type_t requested_type) const;
    std::optional<MakeGoalResult> make_goal_(std::list<Eigen::VectorXd> waypoints,
                                             uint32_t group_index,
                                             const std::string& unix_time,
                                             const Eigen::VectorXd& max_velocity_vec,
                                             const Eigen::VectorXd& max_acceleration_vec,
                                             std::optional<RealtimeTrajectoryLogger>& logger);
    GoalAcceptedMessage send_goal_(uint32_t group_index,
                                   uint32_t axes_controlled,
                                   const std::vector<trajectory_point_t>& trajectory,
                                   const std::vector<tolerance_t>& tolerance);
};

class GoalRequestHandle {
   public:
    GoalRequestHandle(int32_t goal_id,
                      const std::shared_ptr<YaskawaController>& controller,
                      std::shared_future<goal_state_t> completion_future);
    goal_state_t wait();
    std::optional<goal_state_t> wait_for(std::chrono::milliseconds);
    void cancel();
    std::future<GoalStatusMessage> get_status();
    bool is_done() const;

   private:
    int32_t goal_id_;
    std::atomic<bool> is_done_;
    std::weak_ptr<YaskawaController> controller_;
    std::shared_future<goal_state_t> completion_future_;
    mutable std::mutex mutex_;
};

}  // namespace robot
