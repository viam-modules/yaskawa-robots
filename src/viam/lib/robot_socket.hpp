#pragma once

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
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include <third_party/trajectories/Path.h>

extern "C" {
#include "protocol.h"
}

#include "logger.hpp"

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
    int64_t timestamp;             // 8 bytes
    uint8_t num_axes;              // 1 byte
    std::vector<double> position;  // 8 * 8 = 64 bytes
    std::vector<double> velocity;  // 8 * 8 = 64 bytes
    std::vector<double> torque;    // 8 * 8 = 64 bytes
    std::vector<double> position_corrected;

    StatusMessage() = default;
    StatusMessage(const Message&);
};

struct RobotStatusMessage {
    int64_t ts;            // 8 bytes - timestamp
    int mode;              // 4 bytes - robot mode
    bool e_stopped;        // 1 byte - estop status
    bool drives_powered;   // 1 byte - drive power status
    bool motion_possible;  // 1 byte - motion enabled
    bool in_motion;        // 1 byte - motion status
    bool in_error;         // 1 byte - error status
    std::vector<int> error_codes;
    int size;  // 4 bytes - number of active error codes

    RobotStatusMessage() = default;
    RobotStatusMessage(const Message&);
};

struct CheckGroupMessage {
    bool is_known_group = false;
    CheckGroupMessage() = default;
    CheckGroupMessage(const Message&);
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
    double progress;
    int64_t timestamp_ms;

    GoalStatusMessage() = default;
    GoalStatusMessage(const Message& msg);
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

    tcp::socket socket_;
    std::shared_ptr<AsyncQueue<std::pair<Message, std::promise<Message>>>> request_queue_;
    std::atomic<bool> running_{false};

    boost::asio::awaitable<void> process_requests();
    std::future<void> async_send(Message message);
    std::future<Message> async_receive();
};

class UdpRobotSocket : public RobotSocketBase {
   public:
    UdpRobotSocket(boost::asio::io_context& io_context, State& state);
    ~UdpRobotSocket() override;

    std::future<void> connect() override;
    std::future<Message> send_request(Message request) override;
    void disconnect() final;

    void get_status(std::promise<Message>);
    void get_robot_status(std::promise<Message>);
    uint16_t get_local_port() const;

   private:
    using udp = boost::asio::ip::udp;

    State& robot_state_;

    udp::socket socket_;
    std::atomic<bool> running_{false};

    mutable std::shared_mutex status_mutex_;
    std::variant<std::monostate, Message, std::promise<Message>> cached_status_;
    std::variant<std::monostate, Message, std::promise<Message>> cached_robot_status_;

    boost::asio::awaitable<void> receive_messages();
    void handle_status_message(const Message& message);
    void handle_robot_status_message(const Message& message);
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

    boost::asio::io_context& io_context_;
    udp::socket socket_;
    uint16_t port_;
    std::atomic<bool> running_{false};
    std::array<char, 1024> recv_buffer_;
    std::unique_ptr<viam::yaskawa::ViamControllerLogParser> log_parser_;
    boost::asio::awaitable<void> receive_broadcasts();
};

class GoalRequestHandle;

class YaskawaController : public std::enable_shared_from_this<YaskawaController> {
   public:
    explicit YaskawaController(boost::asio::io_context& io_context,
                               double speed,
                               double acceleration,
                               uint32_t group_index,
                               const std::string& host = "127.0.0.1");
    ~YaskawaController() = default;

    std::future<void> connect();
    void disconnect();
    uint32_t get_group_index() const;

    void send_test_trajectory();
    void turn_servo_power_on();
    void send_heartbeat();
    void send_test_error_command();
    void get_error_info();
    StatusMessage get_robot_position_velocity_torque();
    RobotStatusMessage get_robot_status();
    void register_udp_port(uint16_t port);
    void reset_errors();
    std::future<Message> echo_trajectory();  // currently unused and unimplemented on the controller
    GoalStatusMessage get_goal_status(int32_t id);
    void cancel_goal(int32_t id);
    bool stop();
    void setMotionMode(uint8_t mode);
    CartesianPosition getCartPosition();
    AnglePosition cartPosToAngle(CartesianPosition& pos);
    CartesianPosition angleToCartPos(AnglePosition& pos);
    bool checkGroupIndex();

    std::unique_ptr<GoalRequestHandle> move(std::list<Eigen::VectorXd> waypoints, const std::string& unix_time);

   private:
    boost::asio::io_context& io_context_;
    std::string host_;
    State robot_state_;

    std::unique_ptr<TcpRobotSocket> tcp_socket_;
    std::unique_ptr<UdpRobotSocket> udp_socket_;
    std::unique_ptr<UdpBroadcastListener> broadcast_listener_;
    double speed_;
    double acceleration_;
    uint32_t group_index_;
    std::thread heartbeat_;

    static bool is_status_command(message_type_t type);
    Message create_status_response_from_cache(message_type_t requested_type) const;
    std::future<Message> make_goal_(std::list<Eigen::VectorXd> waypoints, const std::string& unix_time);
    std::future<Message> send_goal_(uint32_t group_index,
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
