#include "robot_socket.hpp"
#include <boost/asio/any_io_executor.hpp>
#include <boost/asio/awaitable.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/deferred.hpp>
#include <boost/asio/detached.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/use_awaitable.hpp>
#include <boost/asio/write.hpp>
#include <boost/core/span.hpp>
#include <boost/range/adaptor/copied.hpp>
#include <boost/range/adaptor/sliced.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/begin.hpp>
#include <boost/range/end.hpp>
#include <cstdint>
#include <cstring>
#include <exception>
#include <format>
#include <future>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <ostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>
#include "protocol.h"

#include <third_party/trajectories/Trajectory.h>

constexpr double k_waypoint_equivalancy_epsilon_rad = 1e-4;
constexpr double k_min_timestep_sec = 1e-2;  // determined experimentally, the arm appears to error when given timesteps ~2e-5 and lower

template <typename T>
[[nodiscard]] constexpr decltype(auto) degrees_to_radians(T&& degrees) {
    return std::forward<T>(degrees) * (M_PI / 180.0);
}

template <typename T>
[[nodiscard]] constexpr decltype(auto) radians_to_degrees(T&& radians) {
    return std::forward<T>(radians) * (180.0 / M_PI);
}

template <typename Func>
void sampling_func(std::vector<trajectory_point_t>& samples, double duration_sec, double sampling_frequency_hz, const Func& f) {
    if (duration_sec <= 0.0 || sampling_frequency_hz <= 0.0) {
        throw std::invalid_argument("duration_sec and sampling_frequency_hz are not both positive");
    }
    static constexpr std::size_t k_max_samples = 1000000;
    const auto putative_samples = duration_sec * sampling_frequency_hz;
    if (!std::isfinite(putative_samples) || putative_samples > k_max_samples) {
        throw std::invalid_argument("duration_sec and sampling_frequency_hz exceed the maximum allowable samples");
    }

    // Calculate the number of samples needed. this will always be at least 2.
    const auto num_samples = static_cast<std::size_t>(std::ceil(putative_samples) + 1);

    // Calculate the actual step size
    const double step = duration_sec / static_cast<double>((num_samples - 1));

    // Generate samples by evaluating f at each time point
    for (std::size_t i = 1; i < num_samples - 1; ++i) {
        samples.push_back(f(static_cast<double>(i) * step, step));
    }

    // Ensure the last sample uses exactly the duration_sec
    samples.push_back(f(duration_sec, step));
}

namespace robot {

using namespace boost::asio;

StatusMessage::StatusMessage(const Message& msg) {
    if (msg.header.message_type != MSG_ROBOT_POSITION_VELOCITY_TORQUE)
        throw std::runtime_error(
            std::format("wrong message status type expected {} had {}", (int)MSG_ROBOT_POSITION_VELOCITY_TORQUE, msg.header.message_type));
    if (msg.header.payload_length != msg.payload.size())
        throw std::runtime_error("incorrect status size");

    std::memcpy(&timestamp, msg.payload.data(), sizeof(timestamp));
    std::memcpy(&num_axes, msg.payload.data() + sizeof(timestamp), sizeof(num_axes));
    boost::span<const double> arrays{reinterpret_cast<const double*>(msg.payload.data() + sizeof(timestamp) + sizeof(num_axes)),
                                     4 * MAX_AXES};
    position.reserve(MAX_AXES);
    boost::copy(arrays | boost::adaptors::sliced(0, MAX_AXES), std::back_inserter(position));

    velocity.reserve(MAX_AXES);
    boost::copy(arrays | boost::adaptors::sliced(MAX_AXES, 2 * MAX_AXES), std::back_inserter(velocity));

    torque.reserve(MAX_AXES);
    boost::copy(arrays | boost::adaptors::sliced(2 * MAX_AXES, 3 * MAX_AXES), std::back_inserter(torque));
    position_corrected.reserve(MAX_AXES);
    boost::copy(arrays | boost::adaptors::sliced(3 * MAX_AXES, 4 * MAX_AXES), std::back_inserter(position_corrected));
}

RobotStatusMessage::RobotStatusMessage(const Message& msg) {
    if (msg.header.message_type != MSG_ROBOT_STATUS)
        throw std::runtime_error(
            std::format("wrong message status type expected {} had {}", (int)MSG_ROBOT_STATUS, msg.header.message_type));
    if (msg.header.payload_length != msg.payload.size())
        throw std::runtime_error("incorrect status size");

    const uint8_t* data = msg.payload.data();
    std::memcpy(&ts, data, sizeof(ts));
    data += sizeof(ts);

    std::memcpy(&mode, data, sizeof(mode));
    data += sizeof(mode);

    std::memcpy(&e_stopped, data, sizeof(e_stopped));
    data += sizeof(e_stopped);

    std::memcpy(&drives_powered, data, sizeof(drives_powered));
    data += sizeof(drives_powered);

    std::memcpy(&motion_possible, data, sizeof(motion_possible));
    data += sizeof(motion_possible);

    std::memcpy(&in_motion, data, sizeof(in_motion));
    data += sizeof(in_motion);

    std::memcpy(&in_error, data, sizeof(in_error));
    data += sizeof(in_error);

    error_codes.reserve(MAX_ALARM_COUNT + 1);
    boost::span<const int> alarm{reinterpret_cast<const int*>(data), MAX_ALARM_COUNT + 1};
    boost::copy(alarm | boost::adaptors::sliced(0, MAX_ALARM_COUNT + 1), std::back_inserter(error_codes));
    data += sizeof(int) * (MAX_ALARM_COUNT + 1);
    std::memcpy(&size, data, sizeof(size));
}

Message::Message(message_type_t type, std::vector<uint8_t> data) {
    header.magic_number = PROTOCOL_MAGIC_NUMBER;
    header.version = PROTOCOL_VERSION;
    header.message_type = static_cast<uint8_t>(type);
    header.timestamp_ms =
        (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    header.payload_length = static_cast<uint32_t>(data.size());
    payload = data;
}
Message::Message(protocol_header_t header, std::vector<uint8_t> payload) {
    this->header = header;
    this->payload = payload;
}

Message::Message(Message&& msg) noexcept : header(std::move(msg.header)), payload(std::move(msg.payload)) {}
Message::Message(const Message& msg) : payload(msg.payload) {
    std::memcpy(&header, &msg.header, sizeof(protocol_header_t));
}
Message& Message::operator=(const Message& other) {
    if (this == &other)
        return *this;
    this->header = other.header;
    this->payload.clear();
    this->payload = other.payload;
    return *this;
}

std::ostream& operator<<(std::ostream& os, const Message& msg) {
    os << "Message{type=" << static_cast<int>(msg.header.message_type) << ", timestamp=" << msg.header.timestamp_ms
       << ", payload_length=" << msg.header.payload_length << ", magic=0x" << std::hex << msg.header.magic_number << std::dec
       << ", version=" << static_cast<int>(msg.header.version) << "}";
    return os;
}

// TcpRobotSocket Implementation
TcpRobotSocket::TcpRobotSocket(boost::asio::io_context& io_context, const std::string& host, uint16_t port)
    : RobotSocketBase(io_context, host, port), socket_(io_context), request_queue_(io_context.get_executor()) {}

TcpRobotSocket::~TcpRobotSocket() {
    disconnect();
}

std::future<void> TcpRobotSocket::connect() {
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();
    co_spawn(
        io_context_,
        [this, promise]() mutable -> awaitable<void> {
            try {
                ip::tcp::resolver resolver(io_context_);
                std::cout << "connecting to " << host_ << ":" << port_ << " \n";
                auto endpoints = co_await resolver.async_resolve(host_, std::to_string(port_), use_awaitable);
                co_await async_connect(socket_, endpoints, use_awaitable);

                connected_ = true;
                running_ = true;

                // Start the request processing coroutine
                co_spawn(io_context_, process_requests(), detached);

                promise->set_value();
            } catch (const std::exception& e) {
                connected_ = false;
                promise->set_exception(std::current_exception());
            }
        },
        detached);

    return future;
}

std::future<Message> TcpRobotSocket::send_request(Message request) {
    std::promise<Message> promise;
    auto future = promise.get_future();
    if (!connected_) {
        promise.set_exception(std::make_exception_ptr(std::runtime_error("Not connected")));
    } else {
        request_queue_.push(std::make_pair(std::move(request), std::move(promise)));
    }
    return future;
}

void TcpRobotSocket::disconnect() {
    if (connected_) {
        connected_ = false;
        running_ = false;

        if (socket_.is_open()) {
            socket_.close();
        }
        while (!request_queue_.empty()) {
            auto promise = request_queue_.pop().second;
            promise.set_exception(std::make_exception_ptr(std::runtime_error("Connection closed")));
        }
    }
}

awaitable<void> TcpRobotSocket::process_requests() {
    while (running_) {
        auto [result, error] = co_await request_queue_.async_pop(boost::asio::use_awaitable);
        if (error) {
            throw std::runtime_error("boost error " + error.message() + " while waiting for a request");
        }

        if (!result) {
            throw std::runtime_error("empty request when expecting one");
        }
        auto request_pair = std::move(result.value());
        try {
            // Send request
            std::vector<uint8_t> buffer;
            buffer.resize(sizeof(protocol_header_t) + request_pair.first.payload.size());

            std::memcpy(buffer.data(), &request_pair.first.header, sizeof(protocol_header_t));
            if (!request_pair.first.payload.empty()) {
                std::memcpy(
                    buffer.data() + sizeof(protocol_header_t), request_pair.first.payload.data(), request_pair.first.payload.size());
            }
            co_await async_write(socket_, boost::asio::buffer(buffer), use_awaitable);
            // Try to read response
            std::vector<uint8_t> header_buffer(sizeof(protocol_header_t));
            size_t bytes_received = co_await socket_.async_read_some(boost::asio::buffer(header_buffer), use_awaitable);
            header_buffer.resize(bytes_received);
            if (bytes_received != sizeof(protocol_header_t)) {
                throw std::runtime_error("TCP failed to read header");
            }

            protocol_header_t header = parse_header(header_buffer);
            std::vector<uint8_t> payload_buffer(header.payload_length);

            bytes_received = co_await socket_.async_read_some(boost::asio::buffer(payload_buffer), use_awaitable);
            payload_buffer.resize(bytes_received);
            auto response = Message(header, payload_buffer);

            request_pair.second.set_value(response);

        } catch (const std::exception& e) {
            request_pair.second.set_exception(std::current_exception());
        }
    }
}

protocol_header_t RobotSocketBase::parse_header(std::vector<uint8_t>& buffer) {
    if (buffer.size() < sizeof(protocol_header_t)) {
        throw std::runtime_error("Invalid message: too short");
    }
    protocol_header_t header;
    std::memcpy(&header, buffer.data(), sizeof(protocol_header_t));
    if (header.magic_number != PROTOCOL_MAGIC_NUMBER || header.version != PROTOCOL_VERSION) {
        throw std::runtime_error("Invalid message: wrong magic number or version");
    }
    return header;
}

// UdpRobotSocket Implementation
UdpRobotSocket::UdpRobotSocket(boost::asio::io_context& io_context, State& state)
    : RobotSocketBase(io_context, "127.0.0.1", 0), robot_state_(state), socket_(io_context) {}

UdpRobotSocket::~UdpRobotSocket() {
    disconnect();
}

std::future<void> UdpRobotSocket::connect() {
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();
    try {
        socket_.open(ip::udp::v4());
        socket_.bind(ip::udp::endpoint(ip::udp::v4(), 0));

        connected_ = true;
        running_ = true;
        co_spawn(io_context_, receive_messages(), detached);

        promise->set_value();
    } catch (const std::exception& e) {
        connected_ = false;
        promise->set_exception(std::current_exception());
    }

    return future;
}

std::future<Message> UdpRobotSocket::send_request(Message /* request */) {
    std::promise<Message> promise;
    auto future = promise.get_future();
    promise.set_exception(std::make_exception_ptr(std::runtime_error("UDP socket is read-only")));
    return future;
}

void UdpRobotSocket::disconnect() {
    if (connected_) {
        connected_ = false;
        running_ = false;
        robot_state_.in_error.store(true);

        if (socket_.is_open()) {
            socket_.close();
        }
    }
}

void UdpRobotSocket::get_status(std::promise<Message> promise) {
    std::unique_lock lock(status_mutex_);

    cached_status_ = std::visit(
        [promise = std::move(promise)](auto& current) mutable -> std::variant<std::monostate, Message, std::promise<Message>> {
            using Type = std::decay_t<decltype(current)>;
            if constexpr (std::is_same_v<Type, std::monostate>) {
                return std::move(promise);
            } else if constexpr (std::is_same_v<Type, Message>) {
                promise.set_value(current);
                return current;
            } else {
                // NOTE SHOULD SET EXCEPTION INSTEAD
                return std::move(promise);
            }
        },
        cached_status_);
}

void UdpRobotSocket::get_robot_status(std::promise<Message> promise) {
    std::unique_lock lock(status_mutex_);

    cached_robot_status_ = std::visit(
        [promise = std::move(promise)](auto& current) mutable -> std::variant<std::monostate, Message, std::promise<Message>> {
            using Type = std::decay_t<decltype(current)>;
            if constexpr (std::is_same_v<Type, std::monostate>) {
                return std::move(promise);
            } else if constexpr (std::is_same_v<Type, Message>) {
                promise.set_value(current);
                return current;
            } else {
                // NOTE SHOULD SET EXCEPTION INSTEAD
                return std::move(promise);
            }
        },
        cached_robot_status_);
}

uint16_t UdpRobotSocket::get_local_port() const {
    if (socket_.is_open()) {
        return socket_.local_endpoint().port();
    }
    return 0;
}

awaitable<void> UdpRobotSocket::receive_messages() {
    while (running_) {
        try {
            std::vector<uint8_t> buffer(8192);
            ip::udp::endpoint sender_endpoint;
            size_t bytes_received = co_await socket_.async_receive_from(boost::asio::buffer(buffer), sender_endpoint, use_awaitable);

            buffer.resize(bytes_received);
            Message message = parse_message(buffer);

            // Log received message type
            if (message.header.message_type == MSG_ROBOT_POSITION_VELOCITY_TORQUE) {
                handle_status_message(message);
            } else if (message.header.message_type == MSG_ROBOT_STATUS) {
                handle_robot_status_message(message);
                robot_state_.UpdateState(RobotStatusMessage(message));
            } else {
            }

        } catch (const std::exception& e) {
            running_ = false;
            robot_state_.in_error.store(true);
            std::cout << "error " << e.what() << " while waiting on UDP messages" << std::endl;
        }
    }
}

void UdpRobotSocket::handle_status_message(const Message& message) {
    std::unique_lock lock(status_mutex_);
    cached_status_ = std::visit(
        [message](auto& current) mutable -> std::variant<std::monostate, Message, std::promise<Message>> {
            using Type = std::decay_t<decltype(current)>;
            if constexpr (std::is_same_v<Type, std::promise<Message>>) {
                current.set_value(message);
            }
            return message;
        },
        cached_status_);
}

void UdpRobotSocket::handle_robot_status_message(const Message& message) {
    std::unique_lock lock(status_mutex_);
    cached_robot_status_ = std::visit(
        [message](auto& current) mutable -> std::variant<std::monostate, Message, std::promise<Message>> {
            using Type = std::decay_t<decltype(current)>;
            if constexpr (std::is_same_v<Type, std::promise<Message>>) {
                current.set_value(message);
            }
            return message;
        },
        cached_robot_status_);
}

Message UdpRobotSocket::parse_message(const std::vector<uint8_t>& buffer) {
    if (buffer.size() < sizeof(protocol_header_t)) {
        throw std::runtime_error("Invalid message: too short");
    }

    Message message;
    std::memcpy(&message.header, buffer.data(), sizeof(protocol_header_t));

    if (message.header.magic_number != PROTOCOL_MAGIC_NUMBER || message.header.version != PROTOCOL_VERSION) {
        throw std::runtime_error(
            std::format("invalid message: wrong magic number or version expected magic : {:X} version: {} got magic: {:X} version: {}",
                        PROTOCOL_MAGIC_NUMBER,
                        PROTOCOL_VERSION,
                        (int)message.header.magic_number,
                        (int)message.header.version));
    }

    if (message.header.payload_length > 0) {
        if (buffer.size() < sizeof(protocol_header_t) + message.header.payload_length) {
            throw std::runtime_error(std::format("Invalid message: payload too short expected {} had {}",
                                                 (int)message.header.payload_length,
                                                 buffer.size() - sizeof(protocol_header_t)));
        }

        message.payload.resize(message.header.payload_length);
        std::memcpy(message.payload.data(), buffer.data() + sizeof(protocol_header_t), message.header.payload_length);
    }

    return message;
}

// Robot Implementation
YaskawaController::YaskawaController(boost::asio::io_context& io_context, double speed, double acceleration, const std::string& host)
    : io_context_(io_context), host_(host), robot_state_(State()), speed_(speed), acceleration_(acceleration) {
    tcp_socket_ = std::make_unique<TcpRobotSocket>(io_context_, host_);
}

std::future<void> YaskawaController::connect() {
    return std::async(std::launch::async, [this]() {
        try {
            tcp_socket_->connect().get();
            // Create and connect UDP socket
            udp_socket_ = std::make_unique<UdpRobotSocket>(io_context_, robot_state_);
            udp_socket_->connect().get();

            uint16_t local_udp_port = udp_socket_->get_local_port();
            auto registration_response = register_udp_port(local_udp_port).get();
            std::cout << " UDP port registration response: " << registration_response << " \n";
            // wait for first status;
            get_robot_status().get();

        } catch (const std::exception& e) {
            udp_socket_.reset();
            throw;
        }
    });
}

void YaskawaController::disconnect() {
    if (tcp_socket_) {
        tcp_socket_->disconnect();
    }
    if (udp_socket_) {
        udp_socket_->disconnect();
    }
}

std::future<Message> YaskawaController::get_goal_status(int32_t goal_id) {
    std::vector<uint8_t> payload(sizeof(cancel_goal_payload_t));
    cancel_goal_payload_t* req = reinterpret_cast<cancel_goal_payload_t*>(payload.data());
    req->goal_id = goal_id;

    return tcp_socket_->send_request(Message(MSG_GET_GOAL_STATUS, payload));
}

std::future<Message> YaskawaController::cancel_goal(int32_t goal_id) {
    std::vector<uint8_t> payload(sizeof(cancel_goal_payload_t));
    cancel_goal_payload_t* req = reinterpret_cast<cancel_goal_payload_t*>(payload.data());
    req->goal_id = goal_id;

    return tcp_socket_->send_request(Message(MSG_CANCEL_GOAL, payload));
}

std::future<Message> YaskawaController::send_test_trajectory() {
    if (!robot_state_.IsReady()) {
        throw std::runtime_error(std::format("cannot send test trajectory the robot state is e_stopped {} in error {}",
                                             robot_state_.e_stopped.load(),
                                             robot_state_.in_error.load()));
    }
    return tcp_socket_->send_request(Message(MSG_TEST_TRAJECTORY_COMMAND));
}

std::future<Message> YaskawaController::turn_servo_power_on() {
    if (!robot_state_.IsReady()) {
        throw std::runtime_error(std::format("cannot turn power on the robot state is e_stopped {} in error {}",
                                             robot_state_.e_stopped.load(),
                                             robot_state_.in_error.load()));
    }
    return tcp_socket_->send_request(Message(MSG_TURN_SERVO_POWER_ON));
}

std::future<Message> YaskawaController::send_heartbeat() {
    return tcp_socket_->send_request(Message(MSG_HEARTBEAT));
}

std::future<Message> YaskawaController::send_test_error_command() {
    return tcp_socket_->send_request(Message(MSG_TEST_ERROR_COMMAND));
}

std::future<Message> YaskawaController::get_error_info() {
    return tcp_socket_->send_request(Message(MSG_GET_ERROR_INFO));
}

std::future<Message> YaskawaController::get_robot_position_velocity_torque() {
    std::promise<Message> promise;
    auto future = promise.get_future();
    if (!udp_socket_) {
        promise.set_exception(std::make_exception_ptr(std::runtime_error("UDP socket not connected")));
        return future;
    }

    udp_socket_->get_status(std::move(promise));
    return future;
}

std::future<Message> YaskawaController::get_robot_status() {
    std::promise<Message> promise;
    auto future = promise.get_future();
    if (!udp_socket_) {
        promise.set_exception(std::make_exception_ptr(std::runtime_error("UDP socket not connected")));
        return future;
    }

    udp_socket_->get_robot_status(std::move(promise));
    return future;
}

std::future<Message> YaskawaController::register_udp_port(uint16_t port) {
    std::vector<uint8_t> payload(sizeof(udp_port_registration_payload_t));
    udp_port_registration_payload_t* port_payload = reinterpret_cast<udp_port_registration_payload_t*>(payload.data());
    port_payload->udp_port = port;

    return tcp_socket_->send_request(Message(MSG_REGISTER_UDP_PORT, payload));
}

std::future<Message> YaskawaController::reset_errors() {
    return tcp_socket_->send_request(Message(MSG_RESET_ERRORS));
}

std::future<Message> YaskawaController::send_goal_(uint32_t group_index,
                                                   uint32_t axes_controlled,
                                                   const std::vector<trajectory_point_t>& trajectory,
                                                   const std::vector<tolerance_t>& tolerance) {
    std::vector<uint8_t> payload;
    payload.reserve(sizeof(uint32_t) * 2 + trajectory.size() * sizeof(trajectory_point_t) + tolerance.size() * sizeof(tolerance_t));

    auto append_to = [&](auto obj) {
        const uint8_t* as_bytes = reinterpret_cast<const uint8_t*>(&obj);
        payload.insert(payload.end(), as_bytes, as_bytes + sizeof(obj));
    };
    append_to((uint32_t)axes_controlled);
    append_to((uint32_t)group_index);
    append_to((uint32_t)trajectory.size());
    boost::for_each(trajectory, append_to);
    append_to((uint32_t)tolerance.size());
    boost::for_each(tolerance, append_to);

    return tcp_socket_->send_request(Message(MSG_MOVE_GOAL, payload));
}

std::unique_ptr<GoalRequestHandle> YaskawaController::move(std::list<Eigen::VectorXd> waypoints, const std::string& unix_time) {
    if (!robot_state_.IsReady()) {
        throw std::runtime_error(std::format(
            "cannot move the robot state is e_stopped {} in error {}", robot_state_.e_stopped.load(), robot_state_.in_error.load()));
    }
    auto response = make_goal_(std::move(waypoints), unix_time).get();

    if (response.header.message_type != MSG_GOAL_ACCEPTED) {
        throw std::runtime_error(std::format("Expected MSG_GOAL_ACCEPTED, got {}", (int)response.header.message_type));
    }

    if (response.payload.size() != sizeof(goal_accepted_payload_t)) {
        throw std::runtime_error(std::format(
            "Invalid goal accepted payload size expected {} got {}", sizeof(goal_accepted_payload_t), (int)response.payload.size()));
    }
    const goal_accepted_payload_t* accepted = reinterpret_cast<const goal_accepted_payload_t*>(response.payload.data());
    auto promise = std::promise<goal_state_t>();
    auto handle = std::make_unique<GoalRequestHandle>(accepted->goal_id, shared_from_this(), promise.get_future());

    std::thread([promise = std::move(promise), self = shared_from_this(), goal_id = accepted->goal_id]() mutable {
        try {
            while (1) {
                auto status_msg = GoalStatusMessage(self->get_goal_status(goal_id).get());
                if (status_msg.state == GOAL_STATE_SUCCEEDED || status_msg.state == GOAL_STATE_CANCELLED ||
                    status_msg.state == GOAL_STATE_ABORTED) {
                    promise.set_value_at_thread_exit(status_msg.state);
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } catch (...) {
            try {
                promise.set_exception(std::current_exception());
            } catch (...) {
            }
        }
    }).detach();
    return handle;
}

std::future<Message> YaskawaController::make_goal_(std::list<Eigen::VectorXd> waypoints, const std::string& unix_time) {
    std::cout << "move: start unix_time_ms " << unix_time << " waypoints size " << waypoints.size() << "\n";

    // get current joint position and add that as starting pose to waypoints
    std::cout << "move: get_joint_positions start " << unix_time << "\n";
    auto curr_joint_pos = StatusMessage(get_robot_position_velocity_torque().get()).position;
    std::cout << "move: get_joint_positions end " << unix_time << "\n";

    std::cout << "move: compute_trajectory start " << unix_time << "\n";
    auto curr_waypoint_deg = Eigen::VectorXd::Map(curr_joint_pos.data(), boost::numeric_cast<Eigen::Index>(curr_joint_pos.size()));
    auto curr_waypoint_rad = degrees_to_radians(std::move(curr_waypoint_deg)).eval();
    if (!curr_waypoint_rad.isApprox(waypoints.front(), k_waypoint_equivalancy_epsilon_rad)) {
        waypoints.emplace_front(std::move(curr_waypoint_rad));
    }
    if (waypoints.size() == 1) {  // this tells us if we are already at the goal
        throw std::runtime_error("arm is already at the desired joint positions");
    }

    // Walk all interior points of the waypoints list, if any. If the
    // point of current interest is the cusp of a direction reversal
    // w.r.t. the points immediately before and after it, then splice
    // all waypoints up to and including the cusp point into a new
    // segment, and then begin accumulating a new segment starting at
    // the cusp point. The cusp point is duplicated, forming both the
    // end of one segment and the beginning of the next segment. After
    // exiting the loop, any remaining waypoints form the last (and if
    // no cusps were identified the only) segment. If one or more cusp
    // points were identified, the waypoints list will always have at
    // least two residual waypoints, since the last waypoint is never
    // examined, and the splice call never removes the waypoint being
    // visited.
    //
    // NOTE: This assumes waypoints have been de-duplicated to avoid
    // zero-length segments that would cause numerical issues in
    // normalized() calculations.
    std::vector<decltype(waypoints)> segments;
    for (auto where = next(begin(waypoints)); where != prev(end(waypoints)); ++where) {
        const auto segment_ab = *where - *prev(where);
        const auto segment_bc = *next(where) - *where;
        const auto dot = segment_ab.normalized().dot(segment_bc.normalized());
        if (std::fabs(dot + 1.0) < 1e-3) {
            segments.emplace_back();
            segments.back().splice(segments.back().begin(), waypoints, waypoints.begin(), where);
            segments.back().push_back(*where);
        }
    }
    segments.push_back(std::move(waypoints));

    // set velocity/acceleration constraints
    const auto max_velocity = Eigen::VectorXd::Constant(6, speed_);
    const auto max_acceleration = Eigen::VectorXd::Constant(6, acceleration_);
    std::cout << "generating trajectory with max speed: " << radians_to_degrees(max_velocity[0]) << "\n";

    std::vector<trajectory_point_t> points;

    for (const auto& segment : segments) {
        const Trajectory trajectory(Path(segment, 0.1), max_velocity, max_acceleration);
        trajectory.outputPhasePlaneTrajectory();
        if (!trajectory.isValid()) {
            std::stringstream buffer;
            buffer << "trajectory generation failed for path:";
            for (const auto& position : segment) {
                buffer << "{";
                for (Eigen::Index j = 0; j < 6; j++) {
                    buffer << position[j] << " ";
                }
                buffer << "}";
            }
            throw std::runtime_error(buffer.str());
        }

        const double duration = trajectory.getDuration();
        if (!std::isfinite(duration)) {
            throw std::runtime_error("trajectory.getDuration() was not a finite number");
        }
        // TODO(RSDK-11069): Make this configurable
        // https://viam.atlassian.net/browse/RSDK-11069
        if (duration > 600) {  // if the duration is longer than 10 minutes
            throw std::runtime_error("trajectory.getDuration() exceeds 10 minutes");
        }
        if (duration < k_min_timestep_sec) {
            throw std::runtime_error(std::format("duration {} lower than {} assuming arm is at goal", duration, k_min_timestep_sec));
        }

        // desired sampling frequency. if the duration is small we will oversample but that should be fine.
        constexpr double k_sampling_freq_hz = 5;
        sampling_func(points, duration, k_sampling_freq_hz, [&](const double t, const double step) {
            auto p_eigen = trajectory.getPosition(t);
            auto v_eigen = trajectory.getVelocity(t);
            return trajectory_point_t{{p_eigen[0], p_eigen[1], p_eigen[2], p_eigen[3], p_eigen[4], p_eigen[5]},
                                      {v_eigen[0], v_eigen[1], v_eigen[2], v_eigen[3], v_eigen[4], v_eigen[5]},
                                      {0},
                                      {0},
                                      {boost::numeric_cast<int32_t>(step), 0}};
        });
    }
    std::cout << "move: compute_trajectory end " << unix_time << " samples.size() " << points.size() << " segments " << segments.size() - 1
              << "\n";

    return send_goal_(group_index_, 6, points, {});
}

std::future<Message> YaskawaController::echo_trajectory() {
    // Echo trajectory command has no payload
    return tcp_socket_->send_request(Message(MSG_ECHO_TRAJECTORY));
}

bool YaskawaController::is_status_command(message_type_t type) const {
    return type == MSG_ROBOT_POSITION_VELOCITY_TORQUE || type == MSG_ROBOT_STATUS;
}

// GoalStatusMessage implementation
GoalStatusMessage::GoalStatusMessage(const Message& msg) {
    if (msg.header.message_type != MSG_GOAL_STATUS)
        throw std::runtime_error(std::format("wrong message type expected {} had {}", (int)MSG_GOAL_STATUS, msg.header.message_type));
    if (msg.payload.size() != sizeof(goal_status_payload_t))
        throw std::runtime_error(
            std::format("incorrect goal status payload size expected {} had {}", sizeof(goal_status_payload_t), msg.payload.size()));

    const goal_status_payload_t* payload = reinterpret_cast<const goal_status_payload_t*>(msg.payload.data());
    goal_id = payload->goal_id;
    state = static_cast<goal_state_t>(payload->state);
    progress = payload->progress;
    timestamp_ms = payload->timestamp_ms;
}

// GoalHandle implementation
GoalRequestHandle::GoalRequestHandle(int32_t goal_id,
                                     std::shared_ptr<YaskawaController> controller,
                                     std::shared_future<goal_state_t> completion_future)
    : goal_id_(goal_id), is_done_(false), controller_(controller), completion_future_(completion_future) {}

goal_state_t GoalRequestHandle::wait() {
    return completion_future_.get();
}

std::optional<goal_state_t> GoalRequestHandle::wait_for(std::chrono::milliseconds timeout) {
    if (completion_future_.wait_for(timeout) == std::future_status::ready) {
        return completion_future_.get();
    }
    return std::nullopt;
}

std::future<GoalStatusMessage> GoalRequestHandle::get_status() {
    return std::async(std::launch::async, [this]() mutable -> GoalStatusMessage {
        auto response = controller_->get_goal_status(goal_id_).get();
        if (response.header.message_type == MSG_ERROR) {
            throw std::runtime_error(std::format("received an error message while getting status for goal id {}", goal_id_));
        }
        return GoalStatusMessage(response);
    });
}

void GoalRequestHandle::cancel() {
    auto response = controller_->cancel_goal(goal_id_).get();
    if (response.header.message_type == MSG_ERROR) {
        throw std::runtime_error(std::format("received an error message while cancelling  goal id {}", goal_id_));
    }
}

bool GoalRequestHandle::is_done() const {
    return completion_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
}

State::State() : e_stopped(false), in_motion(false), drive_powered(false), in_error(true) {}
void State::UpdateState(RobotStatusMessage msg) {
    e_stopped.store(msg.e_stopped);
    in_error.store(msg.in_error);
    drive_powered.store(msg.drives_powered);
    in_motion.store(msg.in_motion);
}
bool State::IsReady() {
    return !e_stopped.load() && !in_error.load();
}
}  // namespace robot
