#include "robot_socket.hpp"
#include <algorithm>
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
#include <chrono>
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
#include <viam/lib/logger.hpp>
#include "protocol.h"

#include <third_party/trajectories/Trajectory.h>

// Tolerance for comparing waypoint positions to detect duplicates (radians)
constexpr double k_waypoint_equivalancy_epsilon_rad = 1e-4;

// Minimum timestep between trajectory points (seconds)
// Determined experimentally: the arm appears to error when given timesteps ~2e-5 and lower
constexpr double k_min_timestep_sec = 1e-2;

constexpr const char* goal_state_to_string(goal_state_t state) {
    switch (state) {
        case GOAL_STATE_SUCCEEDED:
            return "goal succeeded";
        case GOAL_STATE_PENDING:
            return "goal pending";
        case GOAL_STATE_ABORTED:
            return "goal aborted (server side)";
        case GOAL_STATE_CANCELLED:
            return "goal cancelled (client side)";
        case GOAL_STATE_ACTIVE:
            return "goal state active";
        default:
            return "unknown goal state";
    }
}

template <typename Func>
void sampling_func(std::vector<trajectory_point_t>& samples, double duration_sec, double sampling_frequency_hz, const Func& f) {
    if (duration_sec <= 0.0 || sampling_frequency_hz <= 0.0) {
        throw std::invalid_argument("duration_sec and sampling_frequency_hz are not both positive");
    }
    static constexpr std::size_t k_max_samples = 200;
    const auto putative_samples = duration_sec * sampling_frequency_hz;
    if (!std::isfinite(putative_samples) || putative_samples > k_max_samples) {
        throw std::invalid_argument("duration_sec and sampling_frequency_hz exceed the maximum allowable samples");
    }
    // Calculate the number of samples needed. this will always be at least 2.
    const auto num_samples = static_cast<std::size_t>(std::ceil(putative_samples) + 1);

    // Calculate the actual step size
    const double step = duration_sec / static_cast<double>((num_samples - 1));

    // Generate samples by evaluating f at each time point
    for (std::size_t i = 0; i < num_samples - 1; ++i) {
        samples.push_back(f(static_cast<double>(i) * step, step));
    }

    // Ensure the last sample uses exactly the duration_sec
    samples.push_back(f(duration_sec, step));
}

namespace robot {

using namespace boost::asio;

/// Parse cartesian position from a protocol message
/// Validates message type and payload size before extracting position data
CartesianPosition::CartesianPosition(const Message& msg) {
    // Validate message type
    if (!(msg.header.message_type == MSG_GET_CART || msg.header.message_type == MSG_FROM_JOINT_TO_CART))
        throw std::runtime_error(
            std::format("wrong message status type expected MSG_GET_CART or MSG_FROM_JOINT_TO_CART had {}", msg.header.message_type));

    // Validate payload size to prevent buffer overruns
    if (msg.payload.size() != sizeof(cartesian_payload_t))
        throw std::runtime_error(std::format(
            "incorrect cartesian payload size: expected {} bytes, got {} bytes", sizeof(cartesian_payload_t), msg.payload.size()));

    // Safe deserialization: verify alignment before reinterpret_cast
    if (reinterpret_cast<uintptr_t>(msg.payload.data()) % alignof(cartesian_payload_t) != 0)
        throw std::runtime_error("cartesian payload data is not properly aligned");

    const cartesian_payload_t* cart_coord = reinterpret_cast<const cartesian_payload_t*>(msg.payload.data());
    x = cart_coord->cartesianCoord[0];
    y = cart_coord->cartesianCoord[1];
    z = cart_coord->cartesianCoord[2];
    rx = cart_coord->cartesianCoord[3];
    ry = cart_coord->cartesianCoord[4];
    rz = cart_coord->cartesianCoord[5];
}
CartesianPosition::CartesianPosition(const CartesianPosition& other) {
    x = other.x;
    y = other.y;
    z = other.z;
    rx = other.rx;
    ry = other.ry;
    rz = other.rz;
}
std::string CartesianPosition::toString() noexcept {
    return std::format(" ({},{},{}) - ({},{},{}) ", x, y, z, rx, ry, rz);
}
/// Parse joint angle position from a protocol message
/// Validates message type and payload size before extracting angle data
AnglePosition::AnglePosition(const Message& msg) {
    // Validate message type
    if (!(msg.header.message_type == MSG_FROM_CART_TO_JOINT))
        throw std::runtime_error(std::format("wrong message status type expected MSG_FROM_CART_TO_JOINT had {}", msg.header.message_type));

    // Validate payload size to prevent buffer overruns
    if (msg.payload.size() != sizeof(position_angle_degree_payload_t))
        throw std::runtime_error(std::format(
            "incorrect angle payload size: expected {} bytes, got {} bytes", sizeof(position_angle_degree_payload_t), msg.payload.size()));

    // Safe deserialization: verify alignment before reinterpret_cast
    if (reinterpret_cast<uintptr_t>(msg.payload.data()) % alignof(position_angle_degree_payload_t) != 0)
        throw std::runtime_error("angle payload data is not properly aligned");

    const position_angle_degree_payload_t* retPos = reinterpret_cast<const position_angle_degree_payload_t*>(msg.payload.data());
    pos.reserve(8);
    boost::copy(retPos->positionAngleDegree, std::back_inserter(pos));
}
AnglePosition::AnglePosition(std::vector<double> posRad) {
    std::transform(posRad.begin(), posRad.end(), posRad.begin(), [](double d) -> double { return d * (180.0 / M_PI); });
    pos.reserve(8);
    boost::copy(posRad, std::back_inserter(pos));
}
void AnglePosition::toRad() {
    std::transform(pos.begin(), pos.end(), pos.begin(), [](double d) -> double { return d * (M_PI / 180.0); });
}
/// Convert angle position to string representation
/// Validates that the position vector has at least 6 dimensions before formatting
std::string AnglePosition::toString() noexcept {
    // Validate that we have at least 6 joint angles
    if (pos.size() < 6) {
        return std::format("AnglePosition[invalid: only {} dimensions, expected at least 6]", pos.size());
    }

    // Format the first 6 joint angles (standard for 6-axis robot)
    return std::format(
        "AnglePosition[{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}] (degrees)", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
}

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
                LOGGING(info) << "connecting to " << host_ << ":" << port_;
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
    if (connected_ == false)
        throw std::runtime_error("socket is disconnected");
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
                // Concurrent status requests not supported
                promise.set_exception(std::make_exception_ptr(std::runtime_error("Concurrent status requests are not supported")));
                return std::move(current);
            }
        },
        cached_status_);
}

void UdpRobotSocket::get_robot_status(std::promise<Message> promise) {
    if (connected_ == false)
        throw std::runtime_error("socket is disconnected");
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
                // Concurrent robot status requests not supported
                promise.set_exception(std::make_exception_ptr(std::runtime_error("Concurrent robot status requests are not supported")));
                return std::move(current);
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
            LOGGING(error) << "error " << e.what() << " while waiting on UDP messages";
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

// UdpBroadcastListener Implementation
UdpBroadcastListener::UdpBroadcastListener(boost::asio::io_context& io_context, uint16_t port)
    : io_context_(io_context), socket_(io_context), port_(port) {}

UdpBroadcastListener::~UdpBroadcastListener() {
    stop();
}

void UdpBroadcastListener::start() {
    if (running_.exchange(true)) {
        return;  // Already running
    }

    try {
        // Open socket and bind to broadcast port
        socket_.open(udp::v4());
        socket_.set_option(udp::socket::reuse_address(true));
        socket_.bind(udp::endpoint(udp::v4(), port_));

        // Start receiving broadcasts
        co_spawn(io_context_, receive_broadcasts(), detached);

        LOGGING(info) << "UDP broadcast listener started on port " << port_;
    } catch (const std::exception& e) {
        running_ = false;
        LOGGING(error) << "Failed to start UDP broadcast listener: " << e.what();
        throw;
    }
}

void UdpBroadcastListener::stop() {
    if (!running_.exchange(false)) {
        return;  // Already stopped
    }

    try {
        if (socket_.is_open()) {
            socket_.close();
        }
        LOGGING(info) << "UDP broadcast listener stopped";
    } catch (const std::exception& e) {
        LOGGING(error) << "Error stopping UDP broadcast listener: " << e.what();
    }
}

boost::asio::awaitable<void> UdpBroadcastListener::receive_broadcasts() {
    while (running_) {
        try {
            udp::endpoint sender_endpoint;

            // Receive broadcast message
            std::size_t bytes_received =
                co_await socket_.async_receive_from(boost::asio::buffer(recv_buffer_), sender_endpoint, use_awaitable);

            if (bytes_received > 0 && running_) {
                // Null-terminate the string (handle case where buffer is full)
                std::size_t str_len = bytes_received;

                // Find null terminator or use full buffer
                for (std::size_t i = 0; i < bytes_received; ++i) {
                    if (recv_buffer_[i] == '\0') {
                        str_len = i;
                        break;
                    }
                }

                // Parse and log the broadcast message
                std::string message(recv_buffer_.data(), str_len);
                if (!message.empty()) {
                    // TODO deconstruct incoming log messages
                    LOGGING(info) << message;
                }
            }
        } catch (const std::exception& e) {
            if (running_) {
                LOGGING(error) << "Error receiving broadcast: " << e.what();
            }
            break;
        }
    }
}

// Robot Implementation
YaskawaController::YaskawaController(boost::asio::io_context& io_context, double speed, double acceleration, const std::string& host)
    : io_context_(io_context), host_(host), robot_state_(State()), speed_(speed), acceleration_(acceleration) {
    tcp_socket_ = std::make_unique<TcpRobotSocket>(io_context_, host_);
    broadcast_listener_ = std::make_unique<UdpBroadcastListener>(io_context_);
    group_index_ = 0;
}

std::future<void> YaskawaController::connect() {
    return std::async(std::launch::async, [this]() {
        try {
            // Establish TCP connection for commands
            tcp_socket_->connect().get();

            // Create and connect UDP socket for receiving status updates
            udp_socket_ = std::make_unique<UdpRobotSocket>(io_context_, robot_state_);
            udp_socket_->connect().get();

            // Register UDP port with robot so it knows where to send status messages
            uint16_t local_udp_port = udp_socket_->get_local_port();
            auto registration_response = register_udp_port(local_udp_port).get();
            LOGGING(info) << "UDP port registration response: " << registration_response;

            // Wait for first status update to ensure connection is fully established
            get_robot_status().get();

            heartbeat_ = std::thread([self = shared_from_this()]() {
                try {
                    while (1) {
                        self->send_heartbeat().get();
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                } catch (const std::exception& e) {
                    LOGGING(error) << "heartbeat thread terminated with " << e.what();
                }
            });

            // Start listening for broadcast messages from robot
            broadcast_listener_->start();

        } catch (const std::exception& e) {
            // Clean up on connection failure
            broadcast_listener_.reset();
            udp_socket_.reset();
            throw;
        }
    });
}

void YaskawaController::disconnect() {
    if (broadcast_listener_) {
        broadcast_listener_->stop();
    }
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

std::future<Message> YaskawaController::setMotionMode(uint8_t mode) {
    std::vector<uint8_t> payload(sizeof(motion_mode_payload_t));
    motion_mode_payload_t* req = reinterpret_cast<motion_mode_payload_t*>(payload.data());
    req->motion_mode = mode;

    return tcp_socket_->send_request(Message(MSG_SET_MOTION_MODE, payload));
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
        reset_errors().get();
        throw std::runtime_error(std::format(
            "cannot move the robot state is e_stopped {} in error {}", robot_state_.e_stopped.load(), robot_state_.in_error.load()));
    }
    // TODO check servo on and & errors

    turn_servo_power_on().get();
    setMotionMode(1).get();

    auto response = make_goal_(std::move(waypoints), unix_time).get();
    LOGGING(info) << response;
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
                if (status_msg.state == GOAL_STATE_ACTIVE)
                    continue;
                if (status_msg.state == GOAL_STATE_SUCCEEDED) {
                    promise.set_value_at_thread_exit(status_msg.state);
                    break;
                } else {
                    throw std::runtime_error(std::format("goal failed - goal status is {}", goal_state_to_string(status_msg.state)));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } catch (std::exception& e) {
            try {
                promise.set_exception_at_thread_exit(std::current_exception());
            } catch (...) {
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }).detach();
    return handle;
}

std::future<Message> YaskawaController::make_goal_(std::list<Eigen::VectorXd> waypoints, const std::string& unix_time) {
    LOGGING(info) << "move: start unix_time_ms " << unix_time << " waypoints size " << waypoints.size();

    auto curr_joint_pos = StatusMessage(get_robot_position_velocity_torque().get()).position;

    auto curr_waypoint_rad = Eigen::VectorXd::Map(curr_joint_pos.data(), boost::numeric_cast<Eigen::Index>(curr_joint_pos.size()));
    if (!curr_waypoint_rad.isApprox(waypoints.front(), k_waypoint_equivalancy_epsilon_rad)) {
        waypoints.emplace_front(std::move(curr_waypoint_rad));
    }
    if (waypoints.size() == 1) {  // this tells us if we are already at the goal
        throw std::invalid_argument("arm is already at the desired joint positions");
    }

    // set velocity/acceleration constraints
    const auto max_velocity = Eigen::VectorXd::Constant(6, speed_);
    const auto max_acceleration = Eigen::VectorXd::Constant(6, acceleration_);
    LOGGING(info) << "generating trajectory with max speed: " << radians_to_degrees(max_velocity[0]);

    std::vector<trajectory_point_t> points;

    const Trajectory trajectory(Path(waypoints, 0.1), max_velocity, max_acceleration);
    if (!trajectory.isValid()) {
        std::stringstream buffer;
        buffer << "trajectory generation failed for path:";
        for (const auto& position : waypoints) {
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
    constexpr double k_sampling_freq_hz = 3;
    sampling_func(points, duration, k_sampling_freq_hz, [&](const double t, const double) {
        auto p_eigen = trajectory.getPosition(t);
        auto v_eigen = trajectory.getVelocity(t);
        auto secs = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(t));
        auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(t) - secs);

        return trajectory_point_t{{p_eigen[0], p_eigen[1], p_eigen[2], p_eigen[3], p_eigen[4], p_eigen[5]},
                                  {v_eigen[0], v_eigen[1], v_eigen[2], v_eigen[3], v_eigen[4], v_eigen[5]},
                                  {0},
                                  {0},
                                  {(int32_t)secs.count(), (int32_t)nanos.count()}};
    });

    LOGGING(info) << "move: compute_trajectory end " << unix_time << " samples.size() " << points.size() << " segments "
                  << waypoints.size() - 1;

    return send_goal_(group_index_, 6, points, {});
}

std::future<Message> YaskawaController::echo_trajectory() {
    // Echo trajectory command has no payload
    return tcp_socket_->send_request(Message(MSG_ECHO_TRAJECTORY));
}
std::future<Message> YaskawaController::stop() {
    return tcp_socket_->send_request(Message(MSG_STOP_MOTION));
}
std::future<Message> YaskawaController::getCartPosition() {
    std::vector<uint8_t> payload(sizeof(group_id_t));
    group_id_t* id = reinterpret_cast<group_id_t*>(payload.data());
    id->group_id = (int32_t)group_index_;
    return tcp_socket_->send_request(Message(MSG_GET_CART, payload));
}
std::future<Message> YaskawaController::cartPosToAngle(CartesianPosition& pos) {
    std::vector<uint8_t> payload(sizeof(cartesian_payload_t));
    cartesian_payload_t* cid = reinterpret_cast<cartesian_payload_t*>(payload.data());
    cid->group_id = (int32_t)group_index_;
    cid->cartesianCoord[0] = pos.x;
    cid->cartesianCoord[1] = pos.y;
    cid->cartesianCoord[2] = pos.z;
    cid->cartesianCoord[3] = pos.rx;
    cid->cartesianCoord[4] = pos.ry;
    cid->cartesianCoord[5] = pos.rz;
    return tcp_socket_->send_request(Message(MSG_FROM_CART_TO_JOINT, payload));
}
std::future<Message> YaskawaController::angleToCartPos(AnglePosition& pos) {
    std::vector<uint8_t> payload(sizeof(position_angle_degree_payload_t));
    position_angle_degree_payload_t* pid = reinterpret_cast<position_angle_degree_payload_t*>(payload.data());
    pid->group_id = (int32_t)group_index_;
    pid->positionAngleDegree[0] = pos.pos[0];
    pid->positionAngleDegree[1] = pos.pos[1];
    pid->positionAngleDegree[2] = pos.pos[2];
    pid->positionAngleDegree[3] = pos.pos[3];
    pid->positionAngleDegree[4] = pos.pos[4];
    pid->positionAngleDegree[5] = pos.pos[5];
    return tcp_socket_->send_request(Message(MSG_FROM_JOINT_TO_CART, payload));
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
