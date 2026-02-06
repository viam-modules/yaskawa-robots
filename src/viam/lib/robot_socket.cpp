#include "robot_socket.hpp"
#include <algorithm>
#include <boost/asio/any_io_executor.hpp>
#include <boost/asio/awaitable.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/deferred.hpp>
#include <boost/asio/detached.hpp>
#include <boost/asio/experimental/awaitable_operators.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/use_awaitable.hpp>
#include <boost/asio/write.hpp>
#include <boost/core/span.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/range/adaptor/copied.hpp>
#include <boost/range/adaptor/sliced.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/begin.hpp>
#include <boost/range/end.hpp>
#include <boost/regex.hpp>
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
#include <viam/module/utils.hpp>

namespace {

constexpr double k_default_waypoint_deduplication_tolerance_rads = 1e-3;
constexpr double k_default_min_timestep_sec = 1e-2;
constexpr double k_default_trajectory_sampling_freq = 3;

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
    static constexpr std::size_t k_max_samples = 2000000;
    const auto putative_samples = duration_sec * sampling_frequency_hz;
    if (!std::isfinite(putative_samples) || putative_samples > k_max_samples) {
        throw std::invalid_argument(
            "duration_sec and sampling_frequency_hz exceed "
            "the maximum allowable samples");
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

}  // namespace

namespace robot {

using namespace boost::asio;

/// Parse cartesian position from a protocol message
/// Validates message type and payload size before extracting position data
CartesianPosition::CartesianPosition(const Message& msg) {
    // Validate message type
    if ((msg.header.message_type != MSG_GET_CART) && (msg.header.message_type != MSG_FROM_JOINT_TO_CART)) {
        throw std::runtime_error(
            std::format("wrong message status type expected MSG_GET_CART or "
                        "MSG_FROM_JOINT_TO_CART had {}",
                        msg.header.message_type));
    }

    // Validate payload size to prevent buffer overruns
    if (msg.payload.size() != sizeof(cartesian_payload_t)) {
        throw std::runtime_error(std::format(
            "incorrect cartesian payload size: expected {} bytes, got {} bytes", sizeof(cartesian_payload_t), msg.payload.size()));
    }

    // Safe deserialization: verify alignment before reinterpret_cast
    if (reinterpret_cast<uintptr_t>(msg.payload.data()) % alignof(cartesian_payload_t) != 0) {
        throw std::runtime_error("cartesian payload data is not properly aligned");
    }

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

/// currently only used in example/main.cpp
std::string CartesianPosition::toString() const {
    return std::format(" ({},{},{}) - ({},{},{}) ", x, y, z, rx, ry, rz);
}
/// Parse joint angle position from a protocol message
/// Validates message type and payload size before extracting angle data
AnglePosition::AnglePosition(const Message& msg) {
    // Validate message type
    if (!(msg.header.message_type == MSG_FROM_CART_TO_JOINT)) {
        throw std::runtime_error(std::format("wrong message status type expected MSG_FROM_CART_TO_JOINT had {}", msg.header.message_type));
    }

    // Validate payload size to prevent buffer overruns
    if (msg.payload.size() != sizeof(position_angle_degree_payload_t)) {
        throw std::runtime_error(std::format(
            "incorrect angle payload size: expected {} bytes, got {} bytes", sizeof(position_angle_degree_payload_t), msg.payload.size()));
    }

    // Safe deserialization: verify alignment before reinterpret_cast
    if (reinterpret_cast<uintptr_t>(msg.payload.data()) % alignof(position_angle_degree_payload_t) != 0) {
        throw std::runtime_error("angle payload data is not properly aligned");
    }

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
/// Validates that the position vector has at least 6 dimensions before
/// formatting.
/// currently only used in example/main.cpp
std::string AnglePosition::toString() {
    // Validate that we have at least 6 joint angles
    if (pos.size() < 6) {
        std::ostringstream buffer;
        buffer << "AnglePosition[invalid: only " << pos.size() << " dimensions, expected at least 6]";
        return buffer.str();
    }

    // Format the first 6 joint angles (standard for 6-axis robot)
    return std::format(
        "AnglePosition[{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}] (degrees)", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
}

StatusMessage::StatusMessage(const Message& msg) {
    if (msg.header.message_type != MSG_ROBOT_POSITION_VELOCITY_TORQUE) {
        throw std::runtime_error(
            std::format("wrong message status type expected {} had {}", (int)MSG_ROBOT_POSITION_VELOCITY_TORQUE, msg.header.message_type));
    }
    if (msg.header.payload_length != msg.payload.size()) {
        throw std::runtime_error("incorrect status size");
    }

    std::memcpy(&timestamp, msg.payload.data(), sizeof(timestamp));
    std::memcpy(&num_axes, msg.payload.data() + sizeof(timestamp), sizeof(num_axes));
    boost::span<const double> arrays{reinterpret_cast<const double*>(msg.payload.data() + sizeof(timestamp) + sizeof(num_axes)),
                                     static_cast<size_t>(4) * MAX_AXES};
    position.reserve(MAX_AXES);
    boost::copy(arrays | boost::adaptors::sliced(0, MAX_AXES), std::back_inserter(position));

    velocity.reserve(MAX_AXES);
    boost::copy(arrays | boost::adaptors::sliced(MAX_AXES, static_cast<size_t>(2) * MAX_AXES), std::back_inserter(velocity));

    torque.reserve(MAX_AXES);
    boost::copy(arrays | boost::adaptors::sliced(static_cast<size_t>(2) * MAX_AXES, static_cast<size_t>(3) * MAX_AXES),
                std::back_inserter(torque));
    position_corrected.reserve(MAX_AXES);
    boost::copy(arrays | boost::adaptors::sliced(static_cast<size_t>(3) * MAX_AXES, static_cast<size_t>(4) * MAX_AXES),
                std::back_inserter(position_corrected));
}

RobotStatusMessage::RobotStatusMessage(const Message& msg) {
    if (msg.header.message_type != MSG_ROBOT_STATUS) {
        throw std::runtime_error(
            std::format("wrong message status type expected {} had {}", (int)MSG_ROBOT_STATUS, msg.header.message_type));
    }
    if (msg.header.payload_length != msg.payload.size()) {
        throw std::runtime_error("incorrect status size");
    }

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

CheckGroupMessage::CheckGroupMessage(const Message& msg) {
    if (msg.header.message_type != MSG_CHECK_GROUP) {
        throw std::runtime_error(
            std::format("wrong message status type expected {} had {}", (int)MSG_CHECK_GROUP, msg.header.message_type));
    }

    // Validate payload size to prevent buffer overruns
    if (msg.payload.size() != sizeof(boolean_payload_t)) {
        throw std::runtime_error(std::format(
            "incorrect boolean_payload_t payload size: expected {} bytes, got {} bytes", sizeof(boolean_payload_t), msg.payload.size()));
    }

    // Safe deserialization: verify alignment before reinterpret_cast
    if (reinterpret_cast<uintptr_t>(msg.payload.data()) % alignof(boolean_payload_t) != 0) {
        throw std::runtime_error("boolean payload data is not properly aligned");
    }

    const boolean_payload_t* group_check = reinterpret_cast<const boolean_payload_t*>(msg.payload.data());
    is_known_group = group_check->value;
}

Message::Message(message_type_t type, std::vector<uint8_t>&& data) : payload(std::move(data)) {
    header.magic_number = PROTOCOL_MAGIC_NUMBER;
    header.version = PROTOCOL_VERSION;
    header.message_type = static_cast<uint8_t>(type);
    header.timestamp_ms =
        (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    header.payload_length = static_cast<uint32_t>(payload.size());
}
Message::Message(protocol_header_t header, std::vector<uint8_t>&& payload) : header(header), payload(std::move(payload)) {}

Message::Message(Message&& msg) noexcept : header(std::move(msg.header)), payload(std::move(msg.payload)) {}
Message::Message(const Message& msg) : payload(msg.payload) {
    std::memcpy(&header, &msg.header, sizeof(protocol_header_t));
}
Message& Message::operator=(const Message& other) {
    if (this == &other) {
        return *this;
    }
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
    : RobotSocketBase(io_context, host, port),
      socket_(io_context),
      request_queue_(AsyncQueue<std::pair<Message, std::promise<Message>>>::create(io_context.get_executor())) {}

TcpRobotSocket::~TcpRobotSocket() {
    try {
        disconnect();
    } catch (const std::exception& ex) {
        LOGGING(error) << "error while closing TCP connection" << ex.what();
    }
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
        request_queue_->push(std::make_pair(std::move(request), std::move(promise)));
    }
    return future;
}

void TcpRobotSocket::disconnect() {
    if (connected_) {
        // stop processing requests
        running_.store(false);
        connected_ = false;
        if (socket_.is_open()) {
            socket_.close();
        }

        while (!request_queue_->empty()) {
            auto promise = request_queue_->pop().second;
            promise.set_exception(std::make_exception_ptr(std::runtime_error("Connection closed")));
        }
    }
}

awaitable<void> TcpRobotSocket::process_requests() {
    while (running_.load()) {
        auto [result, error] = co_await request_queue_->async_pop(boost::asio::use_awaitable);
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
            co_await async_read(socket_, boost::asio::buffer(header_buffer), use_awaitable);

            const protocol_header_t header = parse_header(header_buffer);
            std::vector<uint8_t> payload_buffer(header.payload_length);

            if (header.payload_length > 0) {
                co_await async_read(socket_, boost::asio::buffer(payload_buffer), use_awaitable);
            }
            auto response = Message(header, std::move(payload_buffer));

            request_pair.second.set_value(response);

        } catch (const std::exception& e) {
            request_pair.second.set_exception(std::current_exception());
        }
    }
}

protocol_header_t RobotSocketBase::parse_header(const std::vector<uint8_t>& buffer) {
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
    try {
        disconnect();
    } catch (const std::exception& ex) {
        LOGGING(error) << "error while closing UDP connection" << ex.what();
    }
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
        running_ = false;
        robot_state_.in_error.store(true);
        connected_ = false;
        if (socket_.is_open()) {
            socket_.close();
        }
    }
}

void UdpRobotSocket::get_status(std::promise<Message> promise) {
    if (!connected_) {
        throw std::runtime_error("socket is disconnected");
    }
    const std::unique_lock lock(status_mutex_);

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
    if (!connected_) {
        throw std::runtime_error("socket is disconnected");
    }
    const std::unique_lock lock(status_mutex_);

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
            const size_t bytes_received = co_await socket_.async_receive_from(boost::asio::buffer(buffer), sender_endpoint, use_awaitable);

            buffer.resize(bytes_received);
            const Message message = parse_message(buffer);
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
    const std::unique_lock lock(status_mutex_);
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
    const std::unique_lock lock(status_mutex_);
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
            std::format("invalid message: wrong magic number or version expected magic : {:X} "
                        "version: {} got magic: {:X} version: {}",
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
    : io_context_(io_context), socket_(io_context), port_(port) {
    log_parser_ = std::make_unique<viam::yaskawa::ViamControllerLogParser>();
}

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
        // Flush any remaining data in the parser
        if (log_parser_) {
            log_parser_->flush();
        }

        if (socket_.is_open()) {
            socket_.close();
        }
        LOGGING(info) << "UDP broadcast listener stopped";
    } catch (const std::exception& e) {
        LOGGING(error) << "Error stopping UDP broadcast listener: " << e.what();
    }
}

boost::asio::awaitable<void> UdpBroadcastListener::receive_broadcasts() {
    using namespace boost::asio::experimental::awaitable_operators;
    while (running_) {
        try {
            udp::endpoint sender_endpoint;
            auto timer = boost::asio::steady_timer(io_context_);
            timer.expires_after(std::chrono::milliseconds(200));
            // Receive broadcast message
            auto result = co_await(timer.async_wait(use_awaitable) ||
                                   socket_.async_receive_from(boost::asio::buffer(recv_buffer_), sender_endpoint, use_awaitable));

            if (std::holds_alternative<std::size_t>(result)) {
                const size_t bytes_received = std::get<std::size_t>(result);
                if (bytes_received > 0 && running_) {
                    // Null-terminate the buffer to ensure safe string operations
                    const std::size_t str_pos = std::min(bytes_received, recv_buffer_.size() - 1);
                    recv_buffer_[str_pos] = '\0';

                    // Process the data through the log parser
                    if (log_parser_) {
                        log_parser_->process_data(recv_buffer_.data());
                    }
                } else {
                    log_parser_->flush();
                }
            } else {
                log_parser_->flush();
            }
        } catch (const std::exception& e) {
            if (running_) {
                LOGGING(error) << "Error receiving broadcast: " << e.what();
            }
            break;
        }
    }
    log_parser_->flush();
}

// Robot Implementation
YaskawaController::YaskawaController(boost::asio::io_context& io_context, const viam::sdk::ResourceConfig& config)
    : io_context_(io_context) {
    host_ = find_config_attribute<std::string>(config, "host").value();
    speed_ = find_config_attribute<double>(config, "speed_rad_per_sec").value();
    acceleration_ = find_config_attribute<double>(config, "acceleration_rad_per_sec2").value();

    auto group_index = find_config_attribute<double>(config, "group_index");
    constexpr int k_min_group_index = 0;
    // TODO(RSDK-12470) support multiple arms
    constexpr int k_max_group_index = 0;
    if (group_index && (*group_index < k_min_group_index || *group_index > k_max_group_index || floor(*group_index) != *group_index)) {
        throw std::invalid_argument(std::format("attribute `group_index` should be a whole number between {} and {} , it is : {}",
                                                k_min_group_index,
                                                k_max_group_index,
                                                *group_index));
    }
    group_index_ = static_cast<std::uint32_t>(group_index.value_or(k_min_group_index));
    trajectory_sampling_freq_ =
        find_config_attribute<double>(config, "trajectory_sampling_freq_hz").value_or(k_default_trajectory_sampling_freq);

    auto waypoint_dedup_tolerance_deg = find_config_attribute<double>(config, "waypoint_deduplication_tolerance_deg");
    waypoint_dedup_tolerance_rad_ =
        waypoint_dedup_tolerance_deg ? degrees_to_radians(*waypoint_dedup_tolerance_deg) : k_default_waypoint_deduplication_tolerance_rads;

    tcp_socket_ = std::make_unique<TcpRobotSocket>(io_context_, host_);
    broadcast_listener_ = std::make_unique<UdpBroadcastListener>(io_context_);
}

void YaskawaController::set_trajectory_loggers(std::string robot_model,
                                               std::optional<std::function<std::optional<std::string>()>> telemetry_path_fn) {
    robot_model_ = std::move(robot_model);
    telemetry_path_fn_ = std::move(telemetry_path_fn);
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
            const uint16_t local_udp_port = udp_socket_->get_local_port();
            auto registration_response = register_udp_port(local_udp_port).get();
            LOGGING(info) << "UDP port registration response: " << registration_response;

            // Wait for first status update to ensure connection is fully established
            get_robot_status().get();

            heartbeat_ = std::thread([self = weak_from_this()]() {
                try {
                    while (1) {
                        auto shared = self.lock();
                        if (!shared) {
                            return;
                        }
                        shared->send_heartbeat().get();
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                } catch (const std::exception& e) {
                    LOGGING(info) << "heartbeat thread terminated with " << e.what();
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
    if (udp_socket_) {
        udp_socket_->disconnect();
    }
    if (tcp_socket_) {
        tcp_socket_->disconnect();
    }
    if (broadcast_listener_) {
        broadcast_listener_->stop();
    }
    if (heartbeat_.joinable()) {
        LOGGING(debug) << "Yaskawa Controller shutdown waiting for heartbeat thread to terminate";
        // the heartbeat thread will shutdown once the tcp_socket has closed. Wait for the heartbeat thread to finish
        try {
            heartbeat_.join();
        } catch (const std::exception& e) {
            LOGGING(debug) << "Yaskawa Controller shutdown failed to join heartbeat thread with " << e.what();
        }

        LOGGING(debug) << "heartbeat thread terminated";
    }
}

uint32_t YaskawaController::get_group_index() const {
    return group_index_;
}

double YaskawaController::get_waypoint_deduplication_tolerance_rad() const {
    return waypoint_dedup_tolerance_rad_;
}

std::future<Message> YaskawaController::get_goal_status(int32_t goal_id) {
    std::vector<uint8_t> payload(sizeof(cancel_goal_payload_t));
    cancel_goal_payload_t* req = reinterpret_cast<cancel_goal_payload_t*>(payload.data());
    req->goal_id = goal_id;

    return tcp_socket_->send_request(Message(MSG_GET_GOAL_STATUS, std::move(payload)));
}

std::future<Message> YaskawaController::cancel_goal(int32_t goal_id) {
    std::vector<uint8_t> payload(sizeof(cancel_goal_payload_t));
    cancel_goal_payload_t* req = reinterpret_cast<cancel_goal_payload_t*>(payload.data());
    req->goal_id = goal_id;

    return tcp_socket_->send_request(Message(MSG_CANCEL_GOAL, std::move(payload)));
}

std::future<Message> YaskawaController::setMotionMode(uint8_t mode) {
    std::vector<uint8_t> payload(sizeof(motion_mode_payload_t));
    motion_mode_payload_t* req = reinterpret_cast<motion_mode_payload_t*>(payload.data());
    req->motion_mode = mode;

    return tcp_socket_->send_request(Message(MSG_SET_MOTION_MODE, std::move(payload)));
}

std::future<Message> YaskawaController::send_test_trajectory() {
    if (!robot_state_.IsReady()) {
        throw std::runtime_error(
            std::format("cannot send test trajectory the robot state is e_stopped {} in error "
                        "{}",
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
    // TODO(RSDK-12470) account for group_id_ in request
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
    return tcp_socket_->send_request(Message(MSG_REGISTER_UDP_PORT, std::move(payload)));
}

std::future<Message> YaskawaController::reset_errors() {
    return tcp_socket_->send_request(Message(MSG_RESET_ERRORS));
}

GoalAcceptedMessage YaskawaController::send_goal_(uint32_t group_index,
                                                  uint32_t axes_controlled,
                                                  const std::vector<trajectory_point_t>& trajectory,
                                                  const std::vector<tolerance_t>& tolerance) {
    std::vector<uint8_t> payload;
    payload.reserve((sizeof(uint32_t) * 3) + (trajectory.size() * sizeof(trajectory_point_t)) + (tolerance.size() * sizeof(tolerance_t)));

    auto append_to = [&](auto obj) {
        const uint8_t* as_bytes = reinterpret_cast<const uint8_t*>(&obj);
        payload.insert(payload.end(), as_bytes, as_bytes + sizeof(obj));
    };

    append_to(axes_controlled);
    append_to(group_index);
    append_to(static_cast<uint32_t>(trajectory.size()));
    boost::for_each(trajectory, append_to);
    append_to(static_cast<uint32_t>(tolerance.size()));
    boost::for_each(tolerance, append_to);

    return GoalAcceptedMessage(tcp_socket_->send_request(Message(MSG_MOVE_GOAL, std::move(payload))).get());
}

std::unique_ptr<GoalRequestHandle> YaskawaController::move(std::list<Eigen::VectorXd> waypoints, const std::string& unix_time) {
    if (!robot_state_.IsReady()) {
        auto msg = reset_errors().get();
        if (msg.header.message_type == MSG_ERROR) {
            error_payload_t err_msg;
            std::memcpy(&err_msg, msg.payload.data(), sizeof(err_msg));
            throw std::runtime_error(std::format("failed to reset arm, error code {}", static_cast<const int&>(err_msg.error_code)));
        }
        if (msg.header.message_type != MSG_OK) {
            throw std::runtime_error(std::format("failed to reset arm, got unexpected message type {}", msg.header.message_type));
        }
    }
    // TODO check servo on and & errors
    turn_servo_power_on().get();
    setMotionMode(1).get();

    auto promise = std::promise<goal_state_t>();

    auto goal_result = make_goal_(std::move(waypoints), unix_time);
    if (!goal_result) {
        LOGGING(debug) << "already at desired position";
        promise.set_value(GOAL_STATE_SUCCEEDED);
        return std::make_unique<GoalRequestHandle>(0, shared_from_this(), promise.get_future());
    }

    const auto& accepted = goal_result->accepted;
    LOGGING(info) << "goal accepted: goal_id=" << accepted.goal_id << " num_trajectory_accepted=" << accepted.num_trajectory_accepted;

    auto handle = std::make_unique<GoalRequestHandle>(accepted.goal_id, shared_from_this(), promise.get_future());

    // Derive poll interval from trajectory sampling frequency
    const auto poll_interval = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / trajectory_sampling_freq_));
    LOGGING(info) << "polling interval=" << poll_interval;
    // Single thread handles both chunk streaming and goal monitoring
    std::thread([promise = std::move(promise),
                 self = weak_from_this(),
                 goal_id = accepted.goal_id,
                 remaining = std::move(goal_result->remaining_trajectory),
                 poll_interval]() mutable {
        try {
            constexpr size_t chunk_size = 200;
            constexpr size_t queue_threshold = 50;
            size_t offset = 0;

            while (true) {
                std::this_thread::sleep_for(poll_interval);
                auto shared = self.lock();
                if (!shared) {
                    return;
                }

                const auto status_msg = GoalStatusMessage(shared->get_goal_status(goal_id).get());

                switch (status_msg.state) {
                    case GOAL_STATE_ACTIVE:
                        // Stream remaining chunks when queue is running low
                        if (offset < remaining.size()) {
                            LOGGING(debug) << "queue size: " << status_msg.current_queue_size << " points";
                            if (status_msg.current_queue_size <= queue_threshold) {
                                const size_t end = std::min(offset + chunk_size, remaining.size());
                                const std::vector<trajectory_point_t> chunk(std::next(remaining.begin(), static_cast<ptrdiff_t>(offset)),
                                                                            std::next(remaining.begin(), static_cast<ptrdiff_t>(end)));

                                LOGGING(debug) << "sending chunk: points " << offset << " to " << end << " (" << chunk.size() << " points)";

                                // tolerance can be threaded through here when needed
                                const auto chunk_accepted = shared->send_goal_(shared->group_index_, 6, chunk, {});
                                LOGGING(debug) << "chunk accepted: " << chunk_accepted.num_trajectory_accepted << " points";
                                offset += chunk_accepted.num_trajectory_accepted;
                            }
                        }
                        break;
                    case GOAL_STATE_SUCCEEDED:
                        if (RobotStatusMessage(shared->get_robot_status().get()).in_motion) {
                            break;
                        }
                        promise.set_value_at_thread_exit(status_msg.state);
                        return;
                    case GOAL_STATE_PENDING:
                        break;
                    case GOAL_STATE_CANCELLED:
                    case GOAL_STATE_ABORTED:
                        throw std::runtime_error(std::format("goal failed - goal status is {}", goal_state_to_string(status_msg.state)));
                }
            }
        } catch (std::exception& e) {
            try {
                promise.set_exception_at_thread_exit(std::current_exception());
            } catch (...) {  // NOLINT(bugprone-empty-catch)
            }
        }
    }).detach();
    return handle;
}

std::optional<MakeGoalResult> YaskawaController::make_goal_(std::list<Eigen::VectorXd> waypoints, const std::string& unix_time) {
    LOGGING(info) << "move: start unix_time_ms " << unix_time << " waypoints size " << waypoints.size();

    auto curr_joint_pos = StatusMessage(get_robot_position_velocity_torque().get()).position;

    auto curr_waypoint_rad = Eigen::VectorXd::Map(curr_joint_pos.data(), boost::numeric_cast<Eigen::Index>(curr_joint_pos.size()));
    if (!curr_waypoint_rad.isApprox(waypoints.front(), get_waypoint_deduplication_tolerance_rad())) {
        waypoints.emplace_front(std::move(curr_waypoint_rad));
    }
    if (waypoints.size() == 1) {  // this tells us if we are already at the goal
        return std::nullopt;
    }

    // Save original waypoints for logging before segmentation modifies the list
    const std::list<Eigen::VectorXd> original_waypoints = waypoints;

    std::vector<std::list<Eigen::VectorXd>> segments;
    // Walk interior points and cut at 180 degree direction reversals (dot == -1)
    // github.com/viam-modules/universal-robots/blob/2eaa3243984a299b6565920f4dc60c6fe0ddc8ef/src/viam/ur/module/ur_arm.cpp#L684
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

    // Remainder becomes the last (or only) segment
    segments.push_back(std::move(waypoints));

    // set velocity/acceleration constraints
    const auto max_velocity_vec = Eigen::VectorXd::Constant(6, speed_);
    const auto max_acceleration_vec = Eigen::VectorXd::Constant(6, acceleration_);

    std::vector<trajectory_point_t> samples;
    std::chrono::duration<double> cumulative_time{0};

    for (const auto& segment : segments) {
        const Trajectory trajectory(Path(segment, 0.1), max_velocity_vec, max_acceleration_vec);
        if (!trajectory.isValid()) {
            const std::string error_msg = "trajectory.isValid() was false";
            if (telemetry_path_fn_) {
                auto telemetry_path = (*telemetry_path_fn_)();
                if (telemetry_path) {
                    FailedTrajectoryLogger::log_failure(
                        *telemetry_path, unix_time, robot_model_, group_index_, speed_, acceleration_, original_waypoints, error_msg);
                }
            }
            throw std::runtime_error(error_msg);
        }

        const double duration = trajectory.getDuration();

        if (!std::isfinite(duration)) {
            const std::string error_msg = "trajectory.getDuration() was not a finite number";

            if (telemetry_path_fn_) {
                auto telemetry_path = (*telemetry_path_fn_)();
                if (telemetry_path) {
                    FailedTrajectoryLogger::log_failure(
                        *telemetry_path, unix_time, robot_model_, group_index_, speed_, acceleration_, original_waypoints, error_msg);
                }
            }

            throw std::runtime_error(error_msg);
        }

        // TODO(RSDK-12566): Make this configurable
        // viam.atlassian.net/browse/RSDK-12566
        if (duration > 600) {  // if the duration is longer than 10 minutes
            const std::string error_msg = "trajectory.getDuration() exceeds 10 minutes";

            if (telemetry_path_fn_) {
                auto telemetry_path = (*telemetry_path_fn_)();
                if (telemetry_path) {
                    FailedTrajectoryLogger::log_failure(
                        *telemetry_path, unix_time, robot_model_, group_index_, speed_, acceleration_, original_waypoints, error_msg);
                }
            }

            throw std::runtime_error(error_msg);
        }

        if (duration < k_default_min_timestep_sec) {
            LOGGING(debug) << "duration of move is too small, assuming arm is at goal";
            continue;
        }

        trajectory.outputPhasePlaneTrajectory();

        // Add the t=0 point for the first segment
        if (samples.empty()) {
            auto p_eigen = trajectory.getPosition(0.0);
            auto v_eigen = trajectory.getVelocity(0.0);
            samples.push_back(trajectory_point_t{{p_eigen[0], p_eigen[1], p_eigen[2], p_eigen[3], p_eigen[4], p_eigen[5]},
                                                 {v_eigen[0], v_eigen[1], v_eigen[2], v_eigen[3], v_eigen[4], v_eigen[5]},
                                                 {0},
                                                 {0},
                                                 {0, 0}});
        }

        sampling_func(samples, duration, trajectory_sampling_freq_, [&](const double t, const double) {
            auto p_eigen = trajectory.getPosition(t);
            auto v_eigen = trajectory.getVelocity(t);
            const auto absolute_time = cumulative_time + std::chrono::duration<double>(t);
            auto secs = std::chrono::floor<std::chrono::seconds>(absolute_time);
            auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(absolute_time - secs);
            return trajectory_point_t{{p_eigen[0], p_eigen[1], p_eigen[2], p_eigen[3], p_eigen[4], p_eigen[5]},
                                      {v_eigen[0], v_eigen[1], v_eigen[2], v_eigen[3], v_eigen[4], v_eigen[5]},
                                      {0},
                                      {0},
                                      {static_cast<int32_t>(secs.count()), static_cast<int32_t>(nanos.count())}};
        });

        cumulative_time += std::chrono::duration<double>(duration);
    }

    // Log the successfully generated trajectory
    if (telemetry_path_fn_) {
        auto telemetry_path = (*telemetry_path_fn_)();
        if (telemetry_path) {
            GeneratedTrajectoryLogger::log_trajectory(
                *telemetry_path, unix_time, robot_model_, group_index_, speed_, acceleration_, original_waypoints, samples);
        }
    }

    constexpr size_t chunk_size = 200;  // controller cannot exceed 200 points per message
    LOGGING(debug) << "total trajectory points: " << samples.size();

    // Send the first chunk
    const auto first_end = std::min(chunk_size, samples.size());
    const std::vector<trajectory_point_t> first_chunk(samples.begin(), std::next(samples.begin(), static_cast<ptrdiff_t>(first_end)));
    // tolerance can be threaded through here when needed
    auto accepted = send_goal_(group_index_, 6, first_chunk, {});

    // Return accepted message + any remaining trajectory points
    const size_t offset = accepted.num_trajectory_accepted;
    std::vector<trajectory_point_t> remaining;
    if (offset < samples.size()) {
        remaining.assign(std::next(samples.begin(), static_cast<ptrdiff_t>(offset)), samples.end());
    }

    return MakeGoalResult{std::move(accepted), std::move(remaining)};
}

std::future<Message> YaskawaController::echo_trajectory() {
    // Echo trajectory command has no payload
    return tcp_socket_->send_request(Message(MSG_ECHO_TRAJECTORY));
}
std::future<Message> YaskawaController::stop() {
    // TODO(RSDK-12470) account for group_index_ in request
    return tcp_socket_->send_request(Message(MSG_STOP_MOTION));
}
std::future<Message> YaskawaController::getCartPosition() {
    std::vector<uint8_t> payload(sizeof(group_id_t));
    group_id_t* id = reinterpret_cast<group_id_t*>(payload.data());
    id->group_id = (int32_t)group_index_;
    return tcp_socket_->send_request(Message(MSG_GET_CART, std::move(payload)));
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
    return tcp_socket_->send_request(Message(MSG_FROM_CART_TO_JOINT, std::move(payload)));
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
    return tcp_socket_->send_request(Message(MSG_FROM_JOINT_TO_CART, std::move(payload)));
}

bool YaskawaController::is_status_command(message_type_t type) {
    return type == MSG_ROBOT_POSITION_VELOCITY_TORQUE || type == MSG_ROBOT_STATUS;
}

std::future<Message> YaskawaController::checkGroupIndex() {
    std::vector<uint8_t> payload(sizeof(group_id_t));
    group_id_t* id = reinterpret_cast<group_id_t*>(payload.data());
    id->group_id = (int32_t)group_index_;
    return tcp_socket_->send_request(Message(MSG_CHECK_GROUP, std::move(payload)));
}

// GoalStatusMessage implementation
GoalStatusMessage::GoalStatusMessage(const Message& msg) {
    if (msg.header.message_type != MSG_GOAL_STATUS) {
        throw std::runtime_error(std::format("wrong message type expected {} had {}", (int)MSG_GOAL_STATUS, msg.header.message_type));
    }
    if (msg.payload.size() != sizeof(goal_status_payload_t)) {
        throw std::runtime_error(
            std::format("incorrect goal status payload size expected {} had {}", sizeof(goal_status_payload_t), msg.payload.size()));
    }

    const goal_status_payload_t* payload = reinterpret_cast<const goal_status_payload_t*>(msg.payload.data());
    goal_id = payload->goal_id;
    state = static_cast<goal_state_t>(payload->state);
    current_queue_size = payload->current_queue_size;
    progress = payload->progress;
    timestamp_ms = payload->timestamp_ms;
}

// GoalAcceptedMessage implementation
GoalAcceptedMessage::GoalAcceptedMessage(const Message& msg) {
    if (msg.header.message_type != MSG_GOAL_ACCEPTED) {
        throw std::runtime_error(
            std::format("wrong message type expected {} had {}", static_cast<int>(MSG_GOAL_ACCEPTED), msg.header.message_type));
    }
    if (msg.payload.size() != sizeof(goal_accepted_payload_t)) {
        throw std::runtime_error(
            std::format("incorrect goal accepted payload size expected {} had {}", sizeof(goal_accepted_payload_t), msg.payload.size()));
    }

    const goal_accepted_payload_t* payload = reinterpret_cast<const goal_accepted_payload_t*>(msg.payload.data());
    goal_id = payload->goal_id;
    num_trajectory_accepted = payload->num_trajectory_accepted;
    timestamp_ms = payload->timestamp_ms;
}

// GoalHandle implementation
GoalRequestHandle::GoalRequestHandle(int32_t goal_id,
                                     const std::shared_ptr<YaskawaController>& controller,
                                     std::shared_future<goal_state_t> completion_future)
    : goal_id_(goal_id), is_done_(false), controller_(controller), completion_future_(std::move(completion_future)) {}

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
        auto shared = controller_.lock();
        if (!shared) {
            throw std::runtime_error("YaskawaController no longer exists");
        }
        auto response = shared->get_goal_status(goal_id_).get();
        if (response.header.message_type == MSG_ERROR) {
            throw std::runtime_error(std::format("received an error message while getting status for goal id {}", goal_id_));
        }
        return GoalStatusMessage(response);
    });
}

void GoalRequestHandle::cancel() {
    auto shared = controller_.lock();
    if (shared) {
        auto response = shared->cancel_goal(goal_id_).get();
        if (response.header.message_type == MSG_ERROR) {
            throw std::runtime_error(std::format("received an error message while cancelling  goal id {}", goal_id_));
        }
    }
}

bool GoalRequestHandle::is_done() const {
    return completion_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
}

State::State() : e_stopped(false), in_motion(false), drive_powered(false), in_error(true) {}
void State::UpdateState(const RobotStatusMessage& msg) {
    e_stopped.store(msg.e_stopped);
    in_error.store(msg.in_error);
    drive_powered.store(msg.drives_powered);
    in_motion.store(msg.in_motion);
}
bool State::IsReady() const {
    return !e_stopped.load() && !in_error.load();
}
}  // namespace robot
