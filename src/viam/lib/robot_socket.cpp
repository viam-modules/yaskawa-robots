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
#include <fstream>
#include <future>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <ostream>
#include <ranges>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>
#include <viam/lib/logger.hpp>
#include "protocol.h"
#include "scope_guard.hpp"

#include <third_party/trajectories/Trajectory.h>
#include <viam/module/utils.hpp>
#include <viam/sdk/common/proto_value.hpp>

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/containers/xarray.hpp>
#else
#include <xtensor/xarray.hpp>
#endif

#include <viam/trajex/totg/tools/planner.hpp>
#include <viam/trajex/totg/totg.hpp>
#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/totg/uniform_sampler.hpp>
#include <viam/trajex/totg/waypoint_utils.hpp>
#include <viam/trajex/types/hertz.hpp>

namespace {

xt::xarray<double> eigen_waypoints_to_xarray(const std::list<Eigen::VectorXd>& waypoints) {
    if (waypoints.empty()) {
        return xt::xarray<double>::from_shape({0, 0});
    }

    const size_t num_waypoints = waypoints.size();
    const size_t dof = static_cast<size_t>(waypoints.front().size());

    xt::xarray<double> result = xt::zeros<double>({num_waypoints, dof});

    size_t i = 0;
    for (const auto& waypoint : waypoints) {
        for (size_t j = 0; j < dof; ++j) {
            result(i, j) = waypoint[static_cast<Eigen::Index>(j)];
        }
        ++i;
    }

    return result;
}

constexpr double k_default_waypoint_deduplication_tolerance_rads = 1e-3;
constexpr double k_default_segmentation_threshold = 0.005;
// Build a trajectory_point_t from dynamic-sized position/velocity containers.
// Copies up to NUMBER_OF_DOF elements; remaining slots are zero-initialized.
// Works with Eigen::VectorXd (operator[]) and xt::xarray<double> (operator()).
template <typename Container>
trajectory_point_t make_trajectory_point(const Container& positions, const Container& velocities, duration_t time_from_start) {
    trajectory_point_t pt{};
    const auto n = std::min(static_cast<size_t>(NUMBER_OF_DOF), static_cast<size_t>(positions.size()));
    for (size_t i = 0; i < n; ++i) {
        pt.positions[i] = positions(static_cast<Eigen::Index>(i));
    }
    const auto nv = std::min(static_cast<size_t>(NUMBER_OF_DOF), static_cast<size_t>(velocities.size()));
    for (size_t i = 0; i < nv; ++i) {
        pt.velocities[i] = velocities(static_cast<Eigen::Index>(i));
    }
    pt.time_from_start = time_from_start;
    return pt;
}

struct segment_accumulator {
    std::vector<trajectory_point_t> samples;
    std::chrono::duration<double> cumulative_time{0};
    double total_duration = 0.0;
    double total_generation_time = 0.0;
    size_t total_waypoints = 0;
    double total_arc_length = 0.0;
    size_t segment_count = 0;
};

using viam::ScopeGuard;
constexpr double k_default_min_timestep_sec = 1e-2;
constexpr double k_default_trajectory_sampling_freq = 3;
constexpr auto k_socket_timeout = std::chrono::seconds(5);

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

constexpr size_t k_chunk_size = 200;  // controller cannot exceed 200 points per message

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

    // v6 layout: [group_index:1][group_type:1][timestamp:8][num_axes:1][4×MAX_AXES doubles]
    const uint8_t* data = msg.payload.data();
    std::memcpy(&group_index, data, sizeof(group_index));
    data += sizeof(group_index);

    std::memcpy(&group_type, data, sizeof(group_type));
    data += sizeof(group_type);

    std::memcpy(&timestamp, data, sizeof(timestamp));
    data += sizeof(timestamp);

    std::memcpy(&num_axes, data, sizeof(num_axes));
    data += sizeof(num_axes);

    if (num_axes > MAX_AXES) {
        throw std::runtime_error(std::format("status num_axes {} exceeds MAX_AXES {}", static_cast<int>(num_axes), MAX_AXES));
    }

    std::array<double, static_cast<size_t>(4) * MAX_AXES> arrays{};
    std::memcpy(arrays.data(), data, sizeof(arrays));
    const auto axes = static_cast<size_t>(num_axes);

    position.assign(arrays.begin(), arrays.begin() + axes);
    velocity.assign(arrays.begin() + MAX_AXES, arrays.begin() + MAX_AXES + axes);
    torque.assign(arrays.begin() + (static_cast<size_t>(2) * MAX_AXES), arrays.begin() + (static_cast<size_t>(2) * MAX_AXES) + axes);
    position_corrected.assign(arrays.begin() + (static_cast<size_t>(3) * MAX_AXES),
                              arrays.begin() + (static_cast<size_t>(3) * MAX_AXES) + axes);
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

    std::memcpy(&group_index, data, sizeof(group_index));
    data += sizeof(group_index);

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

CapabilitiesMessage::CapabilitiesMessage(const Message& msg) {
    if (msg.header.message_type != MSG_CAPABILITIES) {
        throw std::runtime_error(
            std::format("wrong message type: expected {} had {}", static_cast<int>(MSG_CAPABILITIES), msg.header.message_type));
    }
    if (msg.payload.size() < sizeof(uint8_t) * 2) {
        throw std::runtime_error("capabilities payload too small");
    }

    const uint8_t* data = msg.payload.data();
    protocol_version = *data++;
    num_groups = *data++;

    if (num_groups > MAX_GROUPS) {
        throw std::runtime_error(std::format("capabilities num_groups {} exceeds MAX_GROUPS {}", num_groups, MAX_GROUPS));
    }

    groups.resize(num_groups);
    for (uint8_t i = 0; i < num_groups; ++i) {
        const auto* cap = reinterpret_cast<const group_capability_t*>(data + (i * sizeof(group_capability_t)));
        groups[i].group_id = cap->group_id;
        groups[i].group_type = cap->group_type;
        groups[i].group_sub_index = cap->group_sub_index;
        groups[i].num_axes = cap->num_axes;
        groups[i].axis_types.assign(cap->axis_types, cap->axis_types + cap->num_axes);
        groups[i].base_axis_motion.assign(cap->base_axis_motion, cap->base_axis_motion + cap->num_axes);
        groups[i].interpolation_period_us = cap->interpolation_period_us;
    }
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

// get_error checks the message for an error and returns the appropriate error message based on the result.
std::string Message::get_error(message_type_t expected_type) const {
    if (header.message_type == expected_type) {
        return "";
    }

    if (header.message_type == MSG_ERROR) {
        error_payload_t err_msg{};
        std::memcpy(&err_msg, payload.data(), std::min(payload.size(), sizeof(err_msg)));
        // Parse both error_code and message string from error_payload_t for better diagnostics.
        // The original code only returned err_msg.message which could be empty on some errors.
        int32_t code = err_msg.error_code;
        std::string msg = std::format("received error code {}", code);
        if (err_msg.message[0] != '\0') {
            err_msg.message[sizeof(err_msg.message) - 1] = '\0';
            msg += std::format(": {}", err_msg.message);
        }
        return msg;
    }

    return std::format("unexpected message type expected {} got {}", static_cast<const int&>(expected_type), header.message_type);
}

// TcpRobotSocket Implementation
TcpRobotSocket::TcpRobotSocket(boost::asio::io_context& io_context, const std::string& host, uint16_t port)
    : RobotSocketBase(io_context, host, port), session_(std::make_shared<Session>(io_context, io_context.get_executor())) {}

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
                co_await async_connect(session_->socket_, endpoints, use_awaitable);

                connected_ = true;
                co_spawn(io_context_, process_requests(session_), detached);

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
        session_->queue_->push(std::make_pair(std::move(request), std::move(promise)));
    }
    return future;
}

void TcpRobotSocket::disconnect() {
    if (connected_) {
        connected_ = false;
        session_->queue_->close();
        session_->queue_->clear();
        if (session_->socket_.is_open()) {
            session_->socket_.close();
        }
    }
}

awaitable<void> TcpRobotSocket::process_requests(std::shared_ptr<Session> session) {
    try {
        while (true) {
            auto [result, error] = co_await session->queue_->async_pop(boost::asio::use_awaitable);
            if (error) {
                LOGGING(debug) << "TCP process_requests: queue error: " << error.message();
                break;
            }
            if (!result) {
                LOGGING(error) << "TCP process_requests: queue returned empty result";
                break;
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
                co_await async_write(session->socket_, boost::asio::buffer(buffer), use_awaitable);
                // Try to read response
                std::vector<uint8_t> header_buffer(sizeof(protocol_header_t));
                co_await async_read(session->socket_, boost::asio::buffer(header_buffer), use_awaitable);

                const protocol_header_t header = parse_header(header_buffer);
                std::vector<uint8_t> payload_buffer(header.payload_length);

                if (header.payload_length > 0) {
                    co_await async_read(session->socket_, boost::asio::buffer(payload_buffer), use_awaitable);
                }
                auto response = Message(header, std::move(payload_buffer));

                request_pair.second.set_value(response);

            } catch (const std::exception& e) {
                request_pair.second.set_exception(std::current_exception());
            }
        }
    } catch (const boost::system::system_error& e) {
        if (e.code() == boost::asio::error::operation_aborted) {
            LOGGING(debug) << "TCP process_requests: socket closed";
        } else {
            LOGGING(error) << "TCP process_requests: system error: " << e.what();
        }
    } catch (const std::exception& ex) {
        LOGGING(error) << "TCP process_requests: unexpected error: " << ex.what();
    } catch (...) {
        LOGGING(error) << "TCP process_requests: unknown exception";
    }
}

protocol_header_t RobotSocketBase::parse_header(const std::vector<uint8_t>& buffer) {
    if (buffer.size() < sizeof(protocol_header_t)) {
        throw std::runtime_error("Invalid message: too short");
    }
    protocol_header_t header;
    std::memcpy(&header, buffer.data(), sizeof(protocol_header_t));
    if (header.magic_number != PROTOCOL_MAGIC_NUMBER) {
        throw std::runtime_error("Invalid message: wrong magic number");
    }
    if (header.version != PROTOCOL_VERSION) {
        throw std::runtime_error(std::format(
            "protocol version mismatch: client={} controller={} — update controller firmware", PROTOCOL_VERSION, header.version));
    }
    return header;
}

// UdpRobotSocket Implementation
UdpRobotSocket::UdpRobotSocket(boost::asio::io_context& io_context, std::shared_ptr<State> state)
    : RobotSocketBase(io_context, "127.0.0.1", 0), robot_state_(std::move(state)) {}

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
        session_ = std::make_shared<Session>(io_context_, robot_state_);
        session_->socket_.open(ip::udp::v4());
        session_->socket_.bind(ip::udp::endpoint(ip::udp::v4(), 0));

        connected_ = true;
        co_spawn(io_context_, receive_messages(session_), detached);

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
        robot_state_->in_error.store(true);
        if (session_) {
            const std::lock_guard lock(session_->status_mutex_);
            auto err = std::make_exception_ptr(std::runtime_error("UDP socket disconnected"));
            for (auto& slot : session_->group_status_) {
                if (auto* sp = std::get_if<SafePromise>(&slot)) {
                    sp->set_exception(err);
                }
                slot = std::monostate{};
            }
            for (auto& slot : session_->group_robot_status_) {
                if (auto* sp = std::get_if<SafePromise>(&slot)) {
                    sp->set_exception(err);
                }
                slot = std::monostate{};
            }
        }
        if (session_ && session_->socket_.is_open()) {
            session_->socket_.close();
        }
    }
}

std::future<Message> UdpRobotSocket::request_from_cache_(std::array<CacheSlot, MAX_GROUPS>& cache, uint8_t group_index) {
    if (!connected_) {
        throw std::runtime_error("socket is disconnected");
    }
    if (group_index >= MAX_GROUPS) {
        throw std::runtime_error("group_index out of range");
    }
    const std::lock_guard lock(session_->status_mutex_);
    auto& slot = cache[group_index];
    return std::visit(
        cacheVisitor{
            [&](std::monostate&) -> std::future<Message> {
                SafePromise sp;
                auto future = sp.get_future();
                slot = std::move(sp);
                return future;
            },
            [&](Message& msg) -> std::future<Message> {
                std::promise<Message> p;
                p.set_value(msg);
                return p.get_future();
            },
            // SafePromise branch is only reachable before the first UDP packet arrives.
            // After that, the slot permanently holds Message (peek semantics).
            [&](SafePromise&) -> std::future<Message> { throw std::runtime_error("another caller is already waiting on this cache slot"); },
        },
        slot);
}

void UdpRobotSocket::save_to_cache_(std::array<CacheSlot, MAX_GROUPS>& cache, uint8_t group_index, const Message& msg) {
    auto& slot = cache[group_index];
    std::visit(cacheVisitor{
                   [&](std::monostate&) { slot = msg; },
                   [&](Message&) { slot = msg; },
                   [&](SafePromise& sp) {
                       sp.set_value(msg);
                       slot = msg;
                   },
               },
               slot);
}

std::future<Message> UdpRobotSocket::get_group_status(uint8_t group_index) {
    return request_from_cache_(session_->group_status_, group_index);
}

std::future<Message> UdpRobotSocket::get_group_robot_status(uint8_t group_index) {
    return request_from_cache_(session_->group_robot_status_, group_index);
}

uint16_t UdpRobotSocket::get_local_port() const {
    if (session_ && session_->socket_.is_open()) {
        return session_->socket_.local_endpoint().port();
    }
    return 0;
}

awaitable<void> UdpRobotSocket::receive_messages(std::shared_ptr<Session> session) {
    try {
        while (true) {
            std::vector<uint8_t> buffer(8192);
            ip::udp::endpoint sender_endpoint;
            const size_t bytes_received =
                co_await session->socket_.async_receive_from(boost::asio::buffer(buffer), sender_endpoint, use_awaitable);

            buffer.resize(bytes_received);
            const Message message = parse_message(buffer);

            if (message.header.message_type == MSG_ROBOT_POSITION_VELOCITY_TORQUE) {
                uint8_t group_index = 0;
                if (!message.payload.empty()) {
                    group_index = message.payload[0];
                }
                if (group_index < MAX_GROUPS) {
                    const std::lock_guard lock(session->status_mutex_);
                    save_to_cache_(session->group_status_, group_index, message);
                }
            } else if (message.header.message_type == MSG_ROBOT_STATUS) {
                uint8_t group_index = 0;
                if (message.payload.size() > sizeof(int64_t)) {
                    group_index = message.payload[sizeof(int64_t)];
                }
                if (group_index < MAX_GROUPS) {
                    const std::lock_guard lock(session->status_mutex_);
                    save_to_cache_(session->group_robot_status_, group_index, message);
                }
                try {
                    session->robot_state_->UpdateState(RobotStatusMessage(message));
                } catch (const std::exception& e) {
                    LOGGING(error) << "UDP receive_messages: UpdateState failed: " << e.what();
                }
            }
        }
    } catch (const boost::system::system_error& e) {
        if (e.code() == boost::asio::error::operation_aborted) {
            LOGGING(debug) << "UDP receive_messages: socket closed";
        } else {
            session->robot_state_->in_error.store(true);
            LOGGING(error) << "UDP receive_messages: system error: " << e.what();
        }
    } catch (const std::exception& e) {
        session->robot_state_->in_error.store(true);
        LOGGING(error) << "UDP receive_messages: unexpected error: " << e.what();
    } catch (...) {
        session->robot_state_->in_error.store(true);
        LOGGING(error) << "UDP receive_messages: unknown exception";
    }
}

Message UdpRobotSocket::parse_message(const std::vector<uint8_t>& buffer) {
    if (buffer.size() < sizeof(protocol_header_t)) {
        throw std::runtime_error("Invalid message: too short");
    }

    Message message;
    std::memcpy(&message.header, buffer.data(), sizeof(protocol_header_t));

    if (message.header.magic_number != PROTOCOL_MAGIC_NUMBER) {
        throw std::runtime_error(std::format(
            "invalid message: wrong magic number expected: {:X} got: {:X}", PROTOCOL_MAGIC_NUMBER, (int)message.header.magic_number));
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
UdpBroadcastListener::UdpBroadcastListener(boost::asio::io_context& io_context, uint16_t port) : io_context_(io_context), port_(port) {}

UdpBroadcastListener::~UdpBroadcastListener() {
    stop();
}

void UdpBroadcastListener::start() {
    if (started_) {
        return;  // Already running
    }

    try {
        session_ = std::make_shared<Session>(io_context_);
        session_->socket_.open(udp::v4());
        session_->socket_.set_option(udp::socket::reuse_address(true));
        session_->socket_.bind(udp::endpoint(udp::v4(), port_));
        started_ = true;
        co_spawn(io_context_, receive_broadcasts(session_), detached);

        LOGGING(info) << "UDP broadcast listener started on port " << port_;
    } catch (const std::exception& e) {
        started_ = false;
        LOGGING(error) << "Failed to start UDP broadcast listener: " << e.what();
        throw;
    }
}

void UdpBroadcastListener::stop() {
    if (!started_) {
        return;  // Already stopped
    }
    started_ = false;
    try {
        if (session_ && session_->socket_.is_open()) {
            session_->socket_.close();
        }
        LOGGING(info) << "UDP broadcast listener stopped";
    } catch (const std::exception& e) {
        LOGGING(error) << "Error stopping UDP broadcast listener: " << e.what();
    }
}

boost::asio::awaitable<void> UdpBroadcastListener::receive_broadcasts(std::shared_ptr<Session> session) {
    using namespace boost::asio::experimental::awaitable_operators;
    try {
        while (true) {
            udp::endpoint sender_endpoint;
            auto timer = boost::asio::steady_timer(session->socket_.get_executor());
            timer.expires_after(std::chrono::milliseconds(200));
            // Receive broadcast message
            auto result =
                co_await(timer.async_wait(use_awaitable) ||
                         session->socket_.async_receive_from(boost::asio::buffer(session->recv_buffer_), sender_endpoint, use_awaitable));

            if (std::holds_alternative<std::size_t>(result)) {
                const size_t bytes_received = std::get<std::size_t>(result);
                if (bytes_received > 0) {
                    // Null-terminate the buffer to ensure safe string operations
                    const std::size_t str_pos = std::min(bytes_received, session->recv_buffer_.size() - 1);
                    session->recv_buffer_[str_pos] = '\0';

                    // Process the data through the log parser
                    if (session->log_parser_) {
                        session->log_parser_->process_data(session->recv_buffer_.data());
                    }
                } else {
                    session->log_parser_->flush();
                }
            } else {
                session->log_parser_->flush();
            }
        }
    } catch (const boost::system::system_error& e) {
        if (e.code() == boost::asio::error::operation_aborted) {
            LOGGING(debug) << "UDP broadcast listener: socket closed";
        } else {
            LOGGING(error) << "UDP broadcast listener: system error: " << e.what();
        }
    } catch (const std::exception& e) {
        LOGGING(error) << "UDP broadcast listener: unexpected error: " << e.what();
    } catch (...) {
        LOGGING(error) << "UDP broadcast listener: unknown exception";
    }
    if (session->log_parser_) {
        session->log_parser_->flush();
    }
}

// Robot Implementation
YaskawaController::YaskawaController(boost::asio::io_context& io_context, const viam::sdk::ResourceConfig& config)
    : io_context_(io_context), robot_state_(std::make_shared<State>()) {
    host_ = find_config_attribute<std::string>(config, "host").value();

    trajectory_sampling_freq_ =
        find_config_attribute<double>(config, "trajectory_sampling_freq_hz").value_or(k_default_trajectory_sampling_freq);

    auto waypoint_dedup_tolerance_deg = find_config_attribute<double>(config, "waypoint_deduplication_tolerance_deg");
    waypoint_dedup_tolerance_rad_ =
        waypoint_dedup_tolerance_deg ? degrees_to_radians(*waypoint_dedup_tolerance_deg) : k_default_waypoint_deduplication_tolerance_rads;

    use_new_trajectory_planner_ = find_config_attribute<bool>(config, "enable_new_trajectory_planner").value_or(true);
    path_tolerance_rad_ = find_config_attribute<double>(config, "path_tolerance_rad").value_or(0.1);
    collinearization_ratio_ = find_config_attribute<double>(config, "collinearization_ratio");
    segmentation_threshold_rad_ =
        find_config_attribute<double>(config, "segmentation_threshold_rad").value_or(k_default_segmentation_threshold);

    auto tcp_port = find_config_attribute<double>(config, "tcp_port");
    tcp_port_ = tcp_port ? static_cast<uint16_t>(*tcp_port) : static_cast<uint16_t>(TCP_PORT);
    tcp_socket_ = std::make_unique<TcpRobotSocket>(io_context_, host_, tcp_port_);
    broadcast_listener_ = std::make_unique<UdpBroadcastListener>(io_context_);
}

YaskawaController::~YaskawaController() {
    try {
        disconnect();
    } catch (...) {
        LOGGING(debug) << "caught unknown exception";
    }
}

namespace {
std::mutex s_registry_mutex;
std::map<std::string, std::weak_ptr<YaskawaController>> s_controller_registry;
}  // namespace

std::shared_ptr<YaskawaController> YaskawaController::get_or_create(boost::asio::io_context& io_context,
                                                                    const viam::sdk::ResourceConfig& config) {
    auto host_attr = find_config_attribute<std::string>(config, "host");
    if (!host_attr) {
        throw std::runtime_error("host attribute is required");
    }
    const auto& host = *host_attr;

    const std::lock_guard lock(s_registry_mutex);
    // The registry stores weak_ptr — if the controller is still alive (held by YaskawaArm
    // instances), lock() returns the existing shared_ptr. A new controller is only created
    // if all references were released (weak_ptr expired).
    auto it = s_controller_registry.find(host);
    if (it != s_controller_registry.end()) {
        if (auto existing = it->second.lock()) {
            return existing;
        }
        s_controller_registry.erase(it);
    }
    auto ctrl = std::make_shared<YaskawaController>(io_context, config);
    s_controller_registry[host] = ctrl;
    return ctrl;
}

void YaskawaController::set_trajectory_loggers(std::string robot_model,
                                               std::optional<std::function<std::optional<std::string>()>> telemetry_path_fn) {
    robot_model_ = std::move(robot_model);
    telemetry_path_fn_ = std::move(telemetry_path_fn);
}

void YaskawaController::establish_connections_() {
    try {
        auto tcp_future = tcp_socket_->connect();
        if (tcp_future.wait_for(k_socket_timeout) != std::future_status::ready) {
            throw std::runtime_error("TCP connect timed out");
        }
        tcp_future.get();

        // Capabilities handshake: verify protocol version and discover controller groups
        auto caps = get_capabilities();
        LOGGING(info) << "controller capabilities: protocol_version=" << static_cast<int>(caps.protocol_version)
                      << " num_groups=" << static_cast<int>(caps.num_groups);
        for (const auto& grp : caps.groups) {
            LOGGING(info) << "  group " << static_cast<int>(grp.group_id) << ": type=" << static_cast<int>(grp.group_type)
                          << " axes=" << static_cast<int>(grp.num_axes) << " interpolation_us=" << grp.interpolation_period_us;
        }

        udp_socket_ = std::make_unique<UdpRobotSocket>(io_context_, robot_state_);
        auto udp_future = udp_socket_->connect();
        if (udp_future.wait_for(k_socket_timeout) != std::future_status::ready) {
            throw std::runtime_error("UDP connect timed out");
        }
        udp_future.get();

        register_udp_port(udp_socket_->get_local_port());
        get_robot_status();
    } catch (...) {
        tcp_socket_->disconnect();
        if (udp_socket_) {
            udp_socket_->disconnect();
        }
        throw;
    }
}

std::future<void> YaskawaController::connect() {
    // If already connected (shared controller, another arm already called connect()),
    // return a resolved future immediately — don't reconnect.
    if (running_) {
        std::promise<void> p;
        p.set_value();
        return p.get_future();
    }
    return std::async(std::launch::async, [this]() {
        try {
            establish_connections_();

            running_ = true;

            // Heartbeat thread: sends periodic heartbeats and retries reconnection indefinitely
            // on failure. Exits when the controller is destroyed (weak_ptr expires) or
            // disconnect() is called (running_ set to false).
            heartbeat_thread_ = std::thread([self = weak_from_this()]() {
                constexpr auto k_heartbeat_interval = std::chrono::milliseconds(100);
                constexpr auto k_reconnect_delay = std::chrono::seconds(1);
                constexpr uint32_t k_log_every_n_failures = 30;
                uint32_t consecutive_failures = 0;

                while (auto shared = self.lock()) {
                    if (!shared->running_) {
                        return;
                    }
                    try {
                        shared->send_heartbeat();
                        consecutive_failures = 0;
                        shared.reset();
                        std::this_thread::sleep_for(k_heartbeat_interval);
                    } catch (const std::exception& e) {
                        ++consecutive_failures;
                        const bool should_log = consecutive_failures == 1 || consecutive_failures % k_log_every_n_failures == 0;
                        const auto log_level = should_log ? viam::yaskawa::LogLevel::WARNING : viam::yaskawa::LogLevel::DEBUG;
                        viam::yaskawa::get_global_logger()->log(log_level) << "heartbeat failed: " << e.what();
                        if (!shared->running_) {
                            return;
                        }
                        // reconnect_() replaces the sockets in-place. Any in-progress
                        // move will fail when it next tries to communicate with the
                        // controller (goal status / chunk sends), which is the expected
                        // behavior when the connection drops mid-move.
                        try {
                            shared->reconnect_();
                            LOGGING(info) << "reconnected successfully";
                            consecutive_failures = 0;
                            shared.reset();
                            std::this_thread::sleep_for(k_heartbeat_interval);
                        } catch (const std::exception& re) {
                            viam::yaskawa::get_global_logger()->log(log_level) << std::format("reconnect failed: {}", re.what());
                            shared.reset();
                            std::this_thread::sleep_for(k_reconnect_delay);
                        }
                    }
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
    LOGGING(info) << "Yaskawa Controller disconnecting";
    running_ = false;
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }
    if (udp_socket_) {
        udp_socket_->disconnect();
    }
    if (tcp_socket_) {
        tcp_socket_->disconnect();
    }
    if (broadcast_listener_) {
        broadcast_listener_->stop();
    }
}

void YaskawaController::reconnect_() {
    LOGGING(info) << "tearing down existing connections for reconnect";

    udp_socket_->disconnect();
    tcp_socket_->disconnect();
    // broadcast_listener_ is not restarted here: it is diagnostic-only and its UDP
    // socket is not affected by TCP/control-plane disconnects.

    // Wait for any in-progress moves to notice the disconnection and clear their flags.
    // The move threads poll at 100Hz so this normally resolves within milliseconds.
    // We proceed after the timeout regardless — the moves will fail on their next
    // communication attempt.
    constexpr auto k_move_drain_timeout = std::chrono::milliseconds(500);
    const auto deadline = std::chrono::steady_clock::now() + k_move_drain_timeout;
    auto any_group_moving = [this]() { return std::ranges::any_of(group_move_in_progress_, [](const auto& flag) { return flag.load(); }); };
    while (any_group_moving() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (any_group_moving()) {
        LOGGING(warning) << "reconnect: move still in progress on some groups after drain, proceeding anyway";
    }

    // Replace TCP socket without going through null: disconnect() above sets connected_=false
    // atomically, so concurrent callers get "Not connected" rather than a null deref.
    // establish_connections_() will create a fresh UDP socket.
    tcp_socket_ = std::make_unique<TcpRobotSocket>(io_context_, host_, tcp_port_);

    establish_connections_();
    LOGGING(info) << "reconnect complete";
}

double YaskawaController::get_waypoint_deduplication_tolerance_rad() const {
    return waypoint_dedup_tolerance_rad_;
}

GoalStatusMessage YaskawaController::get_goal_status(int32_t goal_id) {
    std::vector<uint8_t> payload(sizeof(cancel_goal_payload_t));
    cancel_goal_payload_t* req = reinterpret_cast<cancel_goal_payload_t*>(payload.data());
    req->goal_id = goal_id;

    auto msg = tcp_socket_->send_request(Message(MSG_GET_GOAL_STATUS, std::move(payload))).get();
    if (msg.header.message_type == MSG_ERROR) {
        throw std::runtime_error(std::format("received an error message while getting status for goal id {}", goal_id));
    }
    return GoalStatusMessage(msg);
}

void YaskawaController::cancel_goal(int32_t goal_id) {
    std::vector<uint8_t> payload(sizeof(cancel_goal_payload_t));
    cancel_goal_payload_t* req = reinterpret_cast<cancel_goal_payload_t*>(payload.data());
    req->goal_id = goal_id;
    auto msg = tcp_socket_->send_request(Message(MSG_CANCEL_GOAL, std::move(payload))).get();
    const auto err = msg.get_error(MSG_OK);
    if (!err.empty()) {
        throw std::runtime_error(std::format("an error occurred while cancelling goal id {}: {}", goal_id, err));
    }
    LOGGING(debug) << "MSG_CANCEL_GOAL: " << msg;
}

void YaskawaController::setMotionMode(uint8_t mode) {
    std::vector<uint8_t> payload(sizeof(motion_mode_payload_t));
    motion_mode_payload_t* req = reinterpret_cast<motion_mode_payload_t*>(payload.data());
    req->motion_mode = mode;

    auto msg = tcp_socket_->send_request(Message(MSG_SET_MOTION_MODE, std::move(payload))).get();
    const auto err = msg.get_error(MSG_OK);
    if (!err.empty()) {
        throw std::runtime_error(std::format("failed to set motion mode: {}", err));
    }
    LOGGING(debug) << "MSG_SET_MOTION_MODE: " << msg;
}

void YaskawaController::send_test_trajectory() {
    if (!robot_state_->IsReady()) {
        throw std::runtime_error(
            std::format("cannot send test trajectory the robot state is e_stopped {} in error "
                        "{}",
                        robot_state_->e_stopped.load(),
                        robot_state_->in_error.load()));
    }
    auto msg = tcp_socket_->send_request(Message(MSG_TEST_TRAJECTORY_COMMAND)).get();
    const auto err = msg.get_error(MSG_OK);
    if (!err.empty()) {
        throw std::runtime_error(std::format("failed to send MSG_TEST_TRAJECTORY_COMMAND: {}", err));
    }
    LOGGING(debug) << "MSG_TEST_TRAJECTORY_COMMAND: " << msg;
}

void YaskawaController::turn_servo_power_on() {
    if (!robot_state_->IsReady()) {
        throw std::runtime_error(std::format("cannot turn power on the robot state is e_stopped {} in error {}",
                                             robot_state_->e_stopped.load(),
                                             robot_state_->in_error.load()));
    }
    auto msg = tcp_socket_->send_request(Message(MSG_TURN_SERVO_POWER_ON)).get();
    const auto err = msg.get_error(MSG_OK);
    if (!err.empty()) {
        throw std::runtime_error(std::format("failed to turn on arm servo power: {}", err));
    }
    LOGGING(debug) << "MSG_TURN_SERVO_POWER_ON: " << msg;
}

void YaskawaController::send_heartbeat() {
    if (!tcp_socket_) {
        throw std::runtime_error("heartbeat failed: no TCP connection");
    }
    auto future = tcp_socket_->send_request(Message(MSG_HEARTBEAT));
    if (future.wait_for(k_socket_timeout) != std::future_status::ready) {
        throw std::runtime_error("heartbeat timed out");
    }
    auto msg = future.get();
    const auto err = msg.get_error(MSG_OK);
    if (!err.empty()) {
        throw std::runtime_error(std::format("failed to send heartbeat: {}", err));
    }
    LOGGING(debug) << "MSG_HEARTBEAT: " << msg;
}

void YaskawaController::send_test_error_command() {
    auto msg = tcp_socket_->send_request(Message(MSG_TEST_ERROR_COMMAND)).get();
    const auto err = msg.get_error(MSG_OK);
    if (!err.empty()) {
        throw std::runtime_error(std::format("failed to send MSG_TEST_ERROR_COMMAND: {}", err));
    }
    LOGGING(debug) << "MSG_TEST_ERROR_COMMAND: " << msg;
}

void YaskawaController::get_error_info() {
    // currently unimplemented
    auto msg = tcp_socket_->send_request(Message(MSG_GET_ERROR_INFO)).get();
    const auto err = msg.get_error(MSG_OK);
    if (!err.empty()) {
        throw std::runtime_error(std::format("message {} failed: {}", static_cast<const int&>(MSG_GET_ERROR_INFO), err));
    }
    LOGGING(debug) << "MSG_GET_ERROR_INFO: " << msg;
}

StatusMessage YaskawaController::get_group_position_velocity_torque(uint8_t group_index) {
    if (!udp_socket_) {
        throw std::runtime_error("UDP socket not connected");
    }
    return StatusMessage(udp_socket_->get_group_status(group_index).get());
}

RobotStatusMessage YaskawaController::get_robot_status() {
    return get_group_robot_status(0);
}

RobotStatusMessage YaskawaController::get_group_robot_status(uint8_t group_index) {
    if (!udp_socket_) {
        throw std::runtime_error("UDP socket not connected");
    }
    return RobotStatusMessage(udp_socket_->get_group_robot_status(group_index).get());
}

void YaskawaController::register_udp_port(uint16_t port) {
    std::vector<uint8_t> payload(sizeof(udp_port_registration_v2_payload_t));
    auto* port_payload = reinterpret_cast<udp_port_registration_v2_payload_t*>(payload.data());
    port_payload->udp_port = port;
    port_payload->protocol_version = PROTOCOL_VERSION;

    auto msg = tcp_socket_->send_request(Message(MSG_REGISTER_UDP_PORT, std::move(payload))).get();
    const auto err = msg.get_error(MSG_OK);
    if (!err.empty()) {
        throw std::runtime_error(std::format("message {} failed: {}", static_cast<const int&>(MSG_REGISTER_UDP_PORT), err));
    }
    LOGGING(info) << "UDP port registration response: " << msg;
}

void YaskawaController::reset_errors() {
    auto msg = tcp_socket_->send_request(Message(MSG_RESET_ERRORS)).get();
    const auto err = msg.get_error(MSG_OK);
    if (!err.empty()) {
        throw std::runtime_error(std::format("message {} failed: {}", static_cast<const int&>(MSG_RESET_ERRORS), err));
    }
    LOGGING(debug) << "MSG_RESET_ERRORS: " << msg;
}

GoalAcceptedMessage YaskawaController::send_goal_(uint32_t group_index,
                                                  uint32_t axes_controlled,
                                                  const std::vector<trajectory_point_t>& trajectory,
                                                  const std::vector<tolerance_t>& tolerance) {
    std::vector<uint8_t> payload;
    payload.reserve((sizeof(uint32_t) * 4) + (trajectory.size() * sizeof(trajectory_point_t)) + (tolerance.size() * sizeof(tolerance_t)));

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

std::unique_ptr<GoalRequestHandle> YaskawaController::move(std::list<Eigen::VectorXd> waypoints,
                                                           uint32_t group_index,
                                                           const std::string& unix_time,
                                                           const Eigen::VectorXd& velocity_limits,
                                                           const Eigen::VectorXd& acceleration_limits) {
    if (group_index >= MAX_GROUPS) {
        throw std::runtime_error(std::format("group_index {} exceeds MAX_GROUPS ({})", group_index, MAX_GROUPS));
    }

    // Per-group lock: only one move per group at a time, different groups can move in parallel.
    bool expected = false;
    if (!group_move_in_progress_[group_index].compare_exchange_strong(expected, true)) {
        throw std::runtime_error(std::format("an actuation is already in progress on group {}", group_index));
    }

    // Scope guard clears the per-group lock on any exit path.
    // Dismissed after the monitoring thread is successfully created (which takes over cleanup responsibility).
    ScopeGuard cleanup{[this, group_index]() { group_move_in_progress_[group_index] = false; }};

    // Check if any other group is currently moving. Both reset_errors() and setMotionMode()
    // are global operations that would interfere with in-progress motion on other groups.
    bool any_other_group_moving = false;
    for (uint32_t i = 0; i < MAX_GROUPS; ++i) {
        if (i != group_index && group_move_in_progress_[i].load()) {
            any_other_group_moving = true;
            break;
        }
    }
    if (!any_other_group_moving) {
        if (!robot_state_->IsReady()) {
            reset_errors();
        }
        setMotionMode(1);
    }
    turn_servo_power_on();

    auto promise = std::promise<goal_state_t>();

    std::optional<RealtimeTrajectoryLogger> logger;
    if (telemetry_path_fn_) {
        auto telemetry_path = (*telemetry_path_fn_)();
        if (telemetry_path) {
            try {
                logger.emplace(*telemetry_path, unix_time, robot_model_, group_index);
            } catch (const std::exception& e) {
                LOGGING(warning) << "Failed to create realtime trajectory logger: " << e.what();
            }
        }
    }

    auto goal_result = make_goal_(std::move(waypoints), group_index, unix_time, velocity_limits, acceleration_limits, logger);
    // we only want to move if the future was valid
    if (!goal_result) {
        LOGGING(debug) << "already at desired position";
        // cleanup guard will clear per-group lock on return
        promise.set_value(GOAL_STATE_SUCCEEDED);
        return std::make_unique<GoalRequestHandle>(0, shared_from_this(), promise.get_future());
    }

    const auto& accepted = goal_result->accepted;
    LOGGING(debug) << "goal accepted: goal_id=" << accepted.goal_id;
    if (logger.has_value()) {
        logger->set_goal_accepted_timestamp(accepted.timestamp_ms);
    }

    auto handle = std::make_unique<GoalRequestHandle>(accepted.goal_id, shared_from_this(), promise.get_future());

    // Derive poll interval from trajectory sampling frequency
    // we want to log data at 100 Hz and send chunks at trajectory_sampling_freq_ Hz
    constexpr auto k_logging_freq = 250;
    // Single thread handles both chunk streaming and goal monitoring
    std::thread([promise = std::move(promise),
                 self = weak_from_this(),
                 goal_id = accepted.goal_id,
                 group_index,
                 remaining = std::move(goal_result->remaining_trajectory),
                 goal_status_polling_trigger = (k_logging_freq / static_cast<uint64_t>(trajectory_sampling_freq_)),
                 axes_controlled = static_cast<uint32_t>(velocity_limits.size()),
                 logger = std::move(logger)]() mutable {
        // Scope guard clears per-group lock when thread exits (success or failure)
        const ScopeGuard thread_cleanup{[&self, group_index]() {
            if (auto shared = self.lock()) {
                shared->group_move_in_progress_[group_index] = false;
            }
        }};
        try {
            const auto poll_interval = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / k_logging_freq));
            constexpr size_t queue_threshold = 50;
            size_t offset = 0;
            uint64_t iteration = 0;
            while (true) {
                std::this_thread::sleep_for(poll_interval);
                auto shared = self.lock();
                if (!shared) {
                    LOGGING(error) << "cancelling goal monitor thread : the socket was destroyed!";
                    return;
                }

                // capture robot status at k_logging_freq Hz
                if (logger.has_value()) {
                    try {
                        logger->append_realtime_sample(shared->get_group_position_velocity_torque(static_cast<uint8_t>(group_index)));
                    } catch (const std::exception& e) {
                        LOGGING(warning) << "Failed to log realtime sample: " << e.what();
                    }
                }
                // Check the goal status @goal_status_polling_trigger Hz
                // in most cases this will not cleanly align with the 100 Hz polling frequency,
                // but int math should give us a result thats close enough.
                // TODO : change that with async
                if (iteration++ % goal_status_polling_trigger == 0) {
                    const auto status_msg = shared->get_goal_status(goal_id);

                    switch (status_msg.state) {
                        case GOAL_STATE_ACTIVE:
                            // Stream remaining chunks when queue is running low
                            if (offset < remaining.size()) {
                                LOGGING(debug) << "queue size: " << status_msg.current_queue_size << " points";
                                if (status_msg.current_queue_size <= queue_threshold) {
                                    const size_t end = std::min(offset + k_chunk_size, remaining.size());
                                    const std::vector<trajectory_point_t> chunk(
                                        std::next(remaining.begin(), static_cast<ptrdiff_t>(offset)),
                                        std::next(remaining.begin(), static_cast<ptrdiff_t>(end)));

                                    LOGGING(debug)
                                        << "sending chunk: points " << offset << " to " << end << " (" << chunk.size() << " points)";

                                    // tolerance can be threaded through here when needed
                                    const auto chunk_accepted = shared->send_goal_(group_index, axes_controlled, chunk, {});
                                    LOGGING(debug) << "chunk accepted: " << chunk_accepted.num_trajectory_accepted << " points";
                                    offset += chunk_accepted.num_trajectory_accepted;
                                }
                            }
                            break;
                        case GOAL_STATE_SUCCEEDED:
                            // if we still have data, stop the arm and throw an error.
                            if (offset < remaining.size()) {
                                std::string stop_detail;
                                try {
                                    if (!shared->stop(group_index)) {
                                        LOGGING(warning) << "stop returned false while handling early goal completion";
                                    }
                                } catch (const std::exception& e) {
                                    stop_detail = std::format(", stop failed: {}", e.what());
                                }
                                throw std::runtime_error(std::format(
                                    "goal failed - arm motion ended earlier than expected with {} trajectory points left to process{}",
                                    remaining.size() - offset,
                                    stop_detail));
                            }
                            if (shared->get_robot_status().in_motion) {
                                break;
                            }
                            promise.set_value_at_thread_exit(status_msg.state);
                            return;
                        case GOAL_STATE_PENDING:
                            break;
                        case GOAL_STATE_CANCELLED:
                            promise.set_value_at_thread_exit(GOAL_STATE_CANCELLED);
                            return;
                        case GOAL_STATE_ABORTED:
                            throw std::runtime_error(std::format("goal failed - goal status is {}", static_cast<int>(status_msg.state)));
                    }
                }
            }
        } catch (std::exception& e) {
            try {
                promise.set_exception_at_thread_exit(std::current_exception());
            } catch (...) {
                LOGGING(debug) << "caught unknown exception";
            }
        }
    }).detach();

    // Thread was successfully created and detached — thread_cleanup now owns the
    // responsibility of clearing group_move_in_progress_[group_index], so dismiss the outer guard.
    cleanup.dismiss();
    return handle;
}

std::optional<MakeGoalResult> YaskawaController::make_goal_(std::list<Eigen::VectorXd> waypoints,
                                                            uint32_t group_index,
                                                            const std::string& unix_time,
                                                            const Eigen::VectorXd& max_velocity_vec,
                                                            const Eigen::VectorXd& max_acceleration_vec,
                                                            std::optional<RealtimeTrajectoryLogger>& logger) {
    LOGGING(info) << "move: start unix_time_ms " << unix_time << " waypoints size " << waypoints.size();

    const std::list<Eigen::VectorXd> original_waypoints = waypoints;

    auto log_failure = [&](const std::string& replay_json) {
        if (telemetry_path_fn_) {
            auto telemetry_path = (*telemetry_path_fn_)();
            if (telemetry_path) {
                auto filename = std::format("{}/{}_failed_trajectory.trajex-totg-replay.json", *telemetry_path, unix_time);
                std::ofstream json_file(filename);
                json_file << replay_json;
            }
        }
    };

    using namespace viam::trajex;

    totg::planner_base::config planner_cfg;
    planner_cfg.velocity_limits = xt::xarray<double>::from_shape({static_cast<size_t>(max_velocity_vec.size())});
    planner_cfg.acceleration_limits = xt::xarray<double>::from_shape({static_cast<size_t>(max_acceleration_vec.size())});
    std::ranges::copy(max_velocity_vec, planner_cfg.velocity_limits.begin());
    std::ranges::copy(max_acceleration_vec, planner_cfg.acceleration_limits.begin());
    planner_cfg.path_blend_tolerance = path_tolerance_rad_;
    planner_cfg.colinearization_ratio = collinearization_ratio_;
    planner_cfg.segment_totg = false;

    auto planner =
        totg::planner<segment_accumulator>(planner_cfg)
            .with_waypoint_provider([&](auto& p) -> totg::waypoint_accumulator {
                auto curr_joint_pos = get_group_position_velocity_torque(static_cast<uint8_t>(group_index)).position;
                auto current_pos = p.stash(eigen_waypoints_to_xarray(
                    {Eigen::VectorXd::Map(curr_joint_pos.data(), boost::numeric_cast<Eigen::Index>(curr_joint_pos.size()))}));
                auto goal_waypoints = p.stash(eigen_waypoints_to_xarray(waypoints));

                totg::waypoint_accumulator acc(*current_pos);
                acc.add_waypoints(*goal_waypoints);
                return acc;
            })
            .with_waypoint_preprocessor([&](auto&, auto& acc) { acc = totg::deduplicate_waypoints(acc, waypoint_dedup_tolerance_rad_); })
            .with_segmenter(  // NOLINTNEXTLINE(performance-unnecessary-value-param)
                [&](auto&, totg::waypoint_accumulator acc) {
                    return totg::segment_at_reversals(std::move(acc), segmentation_threshold_rad_);
                });

    if (use_new_trajectory_planner_) {
        planner.with_totg(
            [&](const auto&,
                segment_accumulator& acc,
                const totg::waypoint_accumulator& seg,
                totg::trajectory&& traj,
                std::chrono::microseconds elapsed) {
                acc.total_waypoints += seg.size();
                acc.total_duration += traj.duration().count();
                acc.total_arc_length += static_cast<double>(traj.path().length());
                acc.total_generation_time += std::chrono::duration<double>(elapsed).count();
                ++acc.segment_count;

                if (acc.total_duration > 600) {
                    throw std::runtime_error("total trajectory duration exceeds maximum allowed duration");
                }

                auto sampler = totg::uniform_sampler::quantized_for_trajectory(traj, types::hertz{trajectory_sampling_freq_});

                // The first sample of each segment duplicates the last sample of the previous segment,
                // so we drop it to avoid duplicate trajectory points. However, the FIRST segment's
                // first sample is the robot's current position — the controller validates that the
                // trajectory starts at the commanded position (within START_MAX_PULSE_DEVIATION pulses),
                // so we must keep it.
                const bool is_first_segment = (acc.segment_count == 1);
                for (const auto& sample : traj.samples(sampler) | std::views::drop(is_first_segment ? 0 : 1)) {
                    const auto absolute_time = acc.cumulative_time + std::chrono::duration<double>(sample.time.count());
                    auto secs = std::chrono::floor<std::chrono::seconds>(absolute_time);
                    auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(absolute_time - secs);
                    acc.samples.push_back(
                        make_trajectory_point(sample.configuration,
                                              sample.velocity,
                                              {static_cast<uint32_t>(secs.count()), static_cast<uint32_t>(nanos.count())}));
                }

                acc.cumulative_time += std::chrono::duration<double>(traj.duration());

                LOGGING(info) << "trajex/totg segment generated, waypoints: " << seg.size() << ", duration: " << traj.duration().count()
                              << "s, samples: " << acc.samples.size() << ", arc length: " << traj.path().length();
            },
            [&](const auto& p, const segment_accumulator&, const totg::waypoint_accumulator& seg, const std::exception& e) {
                log_failure(p.serialize_for_replay(seg, e.what()));
            });
    }

    planner.with_legacy(
        [&](const auto&,
            segment_accumulator& acc,
            const totg::waypoint_accumulator& seg,
            Path&&,
            Trajectory&& traj,
            std::chrono::microseconds elapsed) {
            const double duration = traj.getDuration();

            if (!std::isfinite(duration)) {
                throw std::runtime_error("trajectory.getDuration() was not a finite number");
            }
            if (duration > 600) {
                throw std::runtime_error("trajectory.getDuration() exceeds 10 minutes");
            }
            if (duration < k_default_min_timestep_sec) {
                LOGGING(debug) << "duration of move is too small, assuming arm is at goal";
                return;
            }

            if (acc.samples.empty()) {
                acc.samples.push_back(make_trajectory_point(traj.getPosition(0.0), traj.getVelocity(0.0), {0, 0}));
            }

            sampling_func(acc.samples, duration, trajectory_sampling_freq_, [&](const double t, const double) {
                const auto absolute_time = acc.cumulative_time + std::chrono::duration<double>(t);
                auto secs = std::chrono::floor<std::chrono::seconds>(absolute_time);
                auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(absolute_time - secs);
                return make_trajectory_point(
                    traj.getPosition(t), traj.getVelocity(t), {static_cast<uint32_t>(secs.count()), static_cast<uint32_t>(nanos.count())});
            });

            acc.cumulative_time += std::chrono::duration<double>(duration);
            acc.total_waypoints += seg.size();
            acc.total_duration += duration;
            acc.total_generation_time += std::chrono::duration<double>(elapsed).count();
            ++acc.segment_count;
        },
        [&](const auto& p, const segment_accumulator&, const totg::waypoint_accumulator& seg, const std::exception& e) {
            log_failure(p.serialize_for_replay(seg, e.what()));
        });

    auto result = planner.execute([&](const auto& p, auto trajex_out, auto legacy_out) -> std::optional<segment_accumulator> {
        if (trajex_out.receiver) {
            auto& acc = *trajex_out.receiver;
            LOGGING(info) << "trajex/totg trajectory generated, total waypoints: " << acc.total_waypoints
                          << ", total duration: " << acc.total_duration << "s, total samples: " << acc.samples.size()
                          << ", total arc length: " << acc.total_arc_length << ", generation_time: " << acc.total_generation_time << "s";
            return std::move(trajex_out.receiver);
        }

        if (legacy_out.receiver) {
            LOGGING(info) << "trajectory generation uses legacy generator";
            return std::move(legacy_out.receiver);
        }

        if (legacy_out.error) {
            std::rethrow_exception(legacy_out.error);
        }
        if (trajex_out.error) {
            std::rethrow_exception(trajex_out.error);
        }

        if (p.processed_waypoint_count() < 2) {
            return std::nullopt;
        }
        throw std::runtime_error("both trajectory generators failed");
    });

    if (!result || result->samples.empty()) {
        return std::nullopt;
    }

    auto& samples = result->samples;

    // Populate logger with trajectory data
    if (logger.has_value()) {
        logger->set_max_velocity(max_velocity_vec);
        logger->set_max_acceleration(max_acceleration_vec);
        logger->set_waypoints(original_waypoints);
        logger->set_planned_trajectory(samples, static_cast<int>(max_velocity_vec.size()));
    }

    LOGGING(debug) << "total trajectory points: " << samples.size();

    // Send the first chunk
    const auto first_end = std::min(k_chunk_size, samples.size());
    const std::vector<trajectory_point_t> first_chunk(samples.begin(), std::next(samples.begin(), static_cast<ptrdiff_t>(first_end)));
    // tolerance can be threaded through here when needed
    auto accepted = send_goal_(group_index, static_cast<uint32_t>(max_velocity_vec.size()), first_chunk, {});

    // Return accepted message + any remaining trajectory points.
    // If the controller reports 0 or a value exceeding what we sent, assume the entire chunk was consumed.
    // This handles firmware that doesn't set num_trajectory_accepted (field may be uninitialized).
    const size_t raw = accepted.num_trajectory_accepted;
    size_t accepted_count = raw;
    if (raw == 0 || raw > first_end) {
        LOGGING(warning) << "controller reported " << raw << " points accepted (sent " << first_end << "), assuming all accepted";
        accepted_count = first_end;
    }
    std::vector<trajectory_point_t> remaining;
    if (accepted_count < samples.size()) {
        remaining.assign(std::next(samples.begin(), static_cast<ptrdiff_t>(accepted_count)), samples.end());
    }

    return MakeGoalResult{std::move(accepted), std::move(remaining)};
}

std::future<Message> YaskawaController::echo_trajectory() {
    // Echo trajectory command has no payload
    return tcp_socket_->send_request(Message(MSG_ECHO_TRAJECTORY));
}
bool YaskawaController::stop(uint32_t group_index) {
    std::vector<uint8_t> payload(sizeof(group_id_t));
    group_id_t* id = reinterpret_cast<group_id_t*>(payload.data());
    id->group_id = static_cast<int32_t>(group_index);
    auto msg = tcp_socket_->send_request(Message(MSG_STOP_MOTION, std::move(payload))).get();
    const auto err = msg.get_error(MSG_STOP_MOTION);
    if (!err.empty()) {
        throw std::runtime_error(std::format("failed to stop arm motion: {}", err));
    }
    LOGGING(debug) << "MSG_STOP_MOTION: " << msg;
    // Validate payload size to prevent buffer overruns
    if (msg.payload.size() != sizeof(boolean_payload_t)) {
        throw std::runtime_error(std::format(
            "incorrect MSG_STOP_MOTION payload size: expected {} bytes, got {} bytes", sizeof(boolean_payload_t), msg.payload.size()));
    }

    // Safe deserialization: verify alignment before reinterpret_cast
    if (reinterpret_cast<uintptr_t>(msg.payload.data()) % alignof(boolean_payload_t) != 0) {
        throw std::runtime_error("boolean payload data is not properly aligned");
    }

    const boolean_payload_t* is_stopped = reinterpret_cast<const boolean_payload_t*>(msg.payload.data());
    return is_stopped->value;
}
CartesianPosition YaskawaController::getCartPosition(uint32_t group_index) {
    std::vector<uint8_t> payload(sizeof(group_id_t));
    group_id_t* id = reinterpret_cast<group_id_t*>(payload.data());
    id->group_id = static_cast<int32_t>(group_index);
    return CartesianPosition(tcp_socket_->send_request(Message(MSG_GET_CART, std::move(payload))).get());
}
AnglePosition YaskawaController::cartPosToAngle(uint32_t group_index, CartesianPosition& pos) {
    std::vector<uint8_t> payload(sizeof(cartesian_payload_t));
    cartesian_payload_t* cid = reinterpret_cast<cartesian_payload_t*>(payload.data());
    cid->group_id = static_cast<int32_t>(group_index);
    cid->cartesianCoord[0] = pos.x;
    cid->cartesianCoord[1] = pos.y;
    cid->cartesianCoord[2] = pos.z;
    cid->cartesianCoord[3] = pos.rx;
    cid->cartesianCoord[4] = pos.ry;
    cid->cartesianCoord[5] = pos.rz;
    return AnglePosition(tcp_socket_->send_request(Message(MSG_FROM_CART_TO_JOINT, std::move(payload))).get());
}
CartesianPosition YaskawaController::angleToCartPos(uint32_t group_index, AnglePosition& pos) {
    std::vector<uint8_t> payload(sizeof(position_angle_degree_payload_t));
    position_angle_degree_payload_t* pid = reinterpret_cast<position_angle_degree_payload_t*>(payload.data());
    pid->group_id = static_cast<int32_t>(group_index);
    pid->positionAngleDegree[0] = pos.pos[0];
    pid->positionAngleDegree[1] = pos.pos[1];
    pid->positionAngleDegree[2] = pos.pos[2];
    pid->positionAngleDegree[3] = pos.pos[3];
    pid->positionAngleDegree[4] = pos.pos[4];
    pid->positionAngleDegree[5] = pos.pos[5];
    return CartesianPosition(tcp_socket_->send_request(Message(MSG_FROM_JOINT_TO_CART, std::move(payload))).get());
}

bool YaskawaController::is_status_command(message_type_t type) {
    return type == MSG_ROBOT_POSITION_VELOCITY_TORQUE || type == MSG_ROBOT_STATUS;
}

bool YaskawaController::checkGroupIndex(uint32_t group_index) {
    std::vector<uint8_t> payload(sizeof(group_id_t));
    group_id_t* id = reinterpret_cast<group_id_t*>(payload.data());
    id->group_id = static_cast<int32_t>(group_index);
    return CheckGroupMessage(tcp_socket_->send_request(Message(MSG_CHECK_GROUP, std::move(payload))).get()).is_known_group;
}

CapabilitiesMessage YaskawaController::get_capabilities() {
    auto msg = tcp_socket_->send_request(Message(MSG_GET_CAPABILITIES)).get();
    const auto err = msg.get_error(MSG_CAPABILITIES);
    if (!err.empty()) {
        throw std::runtime_error(std::format("MSG_GET_CAPABILITIES failed: {}", err));
    }
    return CapabilitiesMessage(msg);
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
        auto detail = msg.get_error(MSG_GOAL_ACCEPTED);
        throw std::runtime_error(
            detail.empty()
                ? std::format("wrong message type expected {} had {}", static_cast<int>(MSG_GOAL_ACCEPTED), msg.header.message_type)
                : detail);
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

        return shared->get_goal_status(goal_id_);
    });
}

void GoalRequestHandle::cancel() {
    auto shared = controller_.lock();
    if (!shared) {
        return;
    }
    try {
        shared->cancel_goal(goal_id_);
    } catch (const std::exception& e) {
        // Cancel may fail if the goal already completed or the connection dropped.
        // This is expected when cancel races with goal completion.
        LOGGING(warning) << "cancel_goal(" << goal_id_ << ") failed: " << e.what();
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
