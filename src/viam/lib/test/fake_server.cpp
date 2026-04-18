#include "fake_server.hpp"

#include <chrono>
#include <stdexcept>
#include <vector>

extern "C" {
#include "logging.h"
}

namespace test {

static std::atomic<uint16_t> s_next_port{30000};

ServerPorts FakeServer::allocate_ports() {
    auto base = s_next_port.fetch_add(2);
    return {base, static_cast<uint16_t>(base + 1)};
}

command_response_context_t* FakeServer::fault_injecting_handle_command(protocol_header_t* header, void* payload, void* user_data) {
    auto* data = static_cast<FaultCallbackData*>(user_data);

    fault_type_t fault = fault_inject_check(data->fault_ctx, header);
    if (fault != FAULT_NONE) {
        fault_inject_execute(data->fault_ctx, fault);
        if (fault == FAULT_UNRESPONSIVE) {
            return MSG_OK_RESPONSE();
        }
        return nullptr;
    }

    return mock_robot_handle_command(header, payload, data->robot);
}

FakeServer::FakeServer(ServerPorts ports, uint32_t connection_timeout_ms)
    : ports_(ports), groups_(std::begin(k_single_robot), std::end(k_single_robot)) {
    log_to_stdout();
    mock_robot_init(&robot_);
    init_server(connection_timeout_ms);
}

FakeServer::FakeServer(ServerPorts ports, std::span<const GroupConfig> groups, uint32_t connection_timeout_ms)
    : ports_(ports), groups_(groups.begin(), groups.end()) {
    log_to_stdout();
    std::vector<uint8_t> types(groups.size());
    std::vector<uint8_t> axes(groups.size());
    for (size_t i = 0; i < groups.size(); ++i) {
        types[i] = groups[i].group_type;
        axes[i] = groups[i].num_axes;
    }
    mock_robot_init_multi(&robot_, static_cast<uint8_t>(groups.size()), types.data(), axes.data());
    init_server(connection_timeout_ms);
}

void FakeServer::init_server(uint32_t connection_timeout_ms) {
    robot_server_config_t config{};
    config.tcp_port = ports_.tcp_port;
    config.udp_port = ports_.udp_port;
    config.connection_timeout_ms = connection_timeout_ms;
    config.callbacks.on_connection = mock_robot_on_connection;
    config.callbacks.on_disconnection = mock_robot_on_disconnection;
    config.callbacks.handle_command = fault_injecting_handle_command;
    config.callbacks.get_error_info = mock_robot_get_error_info_callback;

    cb_data_.fault_ctx = &fault_ctx_;
    cb_data_.robot = &robot_;
    config.user_data = &cb_data_;

    // Wrap connection callbacks to extract robot from cb_data_
    config.callbacks.on_connection = [](const char* client_ip, uint16_t client_port, void* user_data) {
        auto* data = static_cast<FaultCallbackData*>(user_data);
        mock_robot_on_connection(client_ip, client_port, data->robot);
    };
    config.callbacks.on_disconnection = [](const char* client_ip, uint16_t client_port, void* user_data) {
        auto* data = static_cast<FaultCallbackData*>(user_data);
        mock_robot_on_disconnection(client_ip, client_port, data->robot);
    };
    config.callbacks.get_error_info = [](int32_t* error_code, void* user_data) -> command_response_context_t* {
        auto* data = static_cast<FaultCallbackData*>(user_data);
        return mock_robot_get_error_info_callback(error_code, data->robot);
    };

    server_ctx_ = robot_protocol_create(&config);
    if (!server_ctx_) {
        throw std::runtime_error("robot_protocol_create failed");
    }

    fault_inject_init(&fault_ctx_, server_ctx_);

    if (robot_protocol_start(server_ctx_) != 0) {
        robot_protocol_destroy(server_ctx_);
        throw std::runtime_error("robot_protocol_start failed");
    }

    // Give the server threads time to bind and listen
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

FakeServer::~FakeServer() {
    stop_udp_status_pump();
    robot_protocol_stop(server_ctx_);
    robot_protocol_destroy(server_ctx_);
}

mock_robot_t& FakeServer::robot() {
    return robot_;
}

robot_server_ctx_t* FakeServer::ctx() {
    return server_ctx_;
}

ServerPorts FakeServer::ports() const {
    return ports_;
}

uint8_t FakeServer::num_groups() const {
    return static_cast<uint8_t>(groups_.size());
}

const GroupConfig& FakeServer::group_config(uint8_t index) const {
    return groups_.at(index);
}

void FakeServer::inject_disconnect_after(uint32_t n) {
    fault_inject_disconnect_after(&fault_ctx_, n);
}

void FakeServer::inject_unresponsive_on(uint8_t msg_type) {
    fault_inject_unresponsive_on(&fault_ctx_, msg_type);
}

void FakeServer::inject_close_connection() {
    fault_inject_close_connection(&fault_ctx_);
}

void FakeServer::inject_rule(const fault_rule_t& rule) {
    fault_inject_add_rule(&fault_ctx_, &rule);
}

void FakeServer::reset_faults() {
    fault_inject_reset(&fault_ctx_);
}

fault_inject_ctx_t& FakeServer::faults() {
    return fault_ctx_;
}

void FakeServer::start_udp_status_pump(uint32_t interval_ms) {
    if (pumping_.exchange(true)) {
        return;  // already pumping
    }

    pump_thread_ = std::thread([this, interval_ms]() {
        while (pumping_.load()) {
            auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

            for (uint8_t g = 0; g < robot_.num_groups; ++g) {
                status_payload_t status{};
                mock_robot_get_status(&robot_, &status, now, g);
                robot_protocol_send_position_velocity_torque(server_ctx_, &status);

                robot_status_payload_t robot_status{};
                mock_robot_get_robot_status(&robot_, &robot_status, now, g);
                robot_protocol_send_robot_status(server_ctx_, &robot_status);
            }

            mock_robot_update_goal_progress(&robot_);

            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        }
    });
}

void FakeServer::stop_udp_status_pump() {
    if (pumping_.exchange(false) && pump_thread_.joinable()) {
        pump_thread_.join();
    }
}

}  // namespace test
