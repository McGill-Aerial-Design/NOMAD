// SPDX-License-Identifier: Apache-2.0
// Command-side methods of UdpMavlinkConnection: velocity setpoints,
// data-stream requests, and the command/acknowledgement exchange. Split from
// udp_connection.cpp so each file stays under the 500-line policy; these
// methods run on the same receive mutex as the telemetry path, which lives in
// udp_connection.cpp.
#include "nomad/mavlink/udp_connection.hpp"

#include "nomad/mavlink/protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#else
#include <sys/socket.h>
#endif

namespace nomad::mavlink {
namespace {

// Matches the GCS identity used by the telemetry path in udp_connection.cpp.
constexpr std::uint8_t kSourceSystem = 255;
constexpr std::uint8_t kSourceComponent = 190;

} // namespace

std::optional<CommandAck> UdpMavlinkConnection::send_command(const Command &command,
                                                             std::chrono::milliseconds timeout) {
    // Hold the receive mutex for the whole exchange so the command
    // acknowledgement cannot be consumed by a concurrent telemetry pump.
    std::lock_guard lock(receive_mutex_);
    if (!is_connected()) {
        return std::nullopt;
    }
    if (target_system_ == 0 && !wait_for_heartbeat_locked(std::chrono::seconds(2)).has_value()) {
        return std::nullopt;
    }
    if (!get_state_locked().connected || !has_peer()) {
        return std::nullopt;
    }

    const auto frame = command.use_command_int
                          ? encode_command_int(sequence_++, kSourceSystem, kSourceComponent, target_system_,
                                               target_component_, command)
                          : encode_command_long(sequence_++, kSourceSystem, kSourceComponent, target_system_,
                                                target_component_, command);
    const auto sent = send_frame(frame);
    if (sent != frame.size()) {
        return std::nullopt;
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
        const auto message = receive_message_locked((std::max)(std::chrono::milliseconds(1), remaining));
        if (!message.has_value()) {
            continue;
        }
        if (message->system_id != target_system_) {
            continue;
        }
        const auto acknowledgement = decode_command_ack(*message);
        if (acknowledgement.has_value() && acknowledgement->command == command.id) {
            return acknowledgement;
        }
    }
    return std::nullopt;
}

bool UdpMavlinkConnection::send_velocity(const VelocitySetpoint &setpoint) {
    std::lock_guard lock(receive_mutex_);
    const auto state = get_state_locked();
    const bool is_zero = setpoint.vx == 0.0F && setpoint.vy == 0.0F && setpoint.vz == 0.0F && setpoint.yaw_rate == 0.0F;
    if ((!state.connected && !is_zero) || target_system_ == 0 || !has_peer()) {
        return false;
    }
    if (!std::isfinite(setpoint.vx) || !std::isfinite(setpoint.vy) || !std::isfinite(setpoint.vz) ||
        !std::isfinite(setpoint.yaw_rate)) {
        return false;
    }
    const auto frame = encode_velocity_setpoint(sequence_++, kSourceSystem, kSourceComponent, target_system_,
                                                target_component_, setpoint);
    if (frame.empty() || send_frame(frame) != frame.size()) {
        return false;
    }
    velocity_active_ = setpoint.vx != 0.0F || setpoint.vy != 0.0F || setpoint.vz != 0.0F || setpoint.yaw_rate != 0.0F;
    return true;
}

bool UdpMavlinkConnection::request_data_stream(std::uint8_t stream_id, std::uint16_t message_rate) {
    std::lock_guard lock(receive_mutex_);
    if (!is_connected() || target_system_ == 0 || !has_peer()) {
        return false;
    }
    const auto frame = encode_request_data_stream(sequence_++, kSourceSystem, kSourceComponent, target_system_,
                                                  target_component_, stream_id, message_rate, 1);
    if (frame.empty() || send_frame(frame) != frame.size()) {
        return false;
    }
    return true;
}

} // namespace nomad::mavlink
