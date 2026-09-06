// SPDX-License-Identifier: Apache-2.0
// Parameter read-back: the MAVLink wire codec for PARAM_REQUEST_READ and
// PARAM_VALUE plus the UDP connection conversation that reads a named
// parameter from the autopilot. Read-back gives the core authoritative
// autopilot state (for example FENCE_ENABLE) instead of relying on
// acknowledgements alone.
#include "nomad/mavlink/udp_connection.hpp"

#include "generated.hpp"
#include "nomad/mavlink/protocol.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

namespace nomad::mavlink {
namespace {

constexpr std::uint32_t kParamRequestReadId = MAVLINK_MSG_ID_PARAM_REQUEST_READ;
constexpr std::uint32_t kParamValueId = MAVLINK_MSG_ID_PARAM_VALUE;
constexpr std::uint8_t kParamNameLength = 16;

// GCS identity; must match the values in udp_connection.cpp.
constexpr std::uint8_t kSourceSystem = 255;
constexpr std::uint8_t kSourceComponent = 190;

std::string bounded_wire_name(const char (&wire_id)[kParamNameLength]) {
    std::size_t length = 0;
    while (length < kParamNameLength && wire_id[length] != '\0') {
        length += 1;
    }
    return std::string(wire_id, length);
}

bool param_id_matches(const std::string &wire_id, const std::string &expected) {
    if (wire_id.size() != expected.size()) {
        return false;
    }
    for (std::size_t index = 0; index < wire_id.size(); ++index) {
        if (std::toupper(static_cast<unsigned char>(wire_id[index])) !=
            std::toupper(static_cast<unsigned char>(expected[index]))) {
            return false;
        }
    }
    return true;
}

} // namespace

std::vector<std::uint8_t> encode_param_request_read(std::uint8_t sequence, std::uint8_t system_id,
                                                    std::uint8_t component_id, std::uint8_t target_system,
                                                    std::uint8_t target_component, const std::string &param_id,
                                                    std::int16_t param_index) {
    mavlink_param_request_read_t target{};
    target.target_system = target_system;
    target.target_component = target_component;
    target.param_index = param_index;
    std::memset(target.param_id, 0, sizeof(target.param_id));
    const auto copy_length = (std::min)(param_id.size(), std::size_t{kParamNameLength - 1});
    std::memcpy(target.param_id, param_id.data(), copy_length);
    mavlink_message_t msg{};
    mavlink_msg_param_request_read_encode(system_id, component_id, &msg, &target);
    return encode_message(sequence, system_id, component_id, kParamRequestReadId,
                          payload_of(msg, MAVLINK_MSG_ID_PARAM_REQUEST_READ_LEN), true)
        .value_or(std::vector<std::uint8_t>{});
}

std::optional<ParamValue> decode_param_value(const Message &message) {
    if (message.message_id != kParamValueId || message.payload.size() < MAVLINK_MSG_ID_PARAM_VALUE_MIN_LEN) {
        return std::nullopt;
    }
    const auto msg = to_generated_message(message);
    mavlink_param_value_t value{};
    mavlink_msg_param_value_decode(&msg, &value);
    return ParamValue{
        bounded_wire_name(value.param_id),
        value.param_value,
    };
}

std::optional<float> UdpMavlinkConnection::read_param(const std::string &param_id, std::chrono::milliseconds timeout) {
    std::lock_guard lock(receive_mutex_);
    if (!is_connected() || target_system_ == 0 || !has_peer() || param_id.empty() || param_id.size() >= kParamNameLength) {
        return std::nullopt;
    }
    const auto request = encode_param_request_read(sequence_++, kSourceSystem, kSourceComponent, target_system_,
                                                   target_component_, param_id, -1);
    if (send_frame(request) != request.size()) {
        return std::nullopt;
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        const auto message = receive_message_locked((std::max)(std::chrono::milliseconds(1), remaining));
        if (!message.has_value() || message->system_id != target_system_) {
            continue;
        }
        const auto value = decode_param_value(*message);
        if (value.has_value() && param_id_matches(value->param_id, param_id)) {
            return value->value;
        }
    }
    return std::nullopt;
}

} // namespace nomad::mavlink
