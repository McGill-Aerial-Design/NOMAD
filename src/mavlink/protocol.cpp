// SPDX-License-Identifier: Apache-2.0
#include "nomad/mavlink/protocol.hpp"

#include "generated.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace nomad::mavlink {
namespace {

constexpr std::uint8_t kMavlinkV1Marker = 0xfe;
constexpr std::uint8_t kMavlinkV2Marker = 0xfd;
constexpr std::uint8_t kArmModeFlag = 0x80;  // MAV_MODE_FLAG_SAFETY_ARMED

// Message identifiers used by the telemetry and command paths. The values are
// the generated dialect constants, so they track the pinned firmware tables.
constexpr std::uint32_t kHeartbeatId = MAVLINK_MSG_ID_HEARTBEAT;
constexpr std::uint32_t kSysStatusId = MAVLINK_MSG_ID_SYS_STATUS;
constexpr std::uint32_t kGpsRawIntId = MAVLINK_MSG_ID_GPS_RAW_INT;
constexpr std::uint32_t kAttitudeId = MAVLINK_MSG_ID_ATTITUDE;
constexpr std::uint32_t kGlobalPositionIntId = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
constexpr std::uint32_t kVfrHudId = MAVLINK_MSG_ID_VFR_HUD;
constexpr std::uint32_t kCommandIntId = MAVLINK_MSG_ID_COMMAND_INT;
constexpr std::uint32_t kCommandLongId = MAVLINK_MSG_ID_COMMAND_LONG;
constexpr std::uint32_t kCommandAckId = MAVLINK_MSG_ID_COMMAND_ACK;
constexpr std::uint32_t kSetPositionTargetLocalNedId = MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED;
constexpr std::uint32_t kRequestDataStreamId = MAVLINK_MSG_ID_REQUEST_DATA_STREAM;

std::uint16_t accumulate_crc(std::uint16_t crc, std::uint8_t value) {
    value ^= static_cast<std::uint8_t>(crc & 0xff);
    value ^= static_cast<std::uint8_t>(value << 4);
    return static_cast<std::uint16_t>((crc >> 8) ^ (static_cast<std::uint16_t>(value) << 8) ^
                                      (static_cast<std::uint16_t>(value) << 3) ^
                                      (static_cast<std::uint16_t>(value) >> 4));
}

std::uint16_t calculate_crc(std::span<const std::uint8_t> bytes, std::uint8_t extra) {
    std::uint16_t crc = 0xffff;
    for (const auto byte : bytes) {
        crc = accumulate_crc(crc, byte);
    }
    return accumulate_crc(crc, extra);
}

std::vector<std::uint8_t> append_crc(std::vector<std::uint8_t> frame, std::size_t crc_start, std::uint8_t extra) {
    const auto crc = calculate_crc(std::span<const std::uint8_t>(frame).subspan(crc_start), extra);
    frame.push_back(static_cast<std::uint8_t>(crc & 0xff));
    frame.push_back(static_cast<std::uint8_t>(crc >> 8));
    return frame;
}

std::optional<std::uint8_t> crc_extra(std::uint32_t message_id) {
    const auto *entry = mavlink_get_msg_entry(message_id);
    if (entry == nullptr) {
        return std::nullopt;
    }
    return entry->crc_extra;
}

// MAVLink2 senders trim trailing zero bytes from the payload; receivers must
// treat the missing tail as zero. The full lengths come from the generated
// dialect table (max_msg_len includes trailing extension fields).
std::optional<std::size_t> expected_payload_length(std::uint32_t message_id) {
    const auto *entry = mavlink_get_msg_entry(message_id);
    if (entry == nullptr) {
        return std::nullopt;
    }
    return entry->max_msg_len;
}

std::optional<Message> decode_v1(std::span<const std::uint8_t> frame) {
    const auto payload_size = frame[1];
    const auto expected_size = static_cast<std::size_t>(payload_size) + 8;
    if (frame.size() != expected_size) {
        return std::nullopt;
    }

    const auto message_id = static_cast<std::uint32_t>(frame[5]);
    const auto extra = crc_extra(message_id);
    if (!extra.has_value()) {
        return std::nullopt;
    }
    const auto actual_crc = static_cast<std::uint16_t>(frame[6 + payload_size]) |
                            static_cast<std::uint16_t>(static_cast<std::uint16_t>(frame[7 + payload_size]) << 8);
    if (actual_crc != calculate_crc(frame.subspan(1, 5 + payload_size), *extra)) {
        return std::nullopt;
    }

    Message message;
    message.system_id = frame[3];
    message.component_id = frame[4];
    message.message_id = message_id;
    message.payload.assign(frame.begin() + 6, frame.begin() + 6 + payload_size);
    return message;
}

std::optional<Message> decode_v2(std::span<const std::uint8_t> frame) {
    const auto payload_size = frame[1];
    if (frame[2] != 0) {
        return std::nullopt;
    }
    const auto expected_size = static_cast<std::size_t>(payload_size) + 12;
    if (frame.size() != expected_size) {
        return std::nullopt;
    }

    const auto message_id = static_cast<std::uint32_t>(frame[7]) | (static_cast<std::uint32_t>(frame[8]) << 8) |
                            (static_cast<std::uint32_t>(frame[9]) << 16);
    const auto extra = crc_extra(message_id);
    if (!extra.has_value()) {
        return std::nullopt;
    }
    const auto crc_offset = static_cast<std::size_t>(10 + payload_size);
    const auto actual_crc = static_cast<std::uint16_t>(frame[crc_offset]) |
                            static_cast<std::uint16_t>(static_cast<std::uint16_t>(frame[crc_offset + 1]) << 8);
    if (actual_crc != calculate_crc(frame.subspan(1, 9 + payload_size), *extra)) {
        return std::nullopt;
    }

    Message message;
    message.system_id = frame[5];
    message.component_id = frame[6];
    message.message_id = message_id;
    message.payload.assign(frame.begin() + 10, frame.begin() + 10 + payload_size);
    const auto full_length = expected_payload_length(message_id);
    if (full_length.has_value() && payload_size < *full_length) {
        message.payload.resize(*full_length, 0);
    }
    return message;
}

void update_heartbeat(const Message &message, telemetry::VehicleState &state) {
    const auto heartbeat = decode_heartbeat(message);
    if (!heartbeat.has_value()) {
        return;
    }
    state.connected = true;
    state.system_id = heartbeat->system_id;
    state.component_id = heartbeat->component_id;
    state.custom_mode = heartbeat->custom_mode;
    state.armed = (heartbeat->base_mode & kArmModeFlag) != 0;
}

void update_system_status(const Message &message, telemetry::VehicleState &state) {
    if (message.payload.size() < MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN) {
        return;
    }
    const auto msg = to_generated_message(message);
    mavlink_sys_status_t status{};
    mavlink_msg_sys_status_decode(&msg, &status);
    if (status.voltage_battery != 0xffff) {
        state.battery.voltage_v = static_cast<float>(status.voltage_battery) / 1000.0F;
        state.battery_valid = true;
    }
    if (status.battery_remaining >= 0) {
        state.battery.remaining_percent = static_cast<float>(status.battery_remaining);
        state.battery_valid = true;
    }
}

void update_gps(const Message &message, telemetry::VehicleState &state) {
    if (message.payload.size() < MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN) {
        return;
    }
    const auto msg = to_generated_message(message);
    mavlink_gps_raw_int_t gps{};
    mavlink_msg_gps_raw_int_decode(&msg, &gps);
    state.gps.fix_type = gps.fix_type;
    state.gps.satellites = gps.satellites_visible;
    state.gps_valid = state.gps.fix_type > 0;
}

void update_position(const Message &message, telemetry::VehicleState &state) {
    if (message.payload.size() < MAVLINK_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN) {
        return;
    }
    const auto msg = to_generated_message(message);
    mavlink_global_position_int_t position{};
    mavlink_msg_global_position_int_decode(&msg, &position);
    state.position.latitude_deg = static_cast<double>(position.lat) / 1e7;
    state.position.longitude_deg = static_cast<double>(position.lon) / 1e7;
    state.position.altitude_m = static_cast<float>(position.alt) / 1000.0F;
    state.position.relative_altitude_m = static_cast<float>(position.relative_alt) / 1000.0F;
    state.velocity.north_mps = static_cast<float>(position.vx) / 100.0F;
    state.velocity.east_mps = static_cast<float>(position.vy) / 100.0F;
    state.velocity.down_mps = static_cast<float>(position.vz) / 100.0F;
    state.velocity.groundspeed_mps = std::sqrt(state.velocity.north_mps * state.velocity.north_mps +
                                               state.velocity.east_mps * state.velocity.east_mps);
    state.position_valid = state.gps_valid || state.position.latitude_deg != 0.0 || state.position.longitude_deg != 0.0;
}

void update_attitude(const Message &message, telemetry::VehicleState &state) {
    if (message.payload.size() < MAVLINK_MSG_ID_ATTITUDE_MIN_LEN) {
        return;
    }
    const auto msg = to_generated_message(message);
    mavlink_attitude_t attitude{};
    mavlink_msg_attitude_decode(&msg, &attitude);
    constexpr float kRadiansToDegrees = 57.295779513F;
    state.attitude.roll_deg = attitude.roll * kRadiansToDegrees;
    state.attitude.pitch_deg = attitude.pitch * kRadiansToDegrees;
    state.attitude.yaw_deg = attitude.yaw * kRadiansToDegrees;
    state.attitude_valid = true;
}

void update_vfr_hud(const Message &message, telemetry::VehicleState &state) {
    if (message.payload.size() < MAVLINK_MSG_ID_VFR_HUD_MIN_LEN) {
        return;
    }
    const auto msg = to_generated_message(message);
    mavlink_vfr_hud_t hud{};
    mavlink_msg_vfr_hud_decode(&msg, &hud);
    state.velocity.groundspeed_mps = hud.groundspeed;
    state.position.altitude_m = hud.alt;
    state.velocity.climb_rate_mps = hud.climb;
}

} // namespace

std::optional<std::vector<std::uint8_t>> encode_message(std::uint8_t sequence, std::uint8_t system_id,
                                                        std::uint8_t component_id, std::uint32_t message_id,
                                                        std::span<const std::uint8_t> payload, bool mavlink_v2) {
    const auto extra = crc_extra(message_id);
    if (!extra.has_value() || payload.size() > 255 || (!mavlink_v2 && message_id > 255) || message_id > 0xffffff) {
        return std::nullopt;
    }

    std::vector<std::uint8_t> frame;
    if (!mavlink_v2) {
        frame.reserve(payload.size() + 8);
        frame = {kMavlinkV1Marker, static_cast<std::uint8_t>(payload.size()), sequence, system_id,
                 component_id,     static_cast<std::uint8_t>(message_id)};
        frame.insert(frame.end(), payload.begin(), payload.end());
        return append_crc(std::move(frame), 1, *extra);
    }

    frame.reserve(payload.size() + 12);
    frame = {kMavlinkV2Marker,
             static_cast<std::uint8_t>(payload.size()),
             0,
             0,
             sequence,
             system_id,
             component_id,
             static_cast<std::uint8_t>(message_id & 0xff),
             static_cast<std::uint8_t>((message_id >> 8) & 0xff),
             static_cast<std::uint8_t>((message_id >> 16) & 0xff)};
    frame.insert(frame.end(), payload.begin(), payload.end());
    return append_crc(std::move(frame), 1, *extra);
}

std::vector<std::uint8_t> encode_gcs_heartbeat(std::uint8_t sequence, std::uint8_t system_id,
                                               std::uint8_t component_id) {
    mavlink_heartbeat_t heartbeat{};
    heartbeat.type = MAV_TYPE_GCS;
    heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode = 0;
    heartbeat.custom_mode = 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;
    heartbeat.mavlink_version = MAVLINK_VERSION;
    mavlink_message_t msg{};
    mavlink_msg_heartbeat_encode(system_id, component_id, &msg, &heartbeat);
    // The vehicle's stream-rate logic keys on the MAVLink2 framing used by
    // real GCS software, so announce with a v2 frame.
    return encode_message(sequence, system_id, component_id, kHeartbeatId,
                          payload_of(msg, MAVLINK_MSG_ID_HEARTBEAT_LEN), true)
        .value_or(std::vector<std::uint8_t>{});
}

std::vector<std::uint8_t> encode_velocity_setpoint(std::uint8_t sequence, std::uint8_t system_id,
                                                   std::uint8_t component_id, std::uint8_t target_system,
                                                   std::uint8_t target_component, const VelocitySetpoint &setpoint) {
    mavlink_set_position_target_local_ned_t target{};
    target.vx = setpoint.vx;
    target.vy = setpoint.vy;
    target.vz = setpoint.vz;
    target.yaw_rate = setpoint.yaw_rate;
    target.type_mask = 0x07c7;
    target.target_system = target_system;
    target.target_component = target_component;
    target.coordinate_frame = 9;  // MAV_FRAME_BODY_OFFSET_NED
    mavlink_message_t msg{};
    mavlink_msg_set_position_target_local_ned_encode(system_id, component_id, &msg, &target);
    return encode_message(sequence, system_id, component_id, kSetPositionTargetLocalNedId,
                          payload_of(msg, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN), false)
        .value_or(std::vector<std::uint8_t>{});
}

std::vector<std::uint8_t> encode_command_long(std::uint8_t sequence, std::uint8_t system_id, std::uint8_t component_id,
                                              std::uint8_t target_system, std::uint8_t target_component,
                                              const Command &command) {
    mavlink_command_long_t target{};
    target.param1 = command.parameters[0];
    target.param2 = command.parameters[1];
    target.param3 = command.parameters[2];
    target.param4 = command.parameters[3];
    target.param5 = command.parameters[4];
    target.param6 = command.parameters[5];
    target.param7 = command.parameters[6];
    target.command = command.id;
    target.target_system = target_system;
    target.target_component = target_component;
    mavlink_message_t msg{};
    mavlink_msg_command_long_encode(system_id, component_id, &msg, &target);
    return encode_message(sequence, system_id, component_id, kCommandLongId, payload_of(msg, MAVLINK_MSG_ID_COMMAND_LONG_LEN),
                          false)
        .value_or(std::vector<std::uint8_t>{});
}

std::vector<std::uint8_t> encode_command_int(std::uint8_t sequence, std::uint8_t system_id, std::uint8_t component_id,
                                             std::uint8_t target_system, std::uint8_t target_component,
                                             const Command &command) {
    // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT (6): latitude/longitude are 1e7-
    // scaled integers, altitude is above home. Frame 2 is MAV_FRAME_MISSION
    // and is answered "Unknown mavlink coordinate frame" with MAV_RESULT_DENIED
    // by Copter 4.7.x.
    constexpr std::uint8_t kRelativeAltitudeIntFrame = 6;
    mavlink_command_int_t target{};
    target.param1 = command.parameters[0];
    target.param2 = command.parameters[1];
    target.param3 = command.parameters[2];
    target.param4 = command.parameters[3];
    // 1e7-scaled integers with truncation toward zero, matching pymavlink's
    // int(x * 1e7) semantics; the arithmetic is done in double so a float
    // parameter keeps sub-meter precision at the 1e7 scale.
    target.x = static_cast<std::int32_t>(static_cast<double>(command.parameters[4]) * 1e7);
    target.y = static_cast<std::int32_t>(static_cast<double>(command.parameters[5]) * 1e7);
    target.z = command.parameters[6];
    target.command = command.id;
    target.target_system = target_system;
    target.target_component = target_component;
    target.frame = kRelativeAltitudeIntFrame;
    mavlink_message_t msg{};
    mavlink_msg_command_int_encode(system_id, component_id, &msg, &target);
    return encode_message(sequence, system_id, component_id, kCommandIntId,
                          payload_of(msg, MAVLINK_MSG_ID_COMMAND_INT_LEN), true)
        .value_or(std::vector<std::uint8_t>{});
}

std::optional<Message> decode_message(std::span<const std::uint8_t> frame) {
    if (frame.empty()) {
        return std::nullopt;
    }
    if (frame[0] == kMavlinkV1Marker && frame.size() >= 8) {
        return decode_v1(frame);
    }
    if (frame[0] == kMavlinkV2Marker && frame.size() >= 12) {
        return decode_v2(frame);
    }
    return std::nullopt;
}

std::optional<Message> decode_datagram(std::span<const std::uint8_t> data, std::size_t &consumed) {
    // MAVProxy coalesces bursts of telemetry into single UDP packets of
    // roughly 1400 bytes, so a datagram may contain several MAVLink frames.
    // Skip any stray bytes, then decode the first complete frame and report
    // how many bytes it consumed.
    std::size_t offset = 0;
    while (offset < data.size()) {
        const auto remaining = data.subspan(offset);
        std::size_t frame_size = 0;
        if (remaining[0] == kMavlinkV1Marker && remaining.size() >= 8) {
            frame_size = 8 + static_cast<std::size_t>(remaining[1]);
        } else if (remaining[0] == kMavlinkV2Marker && remaining.size() >= 12 && remaining[2] == 0) {
            frame_size = 12 + static_cast<std::size_t>(remaining[1]);
        } else {
            offset += 1;
            continue;
        }
        if (remaining.size() < frame_size) {
            break;
        }
        const auto message = decode_message(remaining.first(frame_size));
        if (message.has_value()) {
            consumed = offset + frame_size;
            return message;
        }
        offset += 1;
    }
    consumed = 0;
    return std::nullopt;
}

std::optional<Heartbeat> decode_heartbeat(const Message &message) {
    if (message.message_id != kHeartbeatId || message.payload.size() < MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN) {
        return std::nullopt;
    }
    const auto msg = to_generated_message(message);
    mavlink_heartbeat_t heartbeat{};
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    return Heartbeat{
        message.system_id,  message.component_id, heartbeat.custom_mode,
        heartbeat.type,     heartbeat.autopilot,  heartbeat.base_mode,
    };
}

bool accepts_heartbeat(const Heartbeat &heartbeat, std::uint8_t target_system) {
    if (heartbeat.vehicle_type == 6) {  // MAV_TYPE_GCS
        return false;
    }
    return target_system == 0 || heartbeat.system_id == target_system;
}

std::optional<CommandAck> decode_command_ack(const Message &message) {
    if (message.message_id != kCommandAckId || message.payload.size() < MAVLINK_MSG_ID_COMMAND_ACK_MIN_LEN) {
        return std::nullopt;
    }
    const auto msg = to_generated_message(message);
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&msg, &ack);
    return CommandAck{ack.command, ack.result};
}

bool update_vehicle_state(const Message &message, telemetry::VehicleState &state) {
    switch (message.message_id) {
    case kHeartbeatId:
        update_heartbeat(message, state);
        return true;
    case kSysStatusId:
        update_system_status(message, state);
        return true;
    case kGpsRawIntId:
        update_gps(message, state);
        return true;
    case kAttitudeId:
        update_attitude(message, state);
        return true;
    case kGlobalPositionIntId:
        update_position(message, state);
        return true;
    case kVfrHudId:
        update_vfr_hud(message, state);
        return true;
    default:
        return false;
    }
}

std::vector<std::uint8_t> encode_request_data_stream(std::uint8_t sequence, std::uint8_t system_id,
                                                     std::uint8_t component_id, std::uint8_t target_system,
                                                     std::uint8_t target_component, std::uint8_t stream_id,
                                                     std::uint16_t message_rate, std::uint8_t start_stop) {
    mavlink_request_data_stream_t request{};
    request.target_system = target_system;
    request.target_component = target_component;
    request.req_stream_id = stream_id;
    request.req_message_rate = message_rate;
    request.start_stop = start_stop;
    mavlink_message_t msg{};
    mavlink_msg_request_data_stream_encode(system_id, component_id, &msg, &request);
    return encode_message(sequence, system_id, component_id, kRequestDataStreamId,
                          payload_of(msg, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN), false)
        .value_or(std::vector<std::uint8_t>{});
}

} // namespace nomad::mavlink
