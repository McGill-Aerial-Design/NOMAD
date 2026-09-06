// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mavlink/connection.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace nomad::mavlink {

struct Message {
    std::uint8_t system_id{};
    std::uint8_t component_id{};
    std::uint32_t message_id{};
    std::vector<std::uint8_t> payload;
};

std::optional<std::vector<std::uint8_t>> encode_message(std::uint8_t sequence, std::uint8_t system_id,
                                                        std::uint8_t component_id, std::uint32_t message_id,
                                                        std::span<const std::uint8_t> payload, bool mavlink_v2);

// Standard GCS heartbeat (MAV_TYPE_GCS). Many relay and router setups only
// start streaming a UDP leg once the endpoint announces itself, so the core
// sends this heartbeat while it waits for the vehicle's own heartbeat.
std::vector<std::uint8_t> encode_gcs_heartbeat(std::uint8_t sequence, std::uint8_t system_id,
                                               std::uint8_t component_id);

std::vector<std::uint8_t> encode_velocity_setpoint(std::uint8_t sequence, std::uint8_t system_id,
                                                   std::uint8_t component_id, std::uint8_t target_system,
                                                   std::uint8_t target_component, const VelocitySetpoint &setpoint);

std::vector<std::uint8_t> encode_command_long(std::uint8_t sequence, std::uint8_t system_id, std::uint8_t component_id,
                                              std::uint8_t target_system, std::uint8_t target_component,
                                              const Command &command);

// MAVLink COMMAND_INT. ArduPilot handles MAV_CMD_DO_REPOSITION only through
// command_int and rejects the command_long form (MAV_RESULT_UNSUPPORTED), so
// location-bearing commands are encoded here with a GLOBAL_RELATIVE_ALT_INT
// frame and 1e7-scaled integer coordinates.
std::vector<std::uint8_t> encode_command_int(std::uint8_t sequence, std::uint8_t system_id, std::uint8_t component_id,
                                             std::uint8_t target_system, std::uint8_t target_component,
                                             const Command &command);

std::optional<Message> decode_message(std::span<const std::uint8_t> frame);

// Decode the first complete, valid MAVLink frame in a datagram and report how
// many bytes it consumed. A UDP datagram can carry several frames (MAVProxy
// coalesces bursts); callers loop with the returned offset to decode the rest.
std::optional<Message> decode_datagram(std::span<const std::uint8_t> data, std::size_t &consumed);

std::optional<Heartbeat> decode_heartbeat(const Message &message);
bool accepts_heartbeat(const Heartbeat &heartbeat, std::uint8_t target_system);
std::optional<CommandAck> decode_command_ack(const Message &message);
bool update_vehicle_state(const Message &message, telemetry::VehicleState &state);
std::vector<std::uint8_t> encode_fence_point(std::uint8_t sequence, std::uint8_t system_id, std::uint8_t component_id,
                                             std::uint8_t target_system, std::uint8_t target_component,
                                             std::uint8_t index, std::uint8_t total, const FencePoint &point);

std::vector<std::uint8_t> encode_request_data_stream(std::uint8_t sequence, std::uint8_t system_id,
                                                     std::uint8_t component_id, std::uint8_t target_system,
                                                     std::uint8_t target_component, std::uint8_t stream_id,
                                                     std::uint16_t message_rate, std::uint8_t start_stop);

std::optional<FencePoint> decode_fence_point(const Message &message);
std::optional<FenceStatus> decode_fence_status(const Message &message);
std::vector<std::uint8_t> encode_mission_count(std::uint8_t sequence, std::uint8_t system_id,
                                                std::uint8_t component_id, std::uint8_t target_system,
                                                std::uint8_t target_component, std::uint16_t count,
                                                std::uint8_t mission_type);
std::vector<std::uint8_t> encode_mission_item_int(std::uint8_t sequence, std::uint8_t system_id,
                                                  std::uint8_t component_id, std::uint8_t target_system,
                                                  std::uint8_t target_component, const FencePlanItem &item,
                                                  std::uint8_t mission_type);
std::vector<std::uint8_t> encode_mission_request_list(std::uint8_t sequence, std::uint8_t system_id,
                                                      std::uint8_t component_id, std::uint8_t target_system,
                                                      std::uint8_t target_component, std::uint8_t mission_type);
std::vector<std::uint8_t> encode_mission_request_int(std::uint8_t sequence, std::uint8_t system_id,
                                                     std::uint8_t component_id, std::uint8_t target_system,
                                                     std::uint8_t target_component, std::uint16_t index,
                                                     std::uint8_t mission_type);
std::vector<std::uint8_t> encode_mission_clear_all(std::uint8_t sequence, std::uint8_t system_id,
                                                    std::uint8_t component_id, std::uint8_t target_system,
                                                    std::uint8_t target_component, std::uint8_t mission_type);
std::optional<std::uint16_t> decode_mission_sequence(const Message &message);
std::optional<std::uint16_t> decode_mission_count(const Message &message);
std::optional<std::uint8_t> decode_mission_ack(const Message &message);
std::vector<std::uint8_t> encode_param_request_read(std::uint8_t sequence, std::uint8_t system_id,
                                                    std::uint8_t component_id, std::uint8_t target_system,
                                                    std::uint8_t target_component, const std::string &param_id,
                                                    std::int16_t param_index);
std::optional<ParamValue> decode_param_value(const Message &message);
} // namespace nomad::mavlink
