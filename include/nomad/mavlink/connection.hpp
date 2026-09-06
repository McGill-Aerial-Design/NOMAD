// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/telemetry/state.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace nomad::mavlink {

struct Command {
    std::uint16_t id{};
    std::array<float, 7> parameters{};
    // ArduPilot rejects some location-bearing commands (notably
    // MAV_CMD_DO_REPOSITION) as command_long; set this to send COMMAND_INT
    // with a GLOBAL_RELATIVE_ALT_INT frame instead.
    bool use_command_int{false};
};

struct CommandAck {
    std::uint16_t command{};
    std::uint8_t result{};
};

struct VelocitySetpoint {
    float vx{};
    float vy{};
    float vz{};
    float yaw_rate{};
};

struct Heartbeat {
    std::uint8_t system_id{};
    std::uint8_t component_id{};
    std::uint32_t custom_mode{};
    std::uint8_t vehicle_type{};
    std::uint8_t autopilot_type{};
    std::uint8_t base_mode{};
};

struct FencePoint {
    float latitude_deg{};
    float longitude_deg{};
};

struct FenceStatus {
    std::uint8_t breach_status{};
    std::uint16_t breach_count{};
    std::uint8_t breach_type{};
    std::uint32_t breach_time{};
};

struct FencePlanItem {
    FencePoint point{};
    std::uint16_t sequence{};
    std::uint16_t command{};
    // For a polygon vertex item ArduPilot reads the boundary's total vertex
    // count from param1; every vertex of one polygon carries the same count.
    float param1{};
};

struct ParamValue {
    std::string param_id;
    float value{};
};

class MavlinkConnection {
  public:
    virtual ~MavlinkConnection() = default;

    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool is_connected() const = 0;
    virtual std::optional<Heartbeat> wait_for_heartbeat(std::chrono::milliseconds timeout) = 0;
    virtual std::optional<telemetry::VehicleState> wait_for_state(std::chrono::milliseconds timeout) = 0;
    virtual telemetry::VehicleState get_state() const = 0;
    virtual std::optional<CommandAck> send_command(const Command &command, std::chrono::milliseconds timeout) = 0;
    virtual bool send_velocity(const VelocitySetpoint &setpoint) = 0;
    virtual bool is_velocity_active() const = 0;
    virtual bool request_data_stream(std::uint8_t stream_id, std::uint16_t message_rate) = 0;
    virtual bool send_fence_point(const FencePoint &point, std::uint8_t index, std::uint8_t total) = 0;
    virtual bool request_fence_point(std::uint8_t index) = 0;
    virtual std::optional<FencePoint> wait_for_fence_point(std::chrono::milliseconds timeout) = 0;
    virtual bool upload_fence_plan(const std::vector<FencePlanItem> &items) = 0;
    virtual std::optional<std::vector<FencePlanItem>> download_fence_plan(std::chrono::milliseconds timeout) = 0;
    // Reads a named parameter back from the autopilot. The returned value is
    // authoritative autopilot state, not an acknowledgement.
    virtual std::optional<float> read_param(const std::string &param_id, std::chrono::milliseconds timeout) = 0;
};

} // namespace nomad::mavlink
