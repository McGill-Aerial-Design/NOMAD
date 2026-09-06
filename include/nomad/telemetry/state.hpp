// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

namespace nomad::telemetry {

struct Position {
    double latitude_deg{};
    double longitude_deg{};
    float altitude_m{};
    float relative_altitude_m{};
};

struct Velocity {
    float north_mps{};
    float east_mps{};
    float down_mps{};
    float groundspeed_mps{};
    float climb_rate_mps{};
};

struct Attitude {
    float roll_deg{};
    float pitch_deg{};
    float yaw_deg{};
};

struct Battery {
    float voltage_v{};
    float remaining_percent{};
};

struct Gps {
    std::uint8_t fix_type{};
    std::uint8_t satellites{};
};

struct VehicleState {
    bool connected{false};
    bool heartbeat_fresh{false};
    bool armed{false};
    std::uint8_t system_id{};
    std::uint8_t component_id{};
    std::uint32_t custom_mode{0};
    Position position{};
    Velocity velocity{};
    Attitude attitude{};
    Battery battery{};
    Gps gps{};
    bool position_valid{false};
    bool battery_valid{false};
    bool gps_valid{false};
    bool attitude_valid{false};
};

}  // namespace nomad::telemetry
