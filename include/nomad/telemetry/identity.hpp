// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

namespace nomad::telemetry {

// These values are the MAVLink HEARTBEAT enum values. Keep the wire values in
// this small boundary so the rest of the core does not depend on generated
// dialect headers.
inline constexpr std::uint8_t kArduPilotAutopilot = 3;
inline constexpr std::uint8_t kFixedWing = 1;
inline constexpr std::uint8_t kQuadrotor = 2;
inline constexpr std::uint8_t kCoaxial = 3;
inline constexpr std::uint8_t kHelicopter = 4;
inline constexpr std::uint8_t kHexarotor = 13;
inline constexpr std::uint8_t kOctorotor = 14;
inline constexpr std::uint8_t kTricopter = 15;
inline constexpr std::uint8_t kVtolDuorotor = 19;
inline constexpr std::uint8_t kVtolQuadrotor = 20;
inline constexpr std::uint8_t kVtolTiltrotor = 21;
inline constexpr std::uint8_t kDodecarotor = 29;
inline constexpr std::uint8_t kDecarotor = 35;

enum class AircraftClass {
    Unknown,
    Copter,
    Plane,
    QuadPlane,
};

struct VehicleIdentity {
    std::uint8_t autopilot_type{};
    std::uint8_t vehicle_type{};
    AircraftClass aircraft_class{AircraftClass::Unknown};
};

VehicleIdentity identify_vehicle(std::uint8_t autopilot_type, std::uint8_t vehicle_type);

std::string_view aircraft_class_name(AircraftClass aircraft_class);

std::optional<std::uint32_t> guided_mode_for(AircraftClass aircraft_class);

bool is_guided_mode(AircraftClass aircraft_class, std::uint32_t custom_mode);

bool is_auto_mode(AircraftClass aircraft_class, std::uint32_t custom_mode);

bool is_landing_mode(AircraftClass aircraft_class, std::uint32_t custom_mode);

bool is_return_to_launch_mode(AircraftClass aircraft_class, std::uint32_t custom_mode);

} // namespace nomad::telemetry
