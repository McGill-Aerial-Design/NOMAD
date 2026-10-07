// SPDX-License-Identifier: Apache-2.0
#include "nomad/telemetry/identity.hpp"

namespace nomad::telemetry {
namespace {

bool is_copter_type(std::uint8_t vehicle_type) {
    switch (vehicle_type) {
    case kQuadrotor:
    case kCoaxial:
    case kHelicopter:
    case kHexarotor:
    case kOctorotor:
    case kTricopter:
    case kDodecarotor:
    case kDecarotor:
        return true;
    default:
        return false;
    }
}

bool is_quadplane_type(std::uint8_t vehicle_type) {
    return vehicle_type == kVtolDuorotor || vehicle_type == kVtolQuadrotor || vehicle_type == kVtolTiltrotor;
}

} // namespace

VehicleIdentity identify_vehicle(std::uint8_t autopilot_type, std::uint8_t vehicle_type) {
    VehicleIdentity identity{autopilot_type, vehicle_type, AircraftClass::Unknown};
    if (autopilot_type != kArduPilotAutopilot) {
        return identity;
    }
    if (vehicle_type == kFixedWing) {
        identity.aircraft_class = AircraftClass::Plane;
    } else if (is_quadplane_type(vehicle_type)) {
        identity.aircraft_class = AircraftClass::QuadPlane;
    } else if (is_copter_type(vehicle_type)) {
        identity.aircraft_class = AircraftClass::Copter;
    }
    return identity;
}

std::string_view aircraft_class_name(AircraftClass aircraft_class) {
    switch (aircraft_class) {
    case AircraftClass::Unknown:
        return "Unknown";
    case AircraftClass::Copter:
        return "Copter";
    case AircraftClass::Plane:
        return "Plane";
    case AircraftClass::QuadPlane:
        return "QuadPlane";
    }
    return "Unknown";
}

std::optional<std::uint32_t> guided_mode_for(AircraftClass aircraft_class) {
    switch (aircraft_class) {
    case AircraftClass::Copter:
        return 4; // ArduPilot Copter GUIDED.
    case AircraftClass::Plane:
    case AircraftClass::QuadPlane:
        return 15; // ArduPilot Plane/QuadPlane GUIDED.
    case AircraftClass::Unknown:
        return std::nullopt;
    }
    return std::nullopt;
}

bool is_guided_mode(AircraftClass aircraft_class, std::uint32_t custom_mode) {
    const auto guided_mode = guided_mode_for(aircraft_class);
    return guided_mode.has_value() && custom_mode == *guided_mode;
}

bool is_auto_mode(AircraftClass aircraft_class, std::uint32_t custom_mode) {
    switch (aircraft_class) {
    case AircraftClass::Copter:
        return custom_mode == 3; // ArduPilot Copter AUTO.
    case AircraftClass::Plane:
    case AircraftClass::QuadPlane:
        return custom_mode == 10; // ArduPilot Plane AUTO.
    case AircraftClass::Unknown:
        return false;
    }
    return false;
}

bool is_landing_mode(AircraftClass aircraft_class, std::uint32_t custom_mode) {
    switch (aircraft_class) {
    case AircraftClass::Copter:
        return custom_mode == 9;
    case AircraftClass::Plane:
        return false; // AUTO does not prove that Plane is executing NAV_LAND.
    case AircraftClass::QuadPlane:
        return custom_mode == 20; // QLAND is unambiguous; AUTO is not.
    case AircraftClass::Unknown:
        return false;
    }
    return false;
}

bool is_return_to_launch_mode(AircraftClass aircraft_class, std::uint32_t custom_mode) {
    switch (aircraft_class) {
    case AircraftClass::Copter:
        return custom_mode == 6 || custom_mode == 27; // RTL or Auto RTL.
    case AircraftClass::Plane:
        return custom_mode == 11;
    case AircraftClass::QuadPlane:
        return custom_mode == 11 || custom_mode == 21; // RTL or QRTL.
    case AircraftClass::Unknown:
        return false;
    }
    return false;
}

} // namespace nomad::telemetry
