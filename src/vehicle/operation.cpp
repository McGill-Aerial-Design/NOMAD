// SPDX-License-Identifier: Apache-2.0
#include "nomad/vehicle/operation.hpp"

namespace nomad::vehicle {

std::string_view operation_name(VehicleOperation operation) {
    switch (operation) {
    case VehicleOperation::Arm:
        return "arm";
    case VehicleOperation::Disarm:
        return "disarm";
    case VehicleOperation::SetMode:
        return "set mode";
    case VehicleOperation::SetGuidedMode:
        return "set guided mode";
    case VehicleOperation::Takeoff:
        return "takeoff";
    case VehicleOperation::VtolTakeoff:
        return "vtol takeoff";
    case VehicleOperation::TransitionToFixedWing:
        return "transition to fixed wing";
    case VehicleOperation::TransitionToVtol:
        return "transition to VTOL";
    case VehicleOperation::QuadplaneVtolLand:
        return "QuadPlane VTOL land";
    case VehicleOperation::FixedWingRoute:
        return "fixed-wing route";
    case VehicleOperation::FixedWingRecovery:
        return "fixed-wing recovery";
    case VehicleOperation::GotoLocation:
        return "goto location";
    case VehicleOperation::Land:
        return "land";
    case VehicleOperation::ReturnToLaunch:
        return "return to launch";
    case VehicleOperation::BodyVelocity:
        return "body-frame velocity";
    case VehicleOperation::SetServo:
        return "set servo";
    case VehicleOperation::SetRelay:
        return "set relay";
    case VehicleOperation::MotorTest:
        return "motor test";
    case VehicleOperation::ConfigureGimbal:
        return "configure gimbal";
    case VehicleOperation::ReleasePayload:
        return "release payload";
    case VehicleOperation::FenceConfiguration:
        return "fence configuration";
    }
    return "unknown operation";
}

bool supports_operation(telemetry::AircraftClass aircraft_class, VehicleOperation operation) {
    switch (aircraft_class) {
    case telemetry::AircraftClass::Copter:
        switch (operation) {
        case VehicleOperation::Arm:
        case VehicleOperation::Disarm:
        case VehicleOperation::SetMode:
        case VehicleOperation::SetGuidedMode:
        case VehicleOperation::Takeoff:
        case VehicleOperation::GotoLocation:
        case VehicleOperation::Land:
        case VehicleOperation::ReturnToLaunch:
        case VehicleOperation::BodyVelocity:
        case VehicleOperation::SetServo:
        case VehicleOperation::SetRelay:
        case VehicleOperation::MotorTest:
        case VehicleOperation::ConfigureGimbal:
        case VehicleOperation::ReleasePayload:
        case VehicleOperation::FenceConfiguration:
            return true;
        case VehicleOperation::VtolTakeoff:
        case VehicleOperation::TransitionToFixedWing:
        case VehicleOperation::TransitionToVtol:
        case VehicleOperation::QuadplaneVtolLand:
        case VehicleOperation::FixedWingRoute:
        case VehicleOperation::FixedWingRecovery:
            return false;
        }
        return false;
    case telemetry::AircraftClass::Plane:
        return false;
    case telemetry::AircraftClass::QuadPlane:
        switch (operation) {
        case VehicleOperation::Arm:
        case VehicleOperation::SetGuidedMode:
        case VehicleOperation::VtolTakeoff:
        case VehicleOperation::TransitionToFixedWing:
        case VehicleOperation::TransitionToVtol:
        case VehicleOperation::QuadplaneVtolLand:
        case VehicleOperation::FixedWingRoute:
        case VehicleOperation::FixedWingRecovery:
            return true;
        case VehicleOperation::Disarm:
        case VehicleOperation::SetMode:
        case VehicleOperation::Takeoff:
        case VehicleOperation::GotoLocation:
        case VehicleOperation::Land:
        case VehicleOperation::ReturnToLaunch:
        case VehicleOperation::BodyVelocity:
        case VehicleOperation::SetServo:
        case VehicleOperation::SetRelay:
        case VehicleOperation::MotorTest:
        case VehicleOperation::ConfigureGimbal:
        case VehicleOperation::ReleasePayload:
        case VehicleOperation::FenceConfiguration:
            return false;
        }
        return false;
    case telemetry::AircraftClass::Unknown:
        return false;
    }
    return false;
}

} // namespace nomad::vehicle
