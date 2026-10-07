// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/telemetry/identity.hpp"

#include <string_view>

namespace nomad::vehicle {

enum class VehicleOperation {
    Arm,
    Disarm,
    SetMode,
    SetGuidedMode,
    Takeoff,
    VtolTakeoff,
    TransitionToFixedWing,
    TransitionToVtol,
    QuadplaneVtolLand,
    FixedWingRoute,
    FixedWingRecovery,
    GotoLocation,
    Land,
    ReturnToLaunch,
    BodyVelocity,
    SetServo,
    SetRelay,
    MotorTest,
    ConfigureGimbal,
    SetGimbalTarget,
    FenceConfiguration,
};

std::string_view operation_name(VehicleOperation operation);

bool supports_operation(telemetry::AircraftClass aircraft_class, VehicleOperation operation);

} // namespace nomad::vehicle
