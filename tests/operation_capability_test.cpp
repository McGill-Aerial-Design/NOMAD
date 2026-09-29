// SPDX-License-Identifier: Apache-2.0
#include "support/fake_connection.hpp"
#include "nomad/vehicle/operation.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "support/test_harness.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace {

using nomad::telemetry::AircraftClass;
using nomad::vehicle::VehicleOperation;

constexpr std::array kAircraftOperations{
    VehicleOperation::Arm,
    VehicleOperation::Disarm,
    VehicleOperation::SetMode,
    VehicleOperation::SetGuidedMode,
    VehicleOperation::Takeoff,
    VehicleOperation::VtolTakeoff,
    VehicleOperation::TransitionToFixedWing,
    VehicleOperation::TransitionToVtol,
    VehicleOperation::QuadplaneVtolLand,
    VehicleOperation::FixedWingRoute,
    VehicleOperation::FixedWingRecovery,
    VehicleOperation::GotoLocation,
    VehicleOperation::Land,
    VehicleOperation::ReturnToLaunch,
    VehicleOperation::BodyVelocity,
    VehicleOperation::SetServo,
    VehicleOperation::SetRelay,
    VehicleOperation::MotorTest,
    VehicleOperation::ConfigureGimbal,
    VehicleOperation::SetGimbalTarget,
    VehicleOperation::ReleasePayload,
    VehicleOperation::FenceConfiguration,
};

void test_copter_supports_qualified_operations() {
    for (const auto operation : kAircraftOperations) {
        if (operation == VehicleOperation::VtolTakeoff || operation == VehicleOperation::TransitionToFixedWing ||
            operation == VehicleOperation::TransitionToVtol || operation == VehicleOperation::QuadplaneVtolLand ||
            operation == VehicleOperation::FixedWingRoute || operation == VehicleOperation::FixedWingRecovery) {
            CHECK(!nomad::vehicle::supports_operation(AircraftClass::Copter, operation));
        } else {
            CHECK(nomad::vehicle::supports_operation(AircraftClass::Copter, operation));
        }
    }
}

void test_plane_and_unknown_fail_closed() {
    constexpr std::array unqualified_classes{
        AircraftClass::Plane,
        AircraftClass::Unknown,
    };
    for (const auto aircraft_class : unqualified_classes) {
        for (const auto operation : kAircraftOperations) {
            CHECK(!nomad::vehicle::supports_operation(aircraft_class, operation));
        }
    }
}

void test_quadplane_supports_only_qualified_operations() {
    CHECK(nomad::vehicle::supports_operation(AircraftClass::QuadPlane, VehicleOperation::Arm));
    CHECK(nomad::vehicle::supports_operation(AircraftClass::QuadPlane, VehicleOperation::SetGuidedMode));
    CHECK(nomad::vehicle::supports_operation(AircraftClass::QuadPlane, VehicleOperation::VtolTakeoff));
    CHECK(nomad::vehicle::supports_operation(AircraftClass::QuadPlane, VehicleOperation::TransitionToFixedWing));
    CHECK(nomad::vehicle::supports_operation(AircraftClass::QuadPlane, VehicleOperation::FixedWingRoute));
    CHECK(nomad::vehicle::supports_operation(AircraftClass::QuadPlane, VehicleOperation::FixedWingRecovery));
    CHECK(nomad::vehicle::supports_operation(AircraftClass::QuadPlane, VehicleOperation::TransitionToVtol));
    CHECK(nomad::vehicle::supports_operation(AircraftClass::QuadPlane, VehicleOperation::QuadplaneVtolLand));
    for (const auto operation : kAircraftOperations) {
        if (operation == VehicleOperation::Arm || operation == VehicleOperation::SetGuidedMode ||
            operation == VehicleOperation::VtolTakeoff || operation == VehicleOperation::TransitionToFixedWing ||
            operation == VehicleOperation::TransitionToVtol || operation == VehicleOperation::QuadplaneVtolLand ||
            operation == VehicleOperation::FixedWingRoute || operation == VehicleOperation::FixedWingRecovery) {
            continue;
        }
        CHECK(!nomad::vehicle::supports_operation(AircraftClass::QuadPlane, operation));
    }
}

void test_unqualified_aircraft_reject_fence_transport() {
    constexpr std::array unqualified_types{
        nomad::telemetry::kFixedWing,
        nomad::telemetry::kVtolQuadrotor,
        std::uint8_t{0},
    };
    const std::vector<nomad::safety::GlobalPoint> boundary{
        {45.0, -73.0},
        {45.0, -73.1},
        {45.1, -73.0},
    };

    for (const auto vehicle_type : unqualified_types) {
        FakeConnection connection;
        connection.connect();
        if (vehicle_type == 0) {
            connection.state->identity = {};
        } else {
            connection.state->identity =
                nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot, vehicle_type);
        }
        nomad::vehicle::Vehicle vehicle(connection);

        CHECK(!vehicle.upload_fence(boundary).success);
        CHECK(!vehicle.verify_fence_uploaded(boundary).success);
        CHECK(connection.fence_plan_upload_count == 0);
        CHECK(connection.fence_plan_download_count == 0);
        CHECK(connection.parameter_read_count == 0);
    }
}

void test_active_copter_velocity_can_stop_after_identity_loss() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    nomad::vehicle::Vehicle vehicle(connection);
    CHECK(vehicle.update_vio(true, 1.0F).success);
    CHECK(vehicle.set_velocity({1.0F, 0.0F, 0.0F, 0.0F}).success);
    CHECK(connection.velocity_send_count == 1);

    connection.state->identity = {};
    CHECK(vehicle.stop_velocity().success);
    CHECK(connection.velocity_send_count == 2);
    CHECK(connection.last_velocity.vx == 0.0F);
    CHECK(connection.last_velocity.vy == 0.0F);
    CHECK(connection.last_velocity.vz == 0.0F);
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_copter_supports_qualified_operations();
        test_plane_and_unknown_fail_closed();
        test_quadplane_supports_only_qualified_operations();
        test_unqualified_aircraft_reject_fence_transport();
        test_active_copter_velocity_can_stop_after_identity_loss();
    });
}
