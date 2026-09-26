// SPDX-License-Identifier: Apache-2.0
#include "nomad/telemetry/identity.hpp"
#include "support/test_harness.hpp"

#include <cstdint>

namespace {

using nomad::telemetry::AircraftClass;

void test_ardupilot_aircraft_classes() {
    CHECK(nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                              nomad::telemetry::kQuadrotor)
              .aircraft_class == AircraftClass::Copter);
    CHECK(nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                              nomad::telemetry::kFixedWing)
              .aircraft_class == AircraftClass::Plane);
    CHECK(nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                              nomad::telemetry::kVtolQuadrotor)
              .aircraft_class == AircraftClass::QuadPlane);
}

void test_unsupported_identity_is_unknown() {
    const auto unknown_autopilot = nomad::telemetry::identify_vehicle(12, nomad::telemetry::kQuadrotor);
    CHECK(unknown_autopilot.aircraft_class == AircraftClass::Unknown);
    CHECK(nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot, 99).aircraft_class ==
          AircraftClass::Unknown);
    CHECK(nomad::telemetry::aircraft_class_name(AircraftClass::Unknown) == "Unknown");
    CHECK(nomad::telemetry::aircraft_class_name(AircraftClass::Copter) == "Copter");
    CHECK(nomad::telemetry::aircraft_class_name(AircraftClass::Plane) == "Plane");
    CHECK(nomad::telemetry::aircraft_class_name(AircraftClass::QuadPlane) == "QuadPlane");
}

void test_mode_semantics_are_aircraft_specific() {
    CHECK(nomad::telemetry::is_guided_mode(AircraftClass::Copter, 4));
    CHECK(nomad::telemetry::is_guided_mode(AircraftClass::Plane, 15));
    CHECK(nomad::telemetry::is_guided_mode(AircraftClass::QuadPlane, 15));
    CHECK(!nomad::telemetry::is_guided_mode(AircraftClass::QuadPlane, 4));
    CHECK(!nomad::telemetry::is_guided_mode(AircraftClass::Unknown, 4));
    CHECK(nomad::telemetry::is_auto_mode(AircraftClass::Copter, 3));
    CHECK(nomad::telemetry::is_auto_mode(AircraftClass::QuadPlane, 10));
    CHECK(!nomad::telemetry::is_auto_mode(AircraftClass::QuadPlane, 15));
    CHECK(!nomad::telemetry::is_auto_mode(AircraftClass::Unknown, 10));
    CHECK(nomad::telemetry::is_landing_mode(AircraftClass::Copter, 9));
    CHECK(!nomad::telemetry::is_landing_mode(AircraftClass::Plane, 10));
    CHECK(!nomad::telemetry::is_landing_mode(AircraftClass::QuadPlane, 10));
    CHECK(nomad::telemetry::is_landing_mode(AircraftClass::QuadPlane, 20));
    CHECK(nomad::telemetry::is_return_to_launch_mode(AircraftClass::Plane, 11));
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_ardupilot_aircraft_classes();
        test_unsupported_identity_is_unknown();
        test_mode_semantics_are_aircraft_specific();
    });
}
