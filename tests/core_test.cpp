// SPDX-License-Identifier: Apache-2.0
#include "fake_connection.hpp"
#include "nomad/mission/executor.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "test_harness.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void configure_quadplane_takeoff_state(FakeConnection &connection, float relative_altitude_m = 0.0F) {
    connection.state->identity =
        nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                            nomad::telemetry::kVtolQuadrotor);
    connection.state->connected = true;
    connection.state->heartbeat_fresh = true;
    connection.state->position_valid = true;
    connection.state->position.relative_altitude_m = relative_altitude_m;
    connection.state->position_updated_at = std::chrono::steady_clock::now();
    connection.state->gps_valid = true;
    connection.state->gps.fix_type = 3;
    connection.state->gps.satellites = 10;
    connection.state->gps_updated_at = std::chrono::steady_clock::now();
}

void test_state_is_available_through_vehicle() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);

    const auto state = vehicle.wait_for_state(std::chrono::seconds(1));

    CHECK(state.has_value());
    CHECK(state->connected);
    CHECK(state->system_id == 1);
    CHECK(state->identity.aircraft_class == nomad::telemetry::AircraftClass::Copter);
}

void test_state_requires_connection() {
    FakeConnection connection;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto state = vehicle.wait_for_state(std::chrono::seconds(1));

    CHECK(!state.has_value());
}

void test_arm_sends_arm_command() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{400, 0};

    const auto result = vehicle.arm();

    CHECK(result.success);
    CHECK(result.message == "arm verified");
    CHECK(connection.last_command.id == 400);
    CHECK(connection.last_command.parameters[0] == 1.0F);
}

void test_takeoff_rejects_invalid_altitude() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.takeoff(0.0F);

    CHECK(!result.success);
    CHECK(connection.last_command.id == 0);
}

void test_mode_and_takeoff_are_verified() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{176, 0};

    const auto mode_result = vehicle.set_mode(4);
    CHECK(mode_result.success);
    CHECK(mode_result.message == "set mode verified");

    connection.acknowledgement = nomad::mavlink::CommandAck{22, 0};
    const auto takeoff_result = vehicle.takeoff(10.0F);
    CHECK(takeoff_result.success);
    CHECK(takeoff_result.message == "takeoff verified");
}

void test_quadplane_vtol_takeoff_runs_guided_arm_and_climb() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_takeoff_state(connection, 2.0F);
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.vtol_takeoff(5.0F);

    CHECK(result.success);
    CHECK(result.message == "vtol takeoff verified");
    CHECK(connection.command_history.size() == 3);
    CHECK(connection.command_history[0].id == 176);
    CHECK(connection.command_history[0].parameters[1] == 15.0F);
    CHECK(connection.command_history[1].id == 400);
    CHECK(connection.command_history[1].parameters[0] == 1.0F);
    CHECK(connection.command_history[2].id == 22);
    CHECK(connection.command_history[2].parameters[6] == 5.0F);
    CHECK(connection.state->armed);
    CHECK(connection.state->custom_mode == 15);
    CHECK(connection.state->position.relative_altitude_m == 7.0F);
}

void test_quadplane_vtol_takeoff_does_not_treat_ack_as_completion() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_takeoff_state(connection);
    connection.disarm_on_takeoff = true;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.vtol_takeoff(5.0F);

    CHECK(!result.success);
    CHECK(result.message == "vtol takeoff verification failed: vehicle disarmed");
    CHECK(connection.command_history.size() == 3);
    CHECK(connection.command_history.back().id == 22);
}

void test_quadplane_vtol_takeoff_rejects_partial_climb() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_takeoff_state(connection);
    connection.takeoff_altitude_override = 4.0F;
    nomad::vehicle::VehicleConfig config{};
    config.timeouts.vtol_takeoff_state = std::chrono::milliseconds(20);
    nomad::vehicle::Vehicle vehicle(connection, config);

    const auto result = vehicle.vtol_takeoff(5.0F);

    CHECK(!result.success);
    CHECK(result.message == "vtol takeoff acknowledgement received but climb verification timed out");
    CHECK(connection.command_history.size() == 3);
    CHECK(connection.command_history.back().id == 22);
}

void test_guided_mode_uses_its_semantic_operation() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{176, 0};

    const auto result = vehicle.set_guided_mode();

    CHECK(result.success);
    CHECK(result.message == "set guided mode verified");
    CHECK(connection.last_command.id == 176);
    CHECK(connection.last_command.parameters[1] == 4.0F);
}

void test_land_and_rtl_are_verified() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);

    connection.acknowledgement = nomad::mavlink::CommandAck{21, 0};
    CHECK(vehicle.land().success);
    connection.acknowledgement = nomad::mavlink::CommandAck{20, 0};
    CHECK(vehicle.return_to_launch().success);
}

void test_plane_commands_are_rejected_before_transmission() {
    FakeConnection connection;
    connection.connect();
    connection.state->identity =
        nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                            nomad::telemetry::kFixedWing);
    nomad::vehicle::Vehicle vehicle(connection);

    CHECK(!vehicle.arm().success);
    CHECK(!vehicle.disarm().success);
    CHECK(!vehicle.set_guided_mode().success);
    CHECK(!vehicle.set_mode(15).success);
    const auto takeoff_result = vehicle.takeoff(5.0F);
    CHECK(!takeoff_result.success);
    CHECK(takeoff_result.message == "takeoff is not qualified for Plane");
    const auto goto_result = vehicle.goto_location({45.5, -73.6, 5.0F});
    CHECK(!goto_result.success);
    CHECK(goto_result.message == "goto location is not qualified for Plane");
    CHECK(!vehicle.land().success);
    CHECK(!vehicle.return_to_launch().success);
    const auto vtol_takeoff_result = vehicle.vtol_takeoff(5.0F);
    CHECK(!vtol_takeoff_result.success);
    CHECK(vtol_takeoff_result.message == "vtol takeoff is not qualified for Plane");
    CHECK(!vehicle.transition_to_fixed_wing().success);
    CHECK(connection.command_history.empty());
    CHECK(!connection.last_goto.has_value());
}

void test_quadplane_generic_takeoff_is_rejected_before_transmission() {
    FakeConnection connection;
    connection.connect();
    connection.state->identity =
        nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                            nomad::telemetry::kVtolQuadrotor);
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.takeoff(5.0F);

    CHECK(!result.success);
    CHECK(result.message == "takeoff is not qualified for QuadPlane");
    CHECK(connection.command_history.empty());
}

void test_quadplane_vtol_takeoff_rejects_stale_position_before_transmission() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_takeoff_state(connection);
    connection.state->position_updated_at = std::chrono::steady_clock::now() - std::chrono::seconds(3);
    connection.auto_stamp_fresh_fields = false;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.vtol_takeoff(5.0F);

    CHECK(!result.success);
    CHECK(result.message == "vtol takeoff requires a fresh position");
    CHECK(connection.command_history.empty());
}

void test_quadplane_flight_commands_are_rejected_before_transmission() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_takeoff_state(connection);
    nomad::vehicle::Vehicle vehicle(connection);

    CHECK(!vehicle.disarm().success);
    CHECK(vehicle.set_guided_mode().success);
    CHECK(vehicle.arm().success);
    CHECK(!vehicle.set_mode(15).success);
    const auto takeoff_result = vehicle.takeoff(5.0F);
    CHECK(!takeoff_result.success);
    CHECK(takeoff_result.message == "takeoff is not qualified for QuadPlane");
    const auto goto_result = vehicle.goto_location({45.5, -73.6, 5.0F});
    CHECK(!goto_result.success);
    CHECK(goto_result.message == "goto location is not qualified for QuadPlane");
    CHECK(!vehicle.land().success);
    CHECK(!vehicle.return_to_launch().success);
    const auto vtol_result = vehicle.vtol_takeoff(5.0F);
    CHECK(vtol_result.success);
    CHECK(connection.command_history.size() == 4);
    CHECK(connection.command_history.back().id == 22);
    CHECK(!connection.last_goto.has_value());
}

void test_quadplane_vtol_takeoff_rechecks_state_after_preparation() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_takeoff_state(connection);
    connection.stale_position_after_arm = true;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.vtol_takeoff(5.0F);

    CHECK(!result.success);
    CHECK(result.message == "position feed is stale");
    CHECK(connection.command_history.size() == 2);
    CHECK(connection.command_history.back().id == 400);
}

void test_quadplane_vtol_takeoff_rejects_stale_heartbeat_after_preparation() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_takeoff_state(connection);
    connection.stale_heartbeat_after_arm = true;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.vtol_takeoff(5.0F);

    CHECK(!result.success);
    CHECK(result.message == "heartbeat is stale");
    CHECK(connection.command_history.size() == 2);
    CHECK(connection.command_history.back().id == 400);
}

void test_quadplane_vtol_takeoff_rejects_invalid_gps_after_preparation() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_takeoff_state(connection);
    connection.invalidate_gps_after_arm = true;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.vtol_takeoff(5.0F);

    CHECK(!result.success);
    CHECK(result.message == "a valid 3D GPS fix is required");
    CHECK(connection.command_history.size() == 2);
    CHECK(connection.command_history.back().id == 400);
}

void test_unknown_aircraft_rejects_aircraft_specific_commands() {
    FakeConnection connection;
    connection.connect();
    connection.state->identity = {};
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{176, 0};

    CHECK(!vehicle.set_guided_mode().success);
    CHECK(!vehicle.arm().success);
    CHECK(!vehicle.disarm().success);
    CHECK(!vehicle.set_mode(4).success);
    CHECK(!vehicle.takeoff(5.0F).success);
    CHECK(!vehicle.goto_location({45.5, -73.6, 5.0F}).success);
    CHECK(!vehicle.land().success);
    CHECK(!vehicle.return_to_launch().success);
    CHECK(!vehicle.vtol_takeoff(5.0F).success);
    CHECK(!vehicle.transition_to_fixed_wing().success);
    CHECK(connection.command_history.empty());
    CHECK(!connection.last_goto.has_value());
}

void test_disarm_is_verified() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{400, 0};

    const auto result = vehicle.disarm();

    CHECK(result.success);
    CHECK(!connection.state->armed);
}

void test_goto_location_validates_and_verifies() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{192, 0};

    const auto result = vehicle.goto_location({45.500000123, -73.600000123, 10.0F});

    CHECK(result.success);
    CHECK(connection.last_command.id == 0);
    CHECK(connection.last_goto.has_value());
    CHECK(connection.last_goto->latitude_deg == 45.500000123);
    CHECK(connection.last_goto->longitude_deg == -73.600000123);
    CHECK(connection.last_goto->relative_altitude_m == 10.0F);
}

void test_goto_location_rejects_invalid_coordinates() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{192, 0};

    const auto result = vehicle.goto_location({91.0, 0.0, 10.0F});

    CHECK(!result.success);
    CHECK(connection.last_command.id == 0);
    CHECK(!connection.last_goto.has_value());
}

void test_goto_location_rejects_failed_action() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{192, 2};

    const auto result = vehicle.goto_location({45.5, -73.6, 10.0F});

    CHECK(!result.success);
    CHECK(connection.last_goto.has_value());
    CHECK(connection.last_command.id == 0);
}

void test_goto_location_requires_connection() {
    FakeConnection connection;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.goto_location({45.5, -73.6, 10.0F});

    CHECK(!result.success);
    CHECK(!connection.last_goto.has_value());
}

void test_command_rejects_failed_acknowledgement() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{21, 4};

    const auto result = vehicle.land();

    CHECK(!result.success);
}

void test_command_requires_connection() {
    FakeConnection connection;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.land();

    CHECK(!result.success);
}

void test_mission_executor_runs_steps_and_reports_progress() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    nomad::mission::MissionExecutor executor(vehicle);

    const nomad::mission::Mission mission{
        nomad::mission::Action{"guided"},
        nomad::mission::Action{"arm"},
        nomad::mission::Land{},
        nomad::mission::Action{"wait_disarmed"},
    };
    const auto result = executor.execute(mission);

    CHECK(result.success);
    CHECK(result.completed_steps == 4);
}

void test_mission_executor_rejects_unknown_action() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    nomad::mission::MissionExecutor executor(vehicle);

    const auto result = executor.execute({nomad::mission::Action{"unknown"}});

    CHECK(!result.success);
    CHECK(result.completed_steps == 0);
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_state_is_available_through_vehicle();
        test_state_requires_connection();
        test_arm_sends_arm_command();
        test_takeoff_rejects_invalid_altitude();
        test_mode_and_takeoff_are_verified();
        test_quadplane_vtol_takeoff_runs_guided_arm_and_climb();
        test_quadplane_vtol_takeoff_does_not_treat_ack_as_completion();
        test_quadplane_vtol_takeoff_rejects_partial_climb();
        test_guided_mode_uses_its_semantic_operation();
        test_land_and_rtl_are_verified();
        test_plane_commands_are_rejected_before_transmission();
        test_quadplane_generic_takeoff_is_rejected_before_transmission();
        test_quadplane_vtol_takeoff_rejects_stale_position_before_transmission();
        test_quadplane_flight_commands_are_rejected_before_transmission();
        test_quadplane_vtol_takeoff_rechecks_state_after_preparation();
        test_quadplane_vtol_takeoff_rejects_stale_heartbeat_after_preparation();
        test_quadplane_vtol_takeoff_rejects_invalid_gps_after_preparation();
        test_unknown_aircraft_rejects_aircraft_specific_commands();
        test_disarm_is_verified();
        test_goto_location_validates_and_verifies();
        test_goto_location_rejects_invalid_coordinates();
        test_goto_location_rejects_failed_action();
        test_goto_location_requires_connection();
        test_command_rejects_failed_acknowledgement();
        test_command_requires_connection();
        test_mission_executor_runs_steps_and_reports_progress();
        test_mission_executor_rejects_unknown_action();
    });
}
