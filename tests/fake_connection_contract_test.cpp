// SPDX-License-Identifier: Apache-2.0
#include "fake_connection.hpp"
#include "fixed_wing_waypoint_fake_connection.hpp"
#include "nomad/mavlink/connection.hpp"
#include "test_harness.hpp"
#include "vehicle_state_builder.hpp"

#include <chrono>
#include <cstdint>
#include <optional>

namespace {

using Clock = std::chrono::steady_clock;
using nomad::mavlink::Command;
using nomad::telemetry::LandedState;
using nomad::telemetry::VtolState;
using nomad::test::VehicleStateBuilder;

Command arm_command() {
    Command command{};
    command.id = 400;
    command.parameters[0] = 1.0F;
    return command;
}

void test_auto_stamping_refreshes_only_valid_samples() {
    FakeConnection connection;
    const auto old_time = Clock::time_point{} + std::chrono::seconds(10);
    auto &state = *connection.state;
    state.position_valid = true;
    state.position_updated_at = old_time;
    state.velocity_updated_at = old_time;
    state.battery_valid = false;
    state.battery_updated_at = old_time;
    state.gps_valid = true;
    state.gps_updated_at = old_time;
    state.attitude_valid = true;
    state.attitude_updated_at = old_time;
    state.vtol_state_valid = true;
    state.vtol_state_updated_at = old_time;
    state.landed_state_valid = false;
    state.landed_state = LandedState::OnGround;
    state.landed_state_updated_at = old_time;

    const auto sample = connection.wait_for_state(std::chrono::milliseconds::zero());

    CHECK(sample.has_value());
    CHECK(sample->position_updated_at > old_time);
    CHECK(sample->velocity_updated_at > old_time);
    CHECK(sample->battery_updated_at == old_time);
    CHECK(sample->gps_updated_at > old_time);
    CHECK(sample->attitude_updated_at > old_time);
    CHECK(sample->vtol_state_updated_at > old_time);
    CHECK(sample->landed_state_updated_at == old_time);
}

void test_disabling_auto_stamp_preserves_zero_stale_future_and_repeated_times() {
    FakeConnection connection;
    connection.auto_stamp_fresh_fields = false;
    const auto reference = Clock::now();
    const auto stale_time = reference - std::chrono::hours(1);
    const auto future_time = reference + std::chrono::hours(1);
    auto &state = *connection.state;
    state.position_valid = true;
    state.position_updated_at = Clock::time_point{};
    state.gps_valid = true;
    state.gps_updated_at = stale_time;
    state.vtol_state_valid = true;
    state.vtol_state_updated_at = future_time;
    state.attitude_valid = true;
    state.attitude_updated_at = reference;

    const auto first_poll = connection.wait_for_state(std::chrono::milliseconds::zero());
    const auto second_poll = connection.wait_for_state(std::chrono::milliseconds::zero());

    CHECK(first_poll.has_value());
    CHECK(second_poll.has_value());
    CHECK(second_poll->position_updated_at == Clock::time_point{});
    CHECK(second_poll->gps_updated_at == stale_time);
    CHECK(second_poll->vtol_state_updated_at == future_time);
    CHECK(first_poll->attitude_updated_at == reference);
    CHECK(second_poll->attitude_updated_at == reference);
}

void test_send_command_mutates_and_captures_before_transport_or_ack_result() {
    const std::optional<nomad::mavlink::CommandAck> acknowledgements[]{
        nomad::mavlink::CommandAck{1234, 0},
        nomad::mavlink::CommandAck{400, 4},
        std::nullopt,
    };
    for (const auto &acknowledgement : acknowledgements) {
        FakeConnection connection;
        connection.acknowledgement = acknowledgement;

        const auto result = connection.send_command(arm_command(), std::chrono::milliseconds::zero());

        CHECK(connection.command_count() == 1);
        CHECK(connection.command_history.size() == 1);
        CHECK(connection.last_command.id == 400);
        CHECK(connection.command_history.front().parameters[0] == 1.0F);
        CHECK(connection.state->armed);
        if (!acknowledgement.has_value()) {
            CHECK(!result.has_value());
        } else {
            CHECK(result.has_value());
            CHECK(result->command == 400);
            CHECK(result->result == acknowledgement->result);
        }
    }

    FakeConnection transport_failure;
    transport_failure.command_send_results = {false};
    const auto result = transport_failure.send_command(arm_command(), std::chrono::milliseconds::zero());
    CHECK(!result.has_value());
    CHECK(transport_failure.command_count() == 1);
    CHECK(transport_failure.state->armed);
}

void test_expected_session_overload_and_new_fixture_reset_behavior() {
    FakeConnection connection;
    connection.state->session_id = 7;
    nomad::mavlink::MavlinkConnection &transport = connection;
    const auto command = arm_command();

    const auto wrong_session = transport.send_command(command, 6, std::chrono::milliseconds::zero());
    CHECK(!wrong_session.has_value());
    CHECK(connection.command_count() == 0);
    CHECK(!connection.state->armed);

    const auto matching_session = transport.send_command(command, 7, std::chrono::milliseconds::zero());
    CHECK(matching_session.has_value());
    CHECK(matching_session->command == command.id);
    CHECK(connection.command_count() == 1);

    FakeConnection reset_connection;
    CHECK(reset_connection.command_count() == 0);
    CHECK(reset_connection.command_history.empty());
    CHECK(reset_connection.last_command.id == 0);
}

void test_waypoint_poll_completes_transition_then_waypoint_then_sample() {
    FixedWingWaypointFakeConnection connection;
    connection.set_connected(true);
    connection.state->session_id = 5;
    connection.state->vtol_state = VtolState::TransitionToFixedWing;
    connection.state->vtol_state_valid = true;
    connection.complete_transition_on_command = false;
    connection.complete_transition_after_ack_on_poll = true;

    Command transition{};
    transition.id = 3000;
    transition.parameters[0] = 4.0F;
    const auto transition_ack = connection.send_command(transition, std::chrono::milliseconds::zero());
    CHECK(transition_ack.has_value());

    const nomad::mavlink::FixedWingWaypointCommand waypoint{45.1, -73.2, 30.0F, 25.0F};
    connection.fixed_wing_waypoint_samples = {{45.2, -73.3, 31.0F, 26.0F}};
    const auto waypoint_ack = connection.send_fixed_wing_waypoint(waypoint, 5, std::chrono::milliseconds::zero());
    CHECK(waypoint_ack.has_value());

    const auto state = connection.wait_for_state(std::chrono::milliseconds::zero());

    CHECK(state.has_value());
    CHECK(state->vtol_state == VtolState::FixedWing);
    CHECK(state->position.latitude_deg == 45.2);
    CHECK(state->position.longitude_deg == -73.3);
    CHECK(state->position.altitude_m == 31.0F);
    CHECK(state->position.relative_altitude_m == 26.0F);
}

void test_waypoint_expected_session_and_ack_ids_are_returned_as_configured() {
    FixedWingWaypointFakeConnection wrong_session;
    wrong_session.set_connected(true);
    wrong_session.state->session_id = 5;
    const nomad::mavlink::FixedWingWaypointCommand waypoint{45.1, -73.2, 30.0F, 25.0F};

    const auto refused = wrong_session.send_fixed_wing_waypoint(
        waypoint, 4, std::chrono::milliseconds::zero());
    CHECK(!refused.has_value());
    CHECK(wrong_session.fixed_wing_waypoint_send_count == 0);

    FixedWingWaypointFakeConnection wrong_ack;
    wrong_ack.set_connected(true);
    wrong_ack.state->session_id = 5;
    wrong_ack.fixed_wing_waypoint_ack = nomad::mavlink::CommandAck{191, 0};
    const auto accepted_by_result = wrong_ack.send_fixed_wing_waypoint(
        waypoint, 5, std::chrono::milliseconds::zero());
    CHECK(accepted_by_result.has_value());
    CHECK(accepted_by_result->command == 191);
    CHECK(wrong_ack.fixed_wing_waypoint_requests.size() == 1);
    CHECK(wrong_ack.fixed_wing_waypoint_requests.front().latitude_deg == waypoint.latitude_deg);

    FixedWingWaypointFakeConnection session_turnover;
    session_turnover.set_connected(true);
    session_turnover.state->session_id = 5;
    session_turnover.fixed_wing_waypoint_session_change_before_send = true;
    const auto lost_session = session_turnover.send_fixed_wing_waypoint(
        waypoint, 5, std::chrono::milliseconds::zero());
    CHECK(!lost_session.has_value());
    CHECK(session_turnover.state->session_id == 6);
    CHECK(session_turnover.fixed_wing_waypoint_send_count == 0);
    CHECK(session_turnover.fixed_wing_waypoint_requests.empty());
}

void test_state_builder_keeps_omitted_samples_invalid_and_timestamps_explicit() {
    VehicleStateBuilder builder;
    const auto empty = builder.build();
    CHECK(!empty.connected);
    CHECK(!empty.heartbeat_fresh);
    CHECK(!empty.position_valid);
    CHECK(!empty.gps_valid);
    CHECK(!empty.vtol_state_valid);
    CHECK(empty.position_updated_at == Clock::time_point{});
    CHECK(empty.gps_updated_at == Clock::time_point{});
    CHECK(empty.vtol_state_updated_at == Clock::time_point{});

    const auto reference = Clock::now();
    const auto stale_time = reference - std::chrono::hours(1);
    const auto future_time = reference + std::chrono::hours(1);
    builder.set_identity({nomad::telemetry::kArduPilotAutopilot, nomad::telemetry::kFixedWing,
                          nomad::telemetry::AircraftClass::QuadPlane});
    builder.set_link_state(true, true);
    builder.set_armed(true);
    builder.set_mode(10);
    builder.set_session(1, 1, 9);
    builder.set_position({45.0, -73.0, 30.0F, 10.0F}, Clock::time_point{});
    builder.set_gps({3, 12}, stale_time);
    builder.set_vtol_state(VtolState::FixedWing, future_time);

    const auto ready = builder.build();
    CHECK(ready.connected);
    CHECK(ready.heartbeat_fresh);
    CHECK(ready.armed);
    CHECK(ready.custom_mode == 10);
    CHECK(ready.identity.vehicle_type == nomad::telemetry::kFixedWing);
    CHECK(ready.identity.aircraft_class == nomad::telemetry::AircraftClass::QuadPlane);
    CHECK(ready.position_valid);
    CHECK(ready.gps_valid);
    CHECK(ready.vtol_state_valid);
    CHECK(!ready.battery_valid);
    CHECK(!ready.attitude_valid);
    CHECK(!ready.landed_state_valid);
    CHECK(ready.position_updated_at == Clock::time_point{});
    CHECK(ready.gps_updated_at == stale_time);
    CHECK(ready.vtol_state_updated_at == future_time);
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_auto_stamping_refreshes_only_valid_samples();
        test_disabling_auto_stamp_preserves_zero_stale_future_and_repeated_times();
        test_send_command_mutates_and_captures_before_transport_or_ack_result();
        test_expected_session_overload_and_new_fixture_reset_behavior();
        test_waypoint_poll_completes_transition_then_waypoint_then_sample();
        test_waypoint_expected_session_and_ack_ids_are_returned_as_configured();
        test_state_builder_keeps_omitted_samples_invalid_and_timestamps_explicit();
    });
}
