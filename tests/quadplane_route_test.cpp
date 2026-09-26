// SPDX-License-Identifier: Apache-2.0
#include "fake_connection.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "test_harness.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using nomad::telemetry::AircraftClass;
using nomad::telemetry::VtolState;
using nomad::vehicle::RouteWaypoint;
using nomad::vehicle::Vehicle;

const std::vector<RouteWaypoint> kRoute{
    {45.0013, -73.0, 20.0F},
    {45.0026, -73.0, 20.0F},
};

void configure_fixed_wing_quadplane(FakeConnection &connection) {
    connection.connect();
    connection.state->identity = {nomad::telemetry::kArduPilotAutopilot, nomad::telemetry::kFixedWing,
                                  AircraftClass::QuadPlane};
    connection.state->connected = true;
    connection.state->heartbeat_fresh = true;
    connection.state->armed = true;
    connection.state->system_id = 1;
    connection.state->component_id = 1;
    connection.state->custom_mode = 10;
    connection.state->position = {45.0, -73.0, 30.0F, 10.0F};
    connection.state->position_valid = true;
    connection.state->position_updated_at = std::chrono::steady_clock::now();
    connection.state->gps = {3, 12};
    connection.state->gps_valid = true;
    connection.state->gps_updated_at = std::chrono::steady_clock::now();
    connection.state->vtol_state = VtolState::FixedWing;
    connection.state->vtol_state_valid = true;
    connection.state->vtol_state_updated_at = std::chrono::steady_clock::now();
}

Vehicle make_short_timeout_vehicle(FakeConnection &connection) {
    nomad::vehicle::VehicleConfig config{};
    config.timeouts.fixed_wing_route = std::chrono::milliseconds(25);
    return Vehicle(connection, config);
}

void test_zero_route_timeout_rejects_before_navigation() {
    FakeConnection connection;
    configure_fixed_wing_quadplane(connection);
    nomad::vehicle::VehicleConfig config{};
    config.timeouts.fixed_wing_route = std::chrono::milliseconds::zero();
    Vehicle vehicle(connection, config);

    const auto result = vehicle.fixed_wing_route(kRoute);

    CHECK(!result.success);
    CHECK(result.message == "fixed-wing route rejected: route timeout must be positive");
    CHECK(connection.fixed_wing_waypoint_send_count == 0);
    CHECK(connection.command_history.empty());
}

void test_fixed_wing_route_sends_two_waypoints_and_verifies_position() {
    FakeConnection connection;
    configure_fixed_wing_quadplane(connection);
    Vehicle vehicle(connection);

    const auto result = vehicle.fixed_wing_route(kRoute);

    CHECK(result.success);
    CHECK(result.message == "fixed-wing route verified: waypoints=2");
    CHECK(connection.fixed_wing_waypoint_send_count == 2);
    CHECK(connection.fixed_wing_waypoint_requests.size() == 2);
    CHECK(connection.fixed_wing_waypoint_requests[0].latitude_deg == kRoute[0].latitude_deg);
    CHECK(connection.fixed_wing_waypoint_requests[0].longitude_deg == kRoute[0].longitude_deg);
    CHECK(connection.fixed_wing_waypoint_requests[0].relative_altitude_m == kRoute[0].relative_altitude_m);
    CHECK(connection.fixed_wing_waypoint_requests[0].loiter_radius_m == 30.0F);
    CHECK(connection.fixed_wing_waypoint_requests[1].latitude_deg == kRoute[1].latitude_deg);
    CHECK(connection.fixed_wing_waypoint_requests[1].longitude_deg == kRoute[1].longitude_deg);
    CHECK(connection.fixed_wing_waypoint_requests[1].relative_altitude_m == kRoute[1].relative_altitude_m);
    CHECK(connection.state->custom_mode == 15);
}

void test_unqualified_aircraft_reject_route_before_navigation_transport() {
    constexpr std::array classes{AircraftClass::Copter, AircraftClass::Plane, AircraftClass::Unknown};
    for (const auto aircraft_class : classes) {
        FakeConnection connection;
        configure_fixed_wing_quadplane(connection);
        connection.state->identity.aircraft_class = aircraft_class;
        Vehicle vehicle(connection);

        CHECK(!vehicle.fixed_wing_route(kRoute).success);
        CHECK(connection.fixed_wing_waypoint_send_count == 0);
        CHECK(connection.command_history.empty());
    }
}

void test_empty_malformed_and_unreasonable_routes_reject_before_transmission() {
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    const auto infinity = std::numeric_limits<double>::infinity();
    const std::vector<std::vector<RouteWaypoint>> invalid_routes{
        {},
        {{45.0013, -73.0, 20.0F}},
        {{91.0, -73.0, 20.0F}, kRoute[1]},
        {{45.0013, -181.0, 20.0F}, kRoute[1]},
        {{nan, -73.0, 20.0F}, kRoute[1]},
        {{45.0013, infinity, 20.0F}, kRoute[1]},
        {{45.0013, -73.0, std::numeric_limits<float>::quiet_NaN()}, kRoute[1]},
        {{45.0013, -73.0, -1.0F}, kRoute[1]},
        {{45.0013, -73.0, 101.0F}, kRoute[1]},
        {{45.0013, -73.0, 20.0F}, {45.00131, -73.0, 20.0F}},
        {kRoute[0], kRoute[1], {45.0039, -73.0, 20.0F}},
        {{45.0013, -73.0, infinity}, kRoute[1]},
    };
    for (const auto &route : invalid_routes) {
        FakeConnection connection;
        configure_fixed_wing_quadplane(connection);
        Vehicle vehicle(connection);

        CHECK(!vehicle.fixed_wing_route(route).success);
        CHECK(connection.fixed_wing_waypoint_send_count == 0);
        CHECK(connection.command_history.empty());
    }
}

void test_stale_position_gps_vtol_state_and_wrong_vtol_state_reject_before_send() {
    for (int failure = 0; failure < 4; ++failure) {
        FakeConnection connection;
        configure_fixed_wing_quadplane(connection);
        connection.auto_stamp_fresh_fields = false;
        if (failure == 0) {
            connection.state->position_updated_at = std::chrono::steady_clock::now() - std::chrono::seconds(3);
        } else if (failure == 1) {
            connection.state->gps_updated_at = std::chrono::steady_clock::now() - std::chrono::seconds(3);
        } else if (failure == 2) {
            connection.state->vtol_state_updated_at = std::chrono::steady_clock::now() - std::chrono::seconds(4);
        } else {
            connection.state->vtol_state = VtolState::Multicopter;
        }
        Vehicle vehicle(connection);

        const auto result = vehicle.fixed_wing_route(kRoute);
        if (result.success) {
            throw std::runtime_error("interruption case unexpectedly succeeded: " + std::to_string(failure));
        }
        CHECK(connection.fixed_wing_waypoint_send_count == 0);
        CHECK(connection.command_history.empty());
    }
}

void test_unarmed_and_non_auto_quadplane_reject_before_transmission() {
    for (int failure = 0; failure < 2; ++failure) {
        FakeConnection connection;
        configure_fixed_wing_quadplane(connection);
        if (failure == 0) {
            connection.state->armed = false;
        } else {
            connection.state->custom_mode = 15;
        }
        Vehicle vehicle(connection);

        CHECK(!vehicle.fixed_wing_route(kRoute).success);
        CHECK(connection.fixed_wing_waypoint_send_count == 0);
        CHECK(connection.command_history.empty());
    }
}

void test_route_rechecks_state_after_qualified_guided_setup() {
    FakeConnection connection;
    configure_fixed_wing_quadplane(connection);
    connection.invalidate_vtol_after_guided_mode = true;
    Vehicle vehicle(connection);

    const auto result = vehicle.fixed_wing_route(kRoute);

    CHECK(!result.success);
    CHECK(result.message.find("after GUIDED setup") != std::string::npos);
    CHECK(connection.command_history.size() == 1);
    CHECK(connection.fixed_wing_waypoint_send_count == 0);
}

void test_transport_rejects_session_change_at_waypoint_send_boundary() {
    FakeConnection connection;
    configure_fixed_wing_quadplane(connection);
    connection.fixed_wing_waypoint_session_change_before_send = true;
    Vehicle vehicle(connection);

    const auto result = vehicle.fixed_wing_route(kRoute);

    CHECK(!result.success);
    CHECK(result.message.find("no command ACK") != std::string::npos);
    CHECK(connection.fixed_wing_waypoint_send_count == 0);
    CHECK(connection.fixed_wing_waypoint_requests.empty());
}

void test_route_start_ack_failure_does_not_claim_completion() {
    FakeConnection connection;
    configure_fixed_wing_quadplane(connection);
    connection.fixed_wing_waypoint_ack = nomad::mavlink::CommandAck{192, 2};
    Vehicle vehicle(connection);

    const auto result = vehicle.fixed_wing_route(kRoute);

    CHECK(!result.success);
    CHECK(result.message.find("rejected by ArduPilot") != std::string::npos);
    CHECK(connection.fixed_wing_waypoint_send_count == 1);
}

void test_missing_ack_fails_without_route_completion() {
    FakeConnection connection;
    configure_fixed_wing_quadplane(connection);
    connection.fixed_wing_waypoint_transport_enabled = false;
    Vehicle vehicle(connection);

    const auto result = vehicle.fixed_wing_route(kRoute);

    CHECK(!result.success);
    CHECK(result.message.find("no command ACK") != std::string::npos);
    CHECK(connection.fixed_wing_waypoint_send_count == 1);
}

void test_ack_without_post_command_position_progress_times_out() {
    FakeConnection connection;
    configure_fixed_wing_quadplane(connection);
    connection.auto_stamp_fresh_fields = false;
    connection.fixed_wing_waypoint_auto_complete = false;
    auto vehicle = make_short_timeout_vehicle(connection);

    const auto result = vehicle.fixed_wing_route(kRoute);

    CHECK(!result.success);
    CHECK(result.message.find("position verification timed out") != std::string::npos);
    CHECK(connection.fixed_wing_waypoint_send_count == 1);
}

void test_position_reached_before_ack_without_post_ack_progress_does_not_complete_route() {
    FakeConnection connection;
    configure_fixed_wing_quadplane(connection);
    connection.fixed_wing_waypoint_completion_before_ack = true;
    auto vehicle = make_short_timeout_vehicle(connection);

    const auto result = vehicle.fixed_wing_route(kRoute);

    CHECK(!result.success);
    CHECK(result.message.find("position verification timed out") != std::string::npos);
    CHECK(connection.fixed_wing_waypoint_send_count == 1);
    CHECK(connection.state->position.latitude_deg == kRoute[0].latitude_deg);
}

void test_first_waypoint_does_not_complete_two_point_route() {
    FakeConnection connection;
    configure_fixed_wing_quadplane(connection);
    connection.auto_stamp_fresh_fields = false;
    connection.fixed_wing_waypoint_completions = {true, false};
    auto vehicle = make_short_timeout_vehicle(connection);

    const auto result = vehicle.fixed_wing_route(kRoute);

    CHECK(!result.success);
    CHECK(result.message.find("position verification timed out") != std::string::npos);
    CHECK(connection.fixed_wing_waypoint_send_count == 2);
    CHECK(connection.state->position.latitude_deg == kRoute[0].latitude_deg);
}

void test_old_final_waypoint_position_cannot_skip_the_first_route_point() {
    FakeConnection connection;
    configure_fixed_wing_quadplane(connection);
    connection.auto_stamp_fresh_fields = false;
    connection.state->position.latitude_deg = kRoute[1].latitude_deg;
    connection.fixed_wing_waypoint_auto_complete = false;
    auto vehicle = make_short_timeout_vehicle(connection);

    const auto result = vehicle.fixed_wing_route(kRoute);

    CHECK(!result.success);
    CHECK(result.message.find("position verification timed out") != std::string::npos);
    CHECK(connection.fixed_wing_waypoint_send_count == 1);
}

void test_session_link_mode_vtol_and_position_interruption_fail_closed() {
    for (int failure = 0; failure < 5; ++failure) {
        FakeConnection connection;
        configure_fixed_wing_quadplane(connection);
        if (failure == 0) {
            connection.fixed_wing_waypoint_session_change_on_send = true;
        } else if (failure == 1) {
            connection.fixed_wing_waypoint_link_loss_on_send = true;
        } else if (failure == 2) {
            connection.fixed_wing_waypoint_mode_loss_on_send = true;
        } else if (failure == 3) {
            connection.fixed_wing_waypoint_vtol_loss_on_send = true;
        } else {
            connection.fixed_wing_waypoint_auto_complete = false;
            connection.fixed_wing_waypoint_stale_position_on_send = true;
        }
        Vehicle vehicle(connection);

        const auto result = vehicle.fixed_wing_route(kRoute);
        if (result.success) {
            throw std::runtime_error("interruption case unexpectedly succeeded: " + std::to_string(failure));
        }
        CHECK(connection.fixed_wing_waypoint_send_count == 1);
    }
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_fixed_wing_route_sends_two_waypoints_and_verifies_position();
        test_zero_route_timeout_rejects_before_navigation();
        test_unqualified_aircraft_reject_route_before_navigation_transport();
        test_empty_malformed_and_unreasonable_routes_reject_before_transmission();
        test_stale_position_gps_vtol_state_and_wrong_vtol_state_reject_before_send();
        test_unarmed_and_non_auto_quadplane_reject_before_transmission();
        test_route_rechecks_state_after_qualified_guided_setup();
        test_transport_rejects_session_change_at_waypoint_send_boundary();
        test_route_start_ack_failure_does_not_claim_completion();
        test_missing_ack_fails_without_route_completion();
        test_ack_without_post_command_position_progress_times_out();
        test_position_reached_before_ack_without_post_ack_progress_does_not_complete_route();
        test_first_waypoint_does_not_complete_two_point_route();
        test_old_final_waypoint_position_cannot_skip_the_first_route_point();
        test_session_link_mode_vtol_and_position_interruption_fail_closed();
    });
}
