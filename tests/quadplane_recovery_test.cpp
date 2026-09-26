// SPDX-License-Identifier: Apache-2.0
#include "fake_connection.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "test_harness.hpp"

#include <array>
#include <chrono>
#include <limits>
#include <string>

namespace {

using nomad::telemetry::AircraftClass;
using nomad::telemetry::Position;
using nomad::telemetry::VtolState;
using nomad::vehicle::RecoveryPoint;
using nomad::vehicle::Vehicle;

constexpr RecoveryPoint kRecovery{45.0026, -73.0, 20.0F};

void configure(FakeConnection &connection) {
    connection.connect();
    connection.parameters["Q_GUIDED_MODE"] = 0.0F;
    connection.state->identity = {nomad::telemetry::kArduPilotAutopilot, nomad::telemetry::kFixedWing,
                                  AircraftClass::QuadPlane};
    connection.state->connected = true;
    connection.state->heartbeat_fresh = true;
    connection.state->armed = true;
    connection.state->system_id = 1;
    connection.state->component_id = 1;
    connection.state->custom_mode = 15;
    connection.state->position = {45.0, -73.0, 30.0F, 20.0F};
    connection.state->position_valid = true;
    connection.state->position_updated_at = std::chrono::steady_clock::now();
    connection.state->gps = {3, 12};
    connection.state->gps_valid = true;
    connection.state->gps_updated_at = std::chrono::steady_clock::now();
    connection.state->vtol_state = VtolState::FixedWing;
    connection.state->vtol_state_valid = true;
    connection.state->vtol_state_updated_at = std::chrono::steady_clock::now();
}

Vehicle short_vehicle(FakeConnection &connection) {
    nomad::vehicle::VehicleConfig config{};
    config.timeouts.fixed_wing_recovery = std::chrono::milliseconds(25);
    return Vehicle(connection, config);
}

void test_zero_recovery_timeout_rejects_before_parameter_readback() {
    FakeConnection connection;
    configure(connection);
    nomad::vehicle::VehicleConfig config{};
    config.timeouts.fixed_wing_recovery = std::chrono::milliseconds::zero();
    Vehicle vehicle(connection, config);

    const auto result = vehicle.fixed_wing_recovery(kRecovery);

    CHECK(!result.success);
    CHECK(result.message == "fixed-wing recovery rejected: timeout must be positive");
    CHECK(connection.parameter_read_count == 0);
    CHECK(connection.fixed_wing_waypoint_send_count == 0);
}

void check_no_recovery_send(FakeConnection &connection, const RecoveryPoint &point = kRecovery) {
    Vehicle vehicle(connection);
    CHECK(!vehicle.fixed_wing_recovery(point).success);
    CHECK(connection.fixed_wing_waypoint_send_count == 0);
    CHECK(connection.command_history.empty());
}

void test_capability_and_readiness_rejections() {
    constexpr std::array classes{AircraftClass::Copter, AircraftClass::Plane, AircraftClass::Unknown};
    for (const auto aircraft_class : classes) {
        FakeConnection connection;
        configure(connection);
        connection.state->identity.aircraft_class = aircraft_class;
        check_no_recovery_send(connection);
    }
    for (int failure = 0; failure < 12; ++failure) {
        FakeConnection connection;
        configure(connection);
        const auto stale = std::chrono::steady_clock::now() - std::chrono::seconds(4);
        switch (failure) {
        case 0: connection.state->armed = false; break;
        case 1: connection.state->heartbeat_fresh = false; break;
        case 2: connection.state->position_updated_at = stale; break;
        case 3: connection.state->gps_updated_at = stale; break;
        case 4: connection.state->vtol_state_updated_at = stale; break;
        case 5: connection.state->vtol_state = VtolState::Multicopter; break;
        case 6: connection.state->custom_mode = 10; break;
        case 7: connection.state->session_id = 0; break;
        case 8: connection.state->connected = false; break;
        case 9: connection.state->position_valid = false; break;
        case 10: connection.state->gps_valid = false; break;
        case 11: connection.state->vtol_state_valid = false; break;
        }
        check_no_recovery_send(connection);
    }
}

void test_invalid_point_and_near_target_reject_before_send() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    const std::array invalid{
        RecoveryPoint{}, RecoveryPoint{91.0, -73.0, 20.0F}, RecoveryPoint{45.0, -181.0, 20.0F},
        RecoveryPoint{nan, -73.0, 20.0F}, RecoveryPoint{45.0, infinity, 20.0F},
        RecoveryPoint{45.0, -73.0, -1.0F}, RecoveryPoint{45.0, -73.0, 101.0F},
    };
    for (const auto &point : invalid) {
        FakeConnection connection;
        configure(connection);
        check_no_recovery_send(connection, point);
    }
    FakeConnection connection;
    configure(connection);
    connection.state->position.latitude_deg = 45.00255;
    check_no_recovery_send(connection);
}

void test_guided_vtol_configuration_must_be_read_back() {
    for (int failure = 0; failure < 2; ++failure) {
        FakeConnection connection;
        configure(connection);
        if (failure == 0) {
            connection.parameters.erase("Q_GUIDED_MODE");
        }
        if (failure == 1) {
            connection.parameters["Q_GUIDED_MODE"] = 1.0F;
        }
        check_no_recovery_send(connection);
        CHECK(connection.parameter_read_count == 1);
    }
}

void test_session_change_after_parameter_readback_rejects_before_send() {
    FakeConnection connection;
    configure(connection);
    connection.change_session_after_param_read = true;
    check_no_recovery_send(connection);
    CHECK(connection.parameter_read_count == 1);
}

void test_command_and_authoritative_arrival() {
    FakeConnection connection;
    configure(connection);
    connection.fixed_wing_waypoint_auto_complete = false;
    connection.fixed_wing_waypoint_samples = {
        Position{45.0012, -73.0, 30.0F, 20.0F},
        Position{45.0025, -73.0, 30.0F, 20.0F},
    };
    auto vehicle = short_vehicle(connection);

    const auto result = vehicle.fixed_wing_recovery(kRecovery);

    CHECK(result.success);
    CHECK(result.message == "fixed-wing recovery verified: recovery region reached");
    CHECK(connection.command_history.empty());
    CHECK(connection.fixed_wing_waypoint_send_count == 1);
    CHECK(connection.fixed_wing_waypoint_requests.size() == 1);
    const auto &sent = connection.fixed_wing_waypoint_requests[0];
    CHECK(sent.latitude_deg == kRecovery.latitude_deg);
    CHECK(sent.longitude_deg == kRecovery.longitude_deg);
    CHECK(sent.relative_altitude_m == kRecovery.relative_altitude_m);
    CHECK(sent.loiter_radius_m == 30.0F);
}

void test_command_rejection_and_session_change_at_send() {
    for (int failure = 0; failure < 3; ++failure) {
        FakeConnection connection;
        configure(connection);
        if (failure == 0) {
            connection.fixed_wing_waypoint_transport_enabled = false;
        }
        if (failure == 1) {
            connection.fixed_wing_waypoint_ack = nomad::mavlink::CommandAck{192, 2};
        }
        if (failure == 2) {
            connection.fixed_wing_waypoint_session_change_before_send = true;
        }
        Vehicle vehicle(connection);
        CHECK(!vehicle.fixed_wing_recovery(kRecovery).success);
        CHECK(connection.fixed_wing_waypoint_send_count == (failure == 2 ? 0 : 1));
    }
}

void test_ack_without_real_progress_cannot_complete() {
    for (int failure = 0; failure < 5; ++failure) {
        FakeConnection connection;
        configure(connection);
        connection.fixed_wing_waypoint_auto_complete = false;
        if (failure == 1) {
            connection.fixed_wing_waypoint_samples = {{44.9990, -73.0, 30.0F, 20.0F}};
        }
        if (failure == 2) {
            connection.fixed_wing_waypoint_position_before_ack = Position{45.00215, -73.0, 30.0F, 20.0F};
            connection.fixed_wing_waypoint_samples = {{45.0022, -73.0, 30.0F, 20.0F}};
        }
        if (failure == 3) {
            connection.fixed_wing_waypoint_completion_before_ack = true;
            connection.fixed_wing_waypoint_auto_complete = true;
        }
        if (failure == 4) {
            connection.fixed_wing_waypoint_samples = {{45.0025, -73.0, 30.0F, 28.0F}};
        }
        auto vehicle = short_vehicle(connection);
        const auto result = vehicle.fixed_wing_recovery(kRecovery);
        CHECK(!result.success);
        CHECK(result.message.find("timed out") != std::string::npos);
        CHECK(connection.fixed_wing_waypoint_send_count == 1);
    }
}

void test_post_command_interruption_fails_closed() {
    for (int failure = 0; failure < 10; ++failure) {
        FakeConnection connection;
        configure(connection);
        if (failure == 0) {
            connection.fixed_wing_waypoint_session_change_on_send = true;
        }
        if (failure == 1) {
            connection.fixed_wing_waypoint_link_loss_on_send = true;
        }
        if (failure == 2) {
            connection.fixed_wing_waypoint_mode_loss_on_send = true;
        }
        if (failure == 3) {
            connection.fixed_wing_waypoint_vtol_loss_on_send = true;
        }
        if (failure == 4) {
            connection.fixed_wing_waypoint_stale_position_on_send = true;
        }
        if (failure == 5) {
            connection.fixed_wing_waypoint_stale_gps_on_send = true;
        }
        if (failure == 6) {
            connection.fixed_wing_waypoint_stale_vtol_on_send = true;
        }
        if (failure == 7) {
            connection.fixed_wing_waypoint_disarm_on_send = true;
        }
        if (failure == 8) {
            connection.fixed_wing_waypoint_vtol_mc_on_send = true;
        }
        if (failure == 9) {
            connection.fixed_wing_waypoint_heartbeat_loss_on_send = true;
        }
        auto vehicle = short_vehicle(connection);
        CHECK(!vehicle.fixed_wing_recovery(kRecovery).success);
        CHECK(connection.fixed_wing_waypoint_send_count == 1);
    }
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_capability_and_readiness_rejections();
        test_invalid_point_and_near_target_reject_before_send();
        test_guided_vtol_configuration_must_be_read_back();
        test_session_change_after_parameter_readback_rejects_before_send();
        test_command_and_authoritative_arrival();
        test_zero_recovery_timeout_rejects_before_parameter_readback();
        test_command_rejection_and_session_change_at_send();
        test_ack_without_real_progress_cannot_complete();
        test_post_command_interruption_fails_closed();
    });
}
