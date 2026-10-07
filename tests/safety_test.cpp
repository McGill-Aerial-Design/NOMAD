// SPDX-License-Identifier: Apache-2.0
#include "support/fake_connection.hpp"
#include "nomad/safety/geofence.hpp"
#include "nomad/safety/output.hpp"
#include "nomad/safety/velocity.hpp"
#include "nomad/safety/watchdog.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "support/test_harness.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <chrono>
#include <limits>
#include <thread>
#include <vector>
#include <stdexcept>
#include <string>
#include <cstdio>

namespace {

void test_safety_velocity_accepts_clamped_frd_command() {
    const nomad::safety::FlightConditions conditions{
        true, true, true, nomad::safety::kGuidedMode, true, true, 1.0F,
    };
    const auto decision = nomad::safety::evaluate_velocity({}, conditions, {3.0F, -3.0F, 2.0F, 2.0F});

    CHECK(decision.allowed);
    CHECK(decision.setpoint.has_value());
    CHECK(decision.setpoint->vx == 2.0F);
    CHECK(decision.setpoint->vy == 2.0F);
    CHECK(decision.setpoint->vz == -1.0F);
    CHECK(decision.setpoint->yaw_rate == -1.0F);
}

void test_safety_velocity_accepts_plane_guided_mode_when_selected() {
    const nomad::safety::FlightConditions conditions{
        true, true, true, 15, true, true, 1.0F, 0.3F, 15,
    };
    const auto decision = nomad::safety::evaluate_velocity({}, conditions, {1.0F, 0.0F, 0.0F, 0.0F});

    CHECK(decision.allowed);
}

void test_safety_velocity_rejects_each_fault() {
    const nomad::safety::VelocityCommand command{1.0F, 0.0F, 0.0F, 0.0F};
    CHECK(nomad::safety::evaluate_velocity({}, {}, command).reason == nomad::safety::RejectReason::link);
    assert(nomad::safety::evaluate_velocity({}, {true, true, false, nomad::safety::kGuidedMode, true, true, 1.0F, 0.3F},
                                            command)
               .reason == nomad::safety::RejectReason::armed);
    assert(nomad::safety::evaluate_velocity({}, {true, true, true, 3, true, true, 1.0F, 0.3F}, command).reason ==
           nomad::safety::RejectReason::mode);
    assert(nomad::safety::evaluate_velocity({}, {true, true, true, nomad::safety::kGuidedMode, false, true, 1.0F, 0.3F},
                                            command)
               .reason == nomad::safety::RejectReason::vio);

    const auto nonfinite =
        nomad::safety::evaluate_velocity({}, {true, true, true, nomad::safety::kGuidedMode, true, true, 1.0F, 0.3F},
                                         {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 0.0F});
    CHECK(nonfinite.reason == nomad::safety::RejectReason::nonfinite);
}

void test_vehicle_rejects_body_velocity_for_unsupported_aircraft() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    nomad::vehicle::Vehicle vehicle(connection);
    CHECK(vehicle.update_vio(true, 1.0F).success);

    connection.state->identity = nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                                                     nomad::telemetry::kFixedWing);
    CHECK(!vehicle.set_velocity({1.0F, 0.0F, 0.0F, 0.0F}).success);
    CHECK(!vehicle.stop_velocity().success);
    CHECK(connection.velocity_send_count == 0);

    connection.state->identity = nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                                                     nomad::telemetry::kVtolQuadrotor);
    CHECK(!vehicle.set_velocity({1.0F, 0.0F, 0.0F, 0.0F}).success);
    CHECK(!vehicle.stop_velocity().success);
    CHECK(connection.velocity_send_count == 0);

    connection.state->identity = {};
    CHECK(!vehicle.set_velocity({1.0F, 0.0F, 0.0F, 0.0F}).success);
    CHECK(!vehicle.stop_velocity().success);
    CHECK(connection.velocity_send_count == 0);
}

void test_watchdog_stops_for_each_fault() {
    const nomad::safety::WatchdogPolicy policy{};
    const nomad::safety::WatchdogInput healthy{
        true, true, true, true, nomad::safety::kGuidedMode, true, true, true, 1.0F,
    };
    CHECK(!nomad::safety::evaluate_watchdog(policy, healthy).stop);
    assert(nomad::safety::evaluate_watchdog(
               policy, {true, true, true, true, nomad::safety::kGuidedMode, false, true, true, 1.0F})
               .reason == nomad::safety::WatchdogReason::command_timeout);
    assert(nomad::safety::evaluate_watchdog(
               policy, {true, true, false, true, nomad::safety::kGuidedMode, true, true, true, 1.0F})
               .reason == nomad::safety::WatchdogReason::heartbeat_stale);
    assert(nomad::safety::evaluate_watchdog(
               policy, {true, true, true, false, nomad::safety::kGuidedMode, true, true, true, 1.0F})
               .reason == nomad::safety::WatchdogReason::disarmed);
    assert(nomad::safety::evaluate_watchdog(policy, {true, true, true, true, 3, true, true, true, 1.0F}).reason ==
           nomad::safety::WatchdogReason::wrong_mode);
    assert(nomad::safety::evaluate_watchdog(
               policy, {true, true, true, true, nomad::safety::kGuidedMode, true, false, false, 1.0F})
               .reason == nomad::safety::WatchdogReason::vio_stale);
}

void test_vehicle_watchdog_stops_for_command_timeout() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    nomad::safety::WatchdogPolicy policy{};
    policy.command_timeout = std::chrono::milliseconds(20);
    policy.poll_interval = std::chrono::milliseconds(5);
    nomad::vehicle::VehicleConfig config{};
    config.watchdog = policy;
    nomad::vehicle::Vehicle vehicle(connection, config);
    CHECK(vehicle.update_vio(true, 1.0F).success);

    CHECK(vehicle.set_velocity({1.0F, 0.0F, 0.0F, 0.0F}).success);
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    CHECK(!vehicle.velocity_control_active());
    CHECK(vehicle.last_velocity_stop_reason() == nomad::safety::WatchdogReason::command_timeout);
    CHECK(connection.last_velocity.vx == 0.0F);
}

void test_vehicle_watchdog_stops_for_stale_vio_and_mode_loss() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    nomad::safety::WatchdogPolicy policy{};
    policy.command_timeout = std::chrono::milliseconds(200);
    policy.vio_timeout = std::chrono::milliseconds(20);
    policy.poll_interval = std::chrono::milliseconds(5);
    nomad::vehicle::VehicleConfig config{};
    config.watchdog = policy;
    nomad::vehicle::Vehicle vehicle(connection, config);
    CHECK(vehicle.update_vio(true, 1.0F).success);
    CHECK(vehicle.set_velocity({1.0F, 0.0F, 0.0F, 0.0F}).success);
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    CHECK(vehicle.last_velocity_stop_reason() == nomad::safety::WatchdogReason::vio_stale);

    CHECK(vehicle.update_vio(true, 1.0F).success);
    CHECK(vehicle.set_velocity({1.0F, 0.0F, 0.0F, 0.0F}).success);
    connection.set_mode(3);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    CHECK(vehicle.last_velocity_stop_reason() == nomad::safety::WatchdogReason::wrong_mode);
}

void test_vehicle_watchdog_stops_for_link_loss() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    nomad::safety::WatchdogPolicy policy{};
    policy.command_timeout = std::chrono::milliseconds(200);
    policy.poll_interval = std::chrono::milliseconds(5);
    nomad::vehicle::VehicleConfig config{};
    config.watchdog = policy;
    nomad::vehicle::Vehicle vehicle(connection, config);
    CHECK(vehicle.update_vio(true, 1.0F).success);
    CHECK(vehicle.set_velocity({1.0F, 0.0F, 0.0F, 0.0F}).success);

    connection.set_connected(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    CHECK(!vehicle.velocity_control_active());
    CHECK(vehicle.last_velocity_stop_reason() == nomad::safety::WatchdogReason::heartbeat_stale);
}

void test_vehicle_stop_velocity_sends_zero() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    nomad::vehicle::Vehicle vehicle(connection);
    CHECK(vehicle.update_vio(true, 1.0F).success);
    CHECK(vehicle.set_velocity({1.0F, 0.0F, 0.0F, 0.0F}).success);

    const auto result = vehicle.stop_velocity();

    CHECK(result.success);
    CHECK(!vehicle.velocity_control_active());
    CHECK(connection.last_velocity.vx == 0.0F);
}

void test_vehicle_fence_rejects_target_before_transmission() {
    FakeConnection connection;
    connection.connect();
    const nomad::safety::GlobalFencePolicy policy{
        std::vector<nomad::safety::GlobalPoint>{
            {45.0, -73.0},
            {45.0, -72.9999},
            {45.0001, -72.9999},
            {45.0001, -73.0},
        },
        1.0,
    };
    nomad::vehicle::VehicleConfig config{};
    config.fence = policy;
    nomad::vehicle::Vehicle vehicle(connection, config);
    connection.acknowledgement = nomad::mavlink::CommandAck{192, 0};

    const auto result = vehicle.goto_location({45.002, -72.99995, 10.0F});

    CHECK(!result.success);
    CHECK(connection.last_command.id == 0);
    CHECK(!connection.last_goto.has_value());
}

void test_vehicle_goto_location_rejects_stale_position() {
    FakeConnection connection;
    connection.connect();
    connection.acknowledgement = nomad::mavlink::CommandAck{192, 0};
    connection.auto_stamp_fresh_fields = false;
    nomad::vehicle::Vehicle vehicle(connection);
    connection.state->position_valid = true;
    connection.state->position.latitude_deg = 45.5;
    connection.state->position.longitude_deg = -73.6;
    connection.state->position.relative_altitude_m = 5.0F;
    connection.state->position_updated_at =
        std::chrono::steady_clock::now() - std::chrono::seconds(10);

    const auto result = vehicle.goto_location({45.5, -73.6, 5.0F});

    CHECK(!result.success);
    CHECK(result.message.find("stale") != std::string::npos);
}

void test_vehicle_takeoff_altitude_rejects_stale_position() {
    FakeConnection connection;
    connection.connect();
    connection.acknowledgement = nomad::mavlink::CommandAck{22, 0};
    connection.auto_stamp_fresh_fields = false;
    nomad::vehicle::Vehicle vehicle(connection);
    connection.state->position_valid = true;
    connection.state->position.relative_altitude_m = 5.0F;
    connection.state->position_updated_at =
        std::chrono::steady_clock::now() - std::chrono::seconds(10);

    const auto result = vehicle.takeoff(5.0F);

    CHECK(!result.success);
    CHECK(result.message.find("stale") != std::string::npos);
}

void test_generic_servo_validation() {
    CHECK(nomad::safety::validate_servo_command(1, 500).allowed);
    CHECK(nomad::safety::validate_servo_command(16, 2500).allowed);
    CHECK(!nomad::safety::validate_servo_command(0, 1500).allowed);
    CHECK(!nomad::safety::validate_servo_command(17, 1500).allowed);
    CHECK(!nomad::safety::validate_servo_command(8, 499).allowed);
    CHECK(!nomad::safety::validate_servo_command(8, 2501).allowed);
}

void test_vehicle_destructor_sends_zero_velocity_before_shutdown() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    {
        nomad::vehicle::Vehicle vehicle(connection);
        CHECK(vehicle.update_vio(true, 1.0F).success);
        CHECK(vehicle.set_velocity({1.0F, 0.0F, 0.0F, 0.0F}).success);
        CHECK(vehicle.velocity_control_active());
    }

    CHECK(connection.last_velocity.vx == 0.0F);
    CHECK(connection.last_velocity.vy == 0.0F);
    CHECK(connection.last_velocity.vz == 0.0F);
    CHECK(connection.last_velocity.yaw_rate == 0.0F);
}

void test_vehicle_destructor_orders_zero_before_disconnect() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    {
        nomad::vehicle::Vehicle vehicle(connection);
        CHECK(vehicle.update_vio(true, 1.0F).success);
        CHECK(vehicle.set_velocity({1.0F, 0.0F, 0.0F, 0.0F}).success);
    }

    CHECK(connection.event_log.size() >= 1);
    CHECK(connection.event_log.front() == "send_velocity");
    for (const auto &event : connection.event_log) {
        CHECK(event != "disconnect");
    }
}

void test_vehicle_upload_fence_validates_boundary() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);

    const auto empty_result = vehicle.upload_fence({});
    CHECK(!empty_result.success);

    const auto two_point_result = vehicle.upload_fence({
        {45.0, -73.0},
        {45.0, -72.9999},
    });
    CHECK(!two_point_result.success);

    const auto valid_result = vehicle.upload_fence({
        {45.0, -73.0},
        {45.0, -72.9999},
        {45.0001, -72.9999},
        {45.0001, -73.0},
    });
    CHECK(valid_result.success);
    CHECK(connection.uploaded_fence_plan.size() == 4);
    CHECK(connection.uploaded_fence_plan.front().sequence == 0);
    CHECK(connection.uploaded_fence_plan.back().sequence == 3);
    CHECK(connection.uploaded_fence_plan.front().command == 5001);
    CHECK(connection.uploaded_fence_plan.front().param1 == 4.0F);
}

void test_vehicle_verifies_fence_status_and_fails_closed() {
    FakeConnection connection;
    connection.connect();
    connection.parameters["FENCE_ENABLE"] = 1.0F;
    nomad::vehicle::Vehicle vehicle(connection);

    connection.uploaded_fence_plan = {
        {{1.0F, 2.0F}, 0, 5001},
        {{3.0F, 4.0F}, 1, 5001},
        {{5.0F, 6.0F}, 2, 5001},
    };
    assert(vehicle.verify_fence_uploaded({
        {1.0, 2.0},
        {3.0, 4.0},
        {5.0, 6.0},
    }).success);

    // A fence that is not enabled must fail verification: ArduPilot ignores
    // an uploaded plan while FENCE_ENABLE is zero.
    connection.parameters["FENCE_ENABLE"] = 0.0F;
    assert(!vehicle.verify_fence_uploaded({
        {1.0, 2.0},
        {3.0, 4.0},
        {5.0, 6.0},
    }).success);
    connection.parameters["FENCE_ENABLE"] = 1.0F;

    // A fence whose readback is unavailable must fail closed.
    connection.fence_plan_download_result = false;
    assert(!vehicle.verify_fence_uploaded({
        {1.0, 2.0},
        {3.0, 4.0},
        {5.0, 6.0},
    }).success);
    connection.fence_plan_download_result = true;

    // A fence whose enable state cannot be read must fail closed.
    connection.parameters.erase("FENCE_ENABLE");
    assert(!vehicle.verify_fence_uploaded({
        {1.0, 2.0},
        {3.0, 4.0},
        {5.0, 6.0},
    }).success);
}

void test_vehicle_upload_fence_rejects_transport_failure() {
    FakeConnection connection;
    connection.connect();
    connection.fence_plan_upload_result = false;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.upload_fence({
        {45.0, -73.0},
        {45.0, -72.9999},
        {45.0001, -72.9999},
    });

    CHECK(!result.success);
    CHECK(connection.uploaded_fence_plan.size() == 3);
}

void test_vehicle_upload_fence_rejects_invalid_coordinates() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);

    const auto nan_result = vehicle.upload_fence({
        {45.0, -73.0},
        {45.0, -72.9999},
        {std::numeric_limits<double>::quiet_NaN(), -72.9999},
    });
    CHECK(!nan_result.success);

    const auto lat_result = vehicle.upload_fence({
        {45.0, -73.0},
        {45.0, -72.9999},
        {91.0, -72.9999},
    });
    CHECK(!lat_result.success);

    const auto lon_result = vehicle.upload_fence({
        {45.0, -73.0},
        {45.0, -72.9999},
        {45.0001, -181.0},
    });
    CHECK(!lon_result.success);
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_safety_velocity_accepts_clamped_frd_command();
        test_safety_velocity_accepts_plane_guided_mode_when_selected();
        test_safety_velocity_rejects_each_fault();
        test_vehicle_rejects_body_velocity_for_unsupported_aircraft();
        test_watchdog_stops_for_each_fault();
        test_vehicle_watchdog_stops_for_command_timeout();
        test_vehicle_watchdog_stops_for_stale_vio_and_mode_loss();
        test_vehicle_watchdog_stops_for_link_loss();
        test_vehicle_stop_velocity_sends_zero();
        test_vehicle_fence_rejects_target_before_transmission();
        test_vehicle_goto_location_rejects_stale_position();
        test_vehicle_takeoff_altitude_rejects_stale_position();
        test_generic_servo_validation();
        test_vehicle_destructor_sends_zero_velocity_before_shutdown();
        test_vehicle_destructor_orders_zero_before_disconnect();
        test_vehicle_upload_fence_validates_boundary();
        test_vehicle_verifies_fence_status_and_fails_closed();
        test_vehicle_upload_fence_rejects_transport_failure();
        test_vehicle_upload_fence_rejects_invalid_coordinates();
    });
}
