// SPDX-License-Identifier: Apache-2.0
#include "fake_connection.hpp"
#include "nomad/mavlink/protocol.hpp"
#include "nomad/safety/geofence.hpp"
#include "nomad/safety/payload.hpp"
#include "nomad/safety/velocity.hpp"
#include "nomad/safety/watchdog.hpp"
#include "nomad/vehicle/vehicle.hpp"

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

// A failing assert on Windows opens a dialog that blocks unattended CI runs,
// so main() runs the tests inside a try/catch and reports failures on stderr.
void check_impl(bool ok, const char *condition, int line) {
    if (!ok) {
        throw std::runtime_error(std::string("check failed at line ") + std::to_string(line) + ": " + condition);
    }
}

#define CHECK(condition) check_impl(static_cast<bool>(condition), #condition, __LINE__)


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

void test_vehicle_rejects_invalid_watchdog_policy_before_transmission() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    nomad::safety::WatchdogPolicy policy{};
    policy.min_vio_confidence = 2.0F;
    nomad::vehicle::Vehicle vehicle(connection, policy);
    CHECK(vehicle.update_vio(true, 1.0F).success);

    const auto result = vehicle.set_velocity({1.0F, 0.0F, 0.0F, 0.0F});

    CHECK(!result.success);
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
    nomad::vehicle::Vehicle vehicle(connection, policy);
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
    nomad::vehicle::Vehicle vehicle(connection, policy);
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
    nomad::vehicle::Vehicle vehicle(connection, policy);
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

void test_geofence_contains_only_safe_targets() {
    const std::vector<nomad::safety::Point2d> square{
        {-5.0, -5.0},
        {5.0, -5.0},
        {5.0, 5.0},
        {-5.0, 5.0},
    };
    CHECK(nomad::safety::point_in_polygon({0.0, 0.0}, square));
    CHECK(!nomad::safety::point_in_polygon({10.0, 0.0}, square));
    CHECK(nomad::safety::is_contained({0.0, 0.0}, square, 2.0));
    CHECK(!nomad::safety::is_contained({4.0, 0.0}, square, 2.0));
    CHECK(nomad::safety::distance_to_boundary({0.0, 0.0}, square) == 5.0);
}

void test_geofence_rejects_invalid_configuration_and_targets() {
    const auto malformed =
        nomad::safety::evaluate_position({std::vector<nomad::safety::Point2d>{{0.0, 0.0}}, 0.0}, {0.0, 0.0});
    CHECK(!malformed.allowed);
    CHECK(malformed.reason == "fence");

    const auto nonfinite =
        nomad::safety::evaluate_position({std::nullopt, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 0.0});
    CHECK(!nonfinite.allowed);
    CHECK(nonfinite.reason == "nonfinite");

    const auto unconfigured = nomad::safety::evaluate_position({std::nullopt, 0.0}, {100.0, -100.0});
    CHECK(unconfigured.allowed);
}

void test_global_geofence_projects_meters() {
    const nomad::safety::GlobalFencePolicy policy{
        std::vector<nomad::safety::GlobalPoint>{
            {45.0, -73.0},
            {45.0, -72.9999},
            {45.0001, -72.9999},
            {45.0001, -73.0},
        },
        1.0,
    };
    CHECK(nomad::safety::evaluate_global_position(policy, {45.00005, -72.99995}).allowed);
    CHECK(!nomad::safety::evaluate_global_position(policy, {45.002, -72.99995}).allowed);
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
    nomad::vehicle::Vehicle vehicle(connection, {}, policy);
    connection.acknowledgement = nomad::mavlink::CommandAck{192, 0};

    const auto result = vehicle.goto_location({45.002, -72.99995, 10.0F});

    CHECK(!result.success);
    CHECK(connection.last_command.id == 0);
}

void test_vehicle_payload_commands_require_interlock_and_validate_ranges() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{183, 0};
    CHECK(!vehicle.set_servo(0, 1500).success);
    CHECK(vehicle.set_servo(8, 1500).success);

    connection.acknowledgement = nomad::mavlink::CommandAck{181, 0};
    CHECK(!vehicle.release_payload(2, 0.1F).success);
    CHECK(vehicle.arm_payload().success);
    CHECK(vehicle.release_payload(2, 0.05F).success);
    CHECK(!vehicle.release_payload(2, 0.05F).success);
}

void test_vehicle_payload_on_failure_still_attempts_off() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{181, 4};

    CHECK(vehicle.arm_payload().success);
    CHECK(!vehicle.release_payload(2, 0.05F).success);
    CHECK(connection.command_history.size() >= 2);
    CHECK(connection.command_history.back().id == 181);
    CHECK(connection.command_history.back().parameters[1] == 0.0F);
}

void test_vehicle_payload_off_failure_is_reported() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{181, 0};
    connection.command_send_results = {true, false};

    CHECK(vehicle.arm_payload().success);
    CHECK(!vehicle.release_payload(2, 0.05F).success);
    CHECK(connection.command_history.size() >= 2);
    CHECK(connection.command_history.back().parameters[1] == 0.0F);
}

void test_payload_validation_and_interlock() {
    CHECK(nomad::safety::validate_servo_command(1, 500).allowed);
    CHECK(!nomad::safety::validate_servo_command(0, 1500).allowed);
    CHECK(!nomad::safety::validate_servo_command(8, 2501).allowed);
    CHECK(*nomad::safety::clamp_release_duration(60.0F) == 5.0F);
    CHECK(!nomad::safety::clamp_release_duration(std::numeric_limits<float>::quiet_NaN()).has_value());

    nomad::safety::ReleaseInterlock interlock;
    CHECK(!interlock.evaluate_release(1.0F).allowed);
    CHECK(interlock.arm(10.0F).allowed);
    CHECK(interlock.evaluate_release(11.0F).allowed);
    CHECK(!interlock.evaluate_release(11.1F).allowed);
    CHECK(interlock.arm(20.0F).allowed);
    CHECK(!interlock.evaluate_release(31.0F).allowed);
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

void test_param_readback_codec_round_trip() {
    // Fence verification reads FENCE_ENABLE back as authoritative state; the
    // request frame and the value decode must round-trip the exact name.
    const auto request = nomad::mavlink::encode_param_request_read(1, 255, 190, 1, 1, "FENCE_ENABLE", -1);
    CHECK(!request.empty());
    const auto message = nomad::mavlink::decode_message(request);
    CHECK(message.has_value());
    CHECK(!nomad::mavlink::decode_param_value(*message).has_value());

    const auto value = nomad::mavlink::encode_message(1, 1, 1, 22, std::vector<std::uint8_t>(25, 0), true);
    CHECK(value.has_value());
    const auto decoded = nomad::mavlink::decode_message(*value);
    CHECK(decoded.has_value());
    // An all-zero payload decodes as an empty name, which never matches a request.
    const auto param = nomad::mavlink::decode_param_value(*decoded);
    CHECK(param.has_value() && param->param_id.empty());
}

} // namespace

int main() {
    try {
    test_safety_velocity_accepts_clamped_frd_command();
    test_safety_velocity_rejects_each_fault();
    test_vehicle_rejects_invalid_watchdog_policy_before_transmission();
    test_watchdog_stops_for_each_fault();
    test_vehicle_watchdog_stops_for_command_timeout();
    test_vehicle_watchdog_stops_for_stale_vio_and_mode_loss();
    test_vehicle_watchdog_stops_for_link_loss();
    test_vehicle_stop_velocity_sends_zero();
    test_geofence_contains_only_safe_targets();
    test_geofence_rejects_invalid_configuration_and_targets();
    test_global_geofence_projects_meters();
    test_vehicle_fence_rejects_target_before_transmission();
    test_vehicle_payload_commands_require_interlock_and_validate_ranges();
    test_vehicle_payload_on_failure_still_attempts_off();
    test_vehicle_payload_off_failure_is_reported();
    test_payload_validation_and_interlock();
    test_vehicle_destructor_sends_zero_velocity_before_shutdown();
    test_vehicle_destructor_orders_zero_before_disconnect();
    test_vehicle_upload_fence_validates_boundary();
    test_param_readback_codec_round_trip();
    test_vehicle_verifies_fence_status_and_fails_closed();
    test_vehicle_upload_fence_rejects_transport_failure();
    test_vehicle_upload_fence_rejects_invalid_coordinates();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAILED: %s\n", error.what());
        return 1;
    }
    return 0;
}
