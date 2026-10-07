// SPDX-License-Identifier: Apache-2.0
#include "support/fake_connection.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "support/test_harness.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <array>
#include <limits>
#include <stdexcept>
#include <string>
#include <cstdio>

namespace {

void test_vehicle_relay_validates_range_and_sends_on_off() {
    FakeConnection connection;
    connection.connect();
    connection.acknowledgement = nomad::mavlink::CommandAck{181, 0};
    nomad::vehicle::Vehicle vehicle(connection);

    CHECK(!vehicle.set_relay(-1, true).success);
    CHECK(!vehicle.set_relay(16, true).success);
    CHECK(vehicle.set_relay(3, true).success);
    CHECK(connection.last_command.id == 181);
    CHECK(connection.last_command.parameters[0] == 3.0F);
    CHECK(connection.last_command.parameters[1] == 1.0F);

    CHECK(vehicle.set_relay(3, false).success);
    CHECK(connection.last_command.parameters[1] == 0.0F);
}

void test_vehicle_relay_rejection_is_reported() {
    FakeConnection connection;
    connection.connect();
    connection.acknowledgement = nomad::mavlink::CommandAck{181, 4};
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.set_relay(1, true);

    CHECK(!result.success);
}

// 209 is MAV_CMD_DO_MOTOR_TEST in the pinned dialect; the id is part of the
// contract, so it is asserted rather than inferred (see tests/test_command_ids.py).
void test_vehicle_motor_test_validates_and_clamps_timeout() {
    FakeConnection connection;
    connection.connect();
    connection.acknowledgement = nomad::mavlink::CommandAck{209, 0};
    nomad::vehicle::Vehicle vehicle(connection);

    CHECK(!vehicle.motor_test(0, 1000, 1.0F).success);
    CHECK(!vehicle.motor_test(1, 400, 1.0F).success);
    CHECK(!vehicle.motor_test(1, 2600, 1.0F).success);
    CHECK(!vehicle.motor_test(1, 1000, std::numeric_limits<float>::quiet_NaN()).success);

    CHECK(vehicle.motor_test(2, 1200, 5.0F).success);
    CHECK(connection.last_command.id == 209);  // MAV_CMD_DO_MOTOR_TEST
    // MAV_CMD_DO_MOTOR_TEST: instance, throttle type (1 = PWM), throttle value,
    // timeout, motor count, test order, empty.
    CHECK(connection.last_command.parameters[0] == 2.0F);
    CHECK(connection.last_command.parameters[1] == 1.0F);
    CHECK(connection.last_command.parameters[2] == 1200.0F);
    CHECK(connection.last_command.parameters[3] == 3.0F);
    CHECK(connection.last_command.parameters[4] == 1.0F);

    CHECK(vehicle.motor_test(2, 0, 0.01F).success);
    CHECK(connection.last_command.parameters[3] == 0.05F);
}

void test_vehicle_gimbal_configure_validates_mount_mode() {
    FakeConnection connection;
    connection.connect();
    connection.acknowledgement = nomad::mavlink::CommandAck{204, 0};
    nomad::vehicle::Vehicle vehicle(connection);

    CHECK(!vehicle.configure_gimbal(-1).success);
    CHECK(!vehicle.configure_gimbal(5).success);
    CHECK(vehicle.configure_gimbal(2).success);
    CHECK(connection.last_command.id == 204);
    CHECK(connection.last_command.parameters[0] == 2.0F);
    CHECK(connection.last_command.parameters[4] == 2.0F);
}

void test_vehicle_gimbal_target_validates_and_sends_angles() {
    FakeConnection connection;
    connection.connect();
    connection.acknowledgement = nomad::mavlink::CommandAck{205, 0};
    nomad::vehicle::Vehicle vehicle(connection);

    CHECK(!vehicle.set_gimbal_target(std::numeric_limits<double>::quiet_NaN(), 0.0).success);
    CHECK(!vehicle.set_gimbal_target(0.0, std::numeric_limits<double>::infinity()).success);
    CHECK(!vehicle.set_gimbal_target(-90.01, 0.0).success);
    CHECK(!vehicle.set_gimbal_target(0.0, 30.01).success);
    CHECK(connection.command_count() == 0);

    CHECK(vehicle.set_gimbal_target(-90.0, 30.0).success);
    CHECK(connection.last_command.id == 205);
    CHECK(connection.last_command.parameters[0] == -90.0F);
    CHECK(connection.last_command.parameters[1] == 30.0F);
    CHECK(connection.last_command.parameters[2] == 0.0F);
    CHECK(connection.last_command.parameters[6] == 2.0F);

    CHECK(vehicle.set_gimbal_target(90.0, -30.0).success);
    CHECK(connection.last_command.parameters[0] == 90.0F);
    CHECK(connection.last_command.parameters[1] == -30.0F);
}

void test_vehicle_gimbal_target_reports_denied_acknowledgement() {
    FakeConnection connection;
    connection.connect();
    connection.acknowledgement = nomad::mavlink::CommandAck{205, 4};
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.set_gimbal_target(15.0, -5.0);

    CHECK(!result.success);
    CHECK(result.message.find("rejected") != std::string::npos);
    CHECK(connection.last_command.id == 205);
}

void test_unqualified_aircraft_reject_outputs_before_transmission() {
    constexpr std::array unqualified_types{
        nomad::telemetry::kFixedWing,
        nomad::telemetry::kVtolQuadrotor,
        std::uint8_t{0},
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

        CHECK(!vehicle.set_servo(8, 1500).success);
        CHECK(!vehicle.set_relay(2, true).success);
        CHECK(!vehicle.motor_test(1, 1200, 1.0F).success);
        CHECK(!vehicle.configure_gimbal(2).success);
        CHECK(!vehicle.set_gimbal_target(0.0, 0.0).success);
        CHECK(connection.command_history.empty());
    }
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_vehicle_relay_validates_range_and_sends_on_off();
        test_vehicle_relay_rejection_is_reported();
        test_vehicle_motor_test_validates_and_clamps_timeout();
        test_vehicle_gimbal_configure_validates_mount_mode();
        test_vehicle_gimbal_target_validates_and_sends_angles();
        test_vehicle_gimbal_target_reports_denied_acknowledgement();
        test_unqualified_aircraft_reject_outputs_before_transmission();
    });
}
