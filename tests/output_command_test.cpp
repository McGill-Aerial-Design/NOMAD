// SPDX-License-Identifier: Apache-2.0
#include "fake_connection.hpp"
#include "nomad/vehicle/vehicle.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <limits>
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

void test_vehicle_motor_test_validates_and_clamps_timeout() {
    FakeConnection connection;
    connection.connect();
    connection.acknowledgement = nomad::mavlink::CommandAck{139, 0};
    nomad::vehicle::Vehicle vehicle(connection);

    CHECK(!vehicle.motor_test(0, 1000, 1.0F).success);
    CHECK(!vehicle.motor_test(1, 400, 1.0F).success);
    CHECK(!vehicle.motor_test(1, 2600, 1.0F).success);
    CHECK(!vehicle.motor_test(1, 1000, std::numeric_limits<float>::quiet_NaN()).success);

    CHECK(vehicle.motor_test(2, 1200, 5.0F).success);
    CHECK(connection.last_command.id == 139);
    CHECK(connection.last_command.parameters[0] == 2.0F);
    CHECK(connection.last_command.parameters[1] == 1.0F);
    CHECK(connection.last_command.parameters[2] == 1200.0F);
    CHECK(connection.last_command.parameters[3] == 3.0F);

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

void test_vehicle_user_command_requires_finite_parameters() {
    FakeConnection connection;
    connection.connect();
    connection.acknowledgement = nomad::mavlink::CommandAck{31010, 0};
    nomad::vehicle::Vehicle vehicle(connection);

    const auto nonfinite = vehicle.send_user_command(
        {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, std::numeric_limits<float>::quiet_NaN()});
    CHECK(!nonfinite.success);
    CHECK(connection.command_history.empty());

    CHECK(vehicle.send_user_command({1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F}).success);
    CHECK(connection.last_command.id == 31010);
    CHECK(connection.last_command.parameters[6] == 7.0F);
}

} // namespace

int main() {
    try {
    test_vehicle_relay_validates_range_and_sends_on_off();
    test_vehicle_relay_rejection_is_reported();
    test_vehicle_motor_test_validates_and_clamps_timeout();
    test_vehicle_gimbal_configure_validates_mount_mode();
    test_vehicle_user_command_requires_finite_parameters();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAILED: %s\n", error.what());
        return 1;
    }
    return 0;
}
