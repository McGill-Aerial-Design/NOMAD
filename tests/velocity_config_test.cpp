// SPDX-License-Identifier: Apache-2.0
// Velocity configuration tests (SR-VEL-01/02): unset values use the reviewed
// envelope, valid overrides are loaded, and malformed values fail closed.
#include "fake_connection.hpp"
#include "nomad/safety/velocity_config.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "test_harness.hpp"

#include <cstdio>
#include <stdexcept>
#include <string>

namespace {

nomad::safety::FlightConditions healthy_conditions() {
    return {true, true, true, nomad::safety::kGuidedMode, true, true, 1.0F, 0.3F};
}

void test_unset_limits_use_reviewed_defaults() {
    const auto limits = nomad::safety::load_velocity_limits(nullptr, "", nullptr);
    const auto reviewed = nomad::safety::reviewed_velocity_limits();

    CHECK(limits.max_velocity_xy == reviewed.max_velocity_xy);
    CHECK(limits.max_velocity_z == reviewed.max_velocity_z);
    CHECK(limits.max_yaw_rate == reviewed.max_yaw_rate);
}

void test_configured_limits_are_loaded() {
    const auto limits = nomad::safety::load_velocity_limits("1.5", "0.75", "0.4");

    CHECK(limits.max_velocity_xy == 1.5F);
    CHECK(limits.max_velocity_z == 0.75F);
    CHECK(limits.max_yaw_rate == 0.4F);

    const auto decision = nomad::safety::evaluate_velocity(
        limits, healthy_conditions(), {3.0F, 0.0F, 2.0F, 1.0F});
    CHECK(decision.allowed);
    CHECK(decision.setpoint->vx == 1.5F);
    CHECK(decision.setpoint->vz == -0.75F);
    CHECK(decision.setpoint->yaw_rate == -0.4F);
}

void test_vehicle_uses_configured_velocity_limits() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    const nomad::safety::VelocityLimits limits{0.5F, 0.25F, 0.2F};
    nomad::vehicle::VehicleConfig config{};
    config.velocity = limits;
    nomad::vehicle::Vehicle vehicle(connection, config);
    CHECK(vehicle.update_vio(true, 1.0F).success);

    const auto result = vehicle.set_velocity({2.0F, -2.0F, 1.0F, 1.0F});

    CHECK(result.success);
    CHECK(connection.last_velocity.vx == 0.5F);
    CHECK(connection.last_velocity.vy == 0.5F);
    CHECK(connection.last_velocity.vz == -0.25F);
    CHECK(connection.last_velocity.yaw_rate == -0.2F);
}

void test_typed_malformed_limits_fail_closed() {
    const auto limits = nomad::safety::load_velocity_limits(-1.0F, 1.0F, 1.0F);

    CHECK(nomad::safety::evaluate_velocity(limits, healthy_conditions(), {}).reason ==
          nomad::safety::RejectReason::invalid_limits);
}

void test_malformed_limits_fail_closed() {
    for (const char *bad : {"garbage", "-1.0", "inf", "nan", "1.0x"}) {
        const auto xy = nomad::safety::load_velocity_limits(bad, nullptr, nullptr);
        const auto z = nomad::safety::load_velocity_limits(nullptr, bad, nullptr);
        const auto yaw = nomad::safety::load_velocity_limits(nullptr, nullptr, bad);

        CHECK(nomad::safety::evaluate_velocity(xy, healthy_conditions(), {}).reason ==
              nomad::safety::RejectReason::invalid_limits);
        CHECK(nomad::safety::evaluate_velocity(z, healthy_conditions(), {}).reason ==
              nomad::safety::RejectReason::invalid_limits);
        CHECK(nomad::safety::evaluate_velocity(yaw, healthy_conditions(), {}).reason ==
              nomad::safety::RejectReason::invalid_limits);
    }
}

}  // namespace

int main() {
    return nomad::test::run_tests([] {
        test_unset_limits_use_reviewed_defaults();
        test_configured_limits_are_loaded();
        test_vehicle_uses_configured_velocity_limits();
        test_typed_malformed_limits_fail_closed();
        test_malformed_limits_fail_closed();
    });
}
