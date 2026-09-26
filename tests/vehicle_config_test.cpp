// SPDX-License-Identifier: Apache-2.0
#include "fake_connection.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "test_harness.hpp"

#include <chrono>

namespace {

void test_vehicle_config_keeps_reviewed_defaults() {
    const nomad::vehicle::VehicleConfig config{};

    CHECK(config.watchdog.command_timeout == std::chrono::milliseconds(500));
    CHECK(config.watchdog.vio_timeout == std::chrono::milliseconds(1000));
    CHECK(config.watchdog.poll_interval == std::chrono::milliseconds(50));
    CHECK(config.watchdog.min_vio_confidence == 0.3F);
    CHECK(!config.fence.boundary.has_value());
    CHECK(config.fence.margin_m == 0.0);
    CHECK(config.velocity.max_velocity_xy == 2.0F);
    CHECK(config.velocity.max_velocity_z == 1.0F);
    CHECK(config.velocity.max_yaw_rate == 1.0F);
    CHECK(config.timeouts.position_freshness == std::chrono::milliseconds(2000));
    CHECK(config.timeouts.vtol_takeoff_state == std::chrono::milliseconds(30000));
    CHECK(config.timeouts.transition_state == std::chrono::milliseconds(90000));
    CHECK(config.timeouts.fixed_wing_route == std::chrono::milliseconds(180000));
    CHECK(config.timeouts.fixed_wing_recovery == std::chrono::milliseconds(180000));
    CHECK(config.timeouts.transition_ready_dwell == std::chrono::milliseconds(2000));
    CHECK(config.timeouts.quadplane_landing == std::chrono::milliseconds(90000));
}

void test_default_vehicle_uses_reviewed_watchdog_confidence_floor() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    nomad::vehicle::Vehicle vehicle(connection);
    CHECK(vehicle.update_vio(true, 0.29F).success);

    const auto below_floor = vehicle.set_velocity({0.5F, 0.0F, 0.0F, 0.0F});

    CHECK(!below_floor.success);
    CHECK(connection.velocity_send_count == 0);
    CHECK(vehicle.update_vio(true, 0.31F).success);
    CHECK(vehicle.set_velocity({0.5F, 0.0F, 0.0F, 0.0F}).success);
    CHECK(connection.velocity_send_count == 1);
}

void test_vehicle_rejects_invalid_watchdog_policy_before_transmission() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    nomad::safety::WatchdogPolicy policy{};
    policy.min_vio_confidence = 2.0F;
    nomad::vehicle::VehicleConfig config{};
    config.watchdog = policy;
    nomad::vehicle::Vehicle vehicle(connection, config);
    CHECK(vehicle.update_vio(true, 1.0F).success);

    const auto result = vehicle.set_velocity({1.0F, 0.0F, 0.0F, 0.0F});

    CHECK(!result.success);
    CHECK(connection.velocity_send_count == 0);
}

void test_default_vehicle_uses_reviewed_velocity_limits() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    nomad::vehicle::Vehicle vehicle(connection);
    CHECK(vehicle.update_vio(true, 1.0F).success);

    const auto result = vehicle.set_velocity({3.0F, -3.0F, 2.0F, -2.0F});

    CHECK(result.success);
    CHECK(connection.last_velocity.vx == 2.0F);
    CHECK(connection.last_velocity.vy == 2.0F);
    CHECK(connection.last_velocity.vz == -1.0F);
    CHECK(connection.last_velocity.yaw_rate == 1.0F);
}

void test_vehicle_uses_configured_position_freshness() {
    FakeConnection connection;
    connection.connect();
    connection.acknowledgement = nomad::mavlink::CommandAck{192, 0};
    connection.auto_stamp_fresh_fields = false;
    connection.state->position_updated_at =
        std::chrono::steady_clock::now() - std::chrono::milliseconds(20);
    nomad::vehicle::VehicleConfig config{};
    config.timeouts.position_freshness = std::chrono::milliseconds(11);
    nomad::vehicle::Vehicle vehicle(connection, config);

    const auto result = vehicle.goto_location({45.5, -73.6, 5.0F});

    CHECK(!result.success);
    CHECK(result.message == "goto location verification failed: position feed is stale");
    CHECK(connection.last_goto.has_value());
}

void test_copter_takeoff_keeps_its_independent_timeout() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::VehicleConfig config{};
    config.timeouts.vtol_takeoff_state = std::chrono::milliseconds::zero();
    nomad::vehicle::Vehicle vehicle(connection, config);

    const auto result = vehicle.takeoff(5.0F);

    CHECK(result.success);
    CHECK(result.message == "takeoff verified");
    CHECK(connection.command_history.size() == 1);
    CHECK(connection.command_history.front().id == 22);
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_vehicle_config_keeps_reviewed_defaults();
        test_default_vehicle_uses_reviewed_watchdog_confidence_floor();
        test_vehicle_rejects_invalid_watchdog_policy_before_transmission();
        test_default_vehicle_uses_reviewed_velocity_limits();
        test_vehicle_uses_configured_position_freshness();
        test_copter_takeoff_keeps_its_independent_timeout();
    });
}
