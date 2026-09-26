// SPDX-License-Identifier: Apache-2.0

#include "fake_connection.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "test_harness.hpp"

#include <chrono>
#include <cstdint>
#include <optional>

namespace {

class PostAckVtolTakeoffConnection final : public FakeConnection {
  public:
    std::optional<nomad::telemetry::VehicleState> completion_state;
    bool change_session_before_takeoff_send{false};
    int expected_takeoff_send_count{0};

  private:
    bool completion_pending_{false};

    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                           std::chrono::milliseconds timeout) override {
        const auto ack = FakeConnection::send_command(command, timeout);
        if (command.id == 22 && ack.has_value() && ack->result == 0) {
            completion_pending_ = true;
        }
        return ack;
    }

    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                           std::uint64_t expected_session_id,
                                                           std::chrono::milliseconds timeout) override {
        if (command.id == 22) {
            expected_takeoff_send_count += 1;
            if (change_session_before_takeoff_send) {
                ++state->session_id;
                change_session_before_takeoff_send = false;
            }
        }
        if (expected_session_id == 0 || state->session_id != expected_session_id) {
            return std::nullopt;
        }
        const auto ack = send_command(command, timeout);
        if (state->session_id != expected_session_id) {
            return std::nullopt;
        }
        return ack;
    }

    std::optional<nomad::telemetry::VehicleState> wait_for_state(std::chrono::milliseconds timeout) override {
        const auto observed = FakeConnection::wait_for_state(timeout);
        if (!completion_pending_ || !observed.has_value()) {
            return observed;
        }
        if (completion_state.has_value()) {
            *state = *completion_state;
        }
        state->position_updated_at = std::chrono::steady_clock::now();
        completion_pending_ = false;
        return *state;
    }
};

class PreparationSessionRaceConnection final : public FakeConnection {
  public:
    std::uint16_t change_session_before_command{0};

  private:
    void change_session_if_requested(const nomad::mavlink::Command &command) {
        if (command.id != change_session_before_command) {
            return;
        }
        ++state->session_id;
        change_session_before_command = 0;
    }

    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                           std::chrono::milliseconds timeout) override {
        change_session_if_requested(command);
        return FakeConnection::send_command(command, timeout);
    }

    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                           std::uint64_t expected_session_id,
                                                           std::chrono::milliseconds timeout) override {
        change_session_if_requested(command);
        return nomad::mavlink::MavlinkConnection::send_command(command, expected_session_id, timeout);
    }
};

void configure_quadplane_takeoff_state(FakeConnection &connection, float relative_altitude_m) {
    connection.state->identity =
        nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot, nomad::telemetry::kVtolQuadrotor);
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

void test_quadplane_vtol_takeoff_accepts_post_ack_climb_in_admitted_session() {
    PostAckVtolTakeoffConnection connection;
    connection.connect();
    configure_quadplane_takeoff_state(connection, 2.0F);
    connection.auto_stamp_fresh_fields = false;
    connection.takeoff_altitude_override = 2.0F;
    auto completion_state = *connection.state;
    completion_state.armed = true;
    completion_state.custom_mode = 15;
    completion_state.position.relative_altitude_m = 7.0F;
    connection.completion_state = completion_state;
    const auto initial_state = *connection.state;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.vtol_takeoff(5.0F);

    CHECK(result.success);
    CHECK(result.message == "vtol takeoff verified");
    CHECK(connection.command_history.size() == 3);
    CHECK(connection.command_history[2].id == 22);
    CHECK(connection.command_history[2].parameters[6] == 5.0F);
    CHECK(connection.expected_takeoff_send_count == 1);
    CHECK(connection.state->session_id == initial_state.session_id);
    CHECK(connection.state->position.relative_altitude_m == 7.0F);
    CHECK(connection.state->position_updated_at > initial_state.position_updated_at);
}

void test_zero_vtol_timeout_stops_before_a_post_ack_completion_sample() {
    PostAckVtolTakeoffConnection connection;
    connection.connect();
    configure_quadplane_takeoff_state(connection, 2.0F);
    connection.auto_stamp_fresh_fields = false;
    connection.takeoff_altitude_override = 2.0F;
    auto completion_state = *connection.state;
    completion_state.armed = true;
    completion_state.custom_mode = 15;
    completion_state.position.relative_altitude_m = 7.0F;
    connection.completion_state = completion_state;
    nomad::vehicle::VehicleConfig config{};
    config.timeouts.vtol_takeoff_state = std::chrono::milliseconds::zero();
    nomad::vehicle::Vehicle vehicle(connection, config);
    const auto started_at = std::chrono::steady_clock::now();

    const auto result = vehicle.vtol_takeoff(5.0F);
    const auto elapsed = std::chrono::steady_clock::now() - started_at;

    CHECK(!result.success);
    CHECK(result.message == "vtol takeoff acknowledgement received but climb verification timed out");
    CHECK(connection.command_history.size() == 3);
    CHECK(connection.expected_takeoff_send_count == 1);
    CHECK(elapsed < std::chrono::milliseconds(500));
}

void test_quadplane_vtol_takeoff_rejects_pre_ack_high_position_sample() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_takeoff_state(connection, 2.0F);
    connection.auto_stamp_fresh_fields = false;
    connection.takeoff_altitude_override = 7.0F;
    const auto pre_ack_position_timestamp = connection.state->position_updated_at;
    nomad::vehicle::VehicleConfig config{};
    config.timeouts.vtol_takeoff_state = std::chrono::milliseconds(20);
    nomad::vehicle::Vehicle vehicle(connection, config);

    const auto result = vehicle.vtol_takeoff(5.0F);

    CHECK(!result.success);
    CHECK(result.message == "vtol takeoff acknowledgement received but climb verification timed out");
    CHECK(connection.command_history.size() == 3);
    CHECK(connection.state->position.relative_altitude_m == 7.0F);
    CHECK(connection.state->position_updated_at == pre_ack_position_timestamp);
}

void test_quadplane_vtol_takeoff_rejects_new_session_completion() {
    PostAckVtolTakeoffConnection connection;
    connection.connect();
    configure_quadplane_takeoff_state(connection, 2.0F);
    connection.auto_stamp_fresh_fields = false;
    connection.takeoff_altitude_override = 2.0F;
    auto completion_state = *connection.state;
    completion_state.session_id += 1;
    completion_state.armed = true;
    completion_state.custom_mode = 15;
    completion_state.position.relative_altitude_m = 7.0F;
    connection.completion_state = completion_state;
    const auto initial_state = *connection.state;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.vtol_takeoff(5.0F);

    CHECK(!result.success);
    CHECK(result.message == "vtol takeoff verification failed: vehicle session changed");
    CHECK(connection.command_history.size() == 3);
    CHECK(connection.expected_takeoff_send_count == 1);
    CHECK(connection.state->session_id != initial_state.session_id);
    CHECK(connection.state->system_id == initial_state.system_id);
    CHECK(connection.state->component_id == initial_state.component_id);
    CHECK(connection.state->position_updated_at > initial_state.position_updated_at);
}

void test_quadplane_vtol_takeoff_does_not_send_into_new_session() {
    PostAckVtolTakeoffConnection connection;
    connection.connect();
    configure_quadplane_takeoff_state(connection, 2.0F);
    connection.change_session_before_takeoff_send = true;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.vtol_takeoff(5.0F);

    CHECK(!result.success);
    CHECK(result.message == "vtol takeoff timed out waiting for acknowledgement");
    CHECK(connection.expected_takeoff_send_count == 1);
    CHECK(connection.command_history.size() == 2);
    CHECK(connection.command_history.back().id == 400);
}

void test_quadplane_vtol_takeoff_does_not_send_guided_mode_into_new_session() {
    PreparationSessionRaceConnection connection;
    connection.connect();
    configure_quadplane_takeoff_state(connection, 2.0F);
    connection.change_session_before_command = 176;
    const auto admitted_session = connection.state->session_id;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.vtol_takeoff(5.0F);

    CHECK(!result.success);
    CHECK(result.message == "set guided mode timed out waiting for acknowledgement");
    CHECK(connection.command_history.empty());
    CHECK(connection.state->session_id != admitted_session);
}

void test_quadplane_vtol_takeoff_does_not_send_arm_into_new_session() {
    PreparationSessionRaceConnection connection;
    connection.connect();
    configure_quadplane_takeoff_state(connection, 2.0F);
    connection.change_session_before_command = 400;
    const auto admitted_session = connection.state->session_id;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.vtol_takeoff(5.0F);

    CHECK(!result.success);
    CHECK(result.message == "arm timed out waiting for acknowledgement");
    CHECK(connection.command_history.size() == 1);
    CHECK(connection.command_history.front().id == 176);
    CHECK(connection.state->session_id != admitted_session);
    CHECK(!connection.state->armed);
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_quadplane_vtol_takeoff_accepts_post_ack_climb_in_admitted_session();
        test_zero_vtol_timeout_stops_before_a_post_ack_completion_sample();
        test_quadplane_vtol_takeoff_rejects_pre_ack_high_position_sample();
        test_quadplane_vtol_takeoff_rejects_new_session_completion();
        test_quadplane_vtol_takeoff_does_not_send_into_new_session();
        test_quadplane_vtol_takeoff_does_not_send_guided_mode_into_new_session();
        test_quadplane_vtol_takeoff_does_not_send_arm_into_new_session();
    });
}
