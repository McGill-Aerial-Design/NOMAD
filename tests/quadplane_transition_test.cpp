// SPDX-License-Identifier: Apache-2.0

#include "fake_connection.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "test_harness.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>

namespace {

class PostAckTransitionConnection final : public FakeConnection {
  public:
    std::optional<nomad::telemetry::VehicleState> completion_state;
    bool change_session_before_transition_send{false};
    int expected_transition_send_count{0};

  private:
    bool completion_pending_{false};

    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                           std::chrono::milliseconds timeout) override {
        const auto ack = FakeConnection::send_command(command, timeout);
        if (command.id == 3000 && ack.has_value() && ack->result == 0) {
            completion_pending_ = true;
        }
        return ack;
    }

    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                           std::uint64_t expected_session_id,
                                                           std::chrono::milliseconds timeout) override {
        if (command.id == 3000) {
            expected_transition_send_count += 1;
            if (change_session_before_transition_send) {
                ++state->session_id;
                change_session_before_transition_send = false;
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
        state->vtol_state = nomad::telemetry::VtolState::FixedWing;
        state->vtol_state_valid = true;
        state->vtol_state_updated_at = std::chrono::steady_clock::now();
        completion_pending_ = false;
        return *state;
    }
};

void configure_quadplane_transition_state(FakeConnection &connection,
                                           nomad::telemetry::VtolState vtol_state =
                                               nomad::telemetry::VtolState::Multicopter) {
    connection.state->identity =
        nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                            nomad::telemetry::kVtolQuadrotor);
    connection.state->connected = true;
    connection.state->heartbeat_fresh = true;
    connection.state->armed = true;
    connection.state->custom_mode = 10; // ArduPlane AUTO.
    connection.state->vtol_state = vtol_state;
    connection.state->vtol_state_valid = true;
    connection.state->vtol_state_updated_at = std::chrono::steady_clock::now();
}

void configure_post_ack_transition_completion(PostAckTransitionConnection &connection) {
    connection.connect();
    configure_quadplane_transition_state(connection);
    connection.auto_stamp_fresh_fields = false;
    connection.complete_transition_on_command = false;
    connection.completion_state = *connection.state;
}

nomad::vehicle::Vehicle make_short_transition_vehicle(FakeConnection &connection) {
    nomad::vehicle::VehicleConfig config{};
    config.timeouts.transition_state = std::chrono::milliseconds(20);
    return nomad::vehicle::Vehicle(connection, config);
}

void test_quadplane_transition_constructs_command_and_verifies_fixed_wing_state() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    connection.auto_stamp_fresh_fields = false;
    connection.complete_transition_on_command = false;
    connection.complete_transition_after_ack_on_poll = true;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(result.success);
    CHECK(result.message == "transition to fixed wing verified");
    CHECK(connection.command_history.size() == 1);
    CHECK(connection.command_history.front().id == 3000);
    CHECK(connection.command_history.front().parameters[0] == 4.0F);
    CHECK(connection.state->vtol_state == nomad::telemetry::VtolState::FixedWing);
}

void test_zero_transition_timeout_stops_before_post_ack_state_completion() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    nomad::vehicle::VehicleConfig config{};
    config.timeouts.transition_state = std::chrono::milliseconds::zero();
    nomad::vehicle::Vehicle vehicle(connection, config);
    const auto started_at = std::chrono::steady_clock::now();

    const auto result = vehicle.transition_to_fixed_wing();
    const auto elapsed = std::chrono::steady_clock::now() - started_at;

    CHECK(!result.success);
    CHECK(result.message ==
          "transition to fixed wing acknowledgement received but fixed-wing state verification timed out");
    CHECK(connection.command_history.size() == 1);
    CHECK(elapsed < std::chrono::milliseconds(500));
}

void test_quadplane_transition_rejects_same_system_id_new_session() {
    PostAckTransitionConnection connection;
    configure_post_ack_transition_completion(connection);
    const auto initial_system_id = connection.state->system_id;
    const auto initial_session_id = connection.state->session_id;
    connection.completion_state->session_id += 1;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing verification failed: vehicle session changed");
    CHECK(connection.expected_transition_send_count == 1);
    CHECK(connection.state->system_id == initial_system_id);
    CHECK(connection.state->session_id != initial_session_id);
}

void test_quadplane_transition_rejects_changed_component() {
    PostAckTransitionConnection connection;
    configure_post_ack_transition_completion(connection);
    const auto initial_component_id = connection.state->component_id;
    connection.completion_state->component_id += 1;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing verification failed: vehicle component changed");
    CHECK(connection.expected_transition_send_count == 1);
    CHECK(connection.state->component_id != initial_component_id);
}

void test_quadplane_transition_rejects_disarmed_completion() {
    PostAckTransitionConnection connection;
    configure_post_ack_transition_completion(connection);
    connection.completion_state->armed = false;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing verification failed: vehicle disarmed");
    CHECK(connection.expected_transition_send_count == 1);
    CHECK(!connection.state->armed);
}

void test_quadplane_transition_rejects_unexpected_mode_completion() {
    PostAckTransitionConnection connection;
    configure_post_ack_transition_completion(connection);
    connection.completion_state->custom_mode = 15;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing verification failed: AUTO mode is required");
    CHECK(connection.expected_transition_send_count == 1);
    CHECK(connection.state->custom_mode == 15);
}

void test_quadplane_transition_does_not_send_into_new_session() {
    PostAckTransitionConnection connection;
    configure_post_ack_transition_completion(connection);
    connection.change_session_before_transition_send = true;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing timed out waiting for acknowledgement");
    CHECK(connection.expected_transition_send_count == 1);
    CHECK(connection.command_history.empty());
}

void test_quadplane_transition_does_not_accept_fixed_wing_state_before_ack() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    connection.auto_stamp_fresh_fields = false;
    connection.complete_transition_on_command = true;
    auto vehicle = make_short_transition_vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message ==
          "transition to fixed wing acknowledgement received but fixed-wing state verification timed out");
    CHECK(connection.state->vtol_state == nomad::telemetry::VtolState::FixedWing);
}

void test_quadplane_transition_requires_auto_and_fresh_authoritative_state() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    connection.state->custom_mode = 15;
    nomad::vehicle::Vehicle vehicle(connection);

    auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing rejected: AUTO mode is required");
    CHECK(connection.command_history.empty());

    connection.state->custom_mode = 10;
    connection.state->vtol_state_valid = false;
    result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing rejected: VTOL state is unavailable");
    CHECK(connection.command_history.empty());

    connection.state->vtol_state_valid = true;
    connection.state->vtol_state_updated_at = std::chrono::steady_clock::now() - std::chrono::seconds(4);
    connection.auto_stamp_fresh_fields = false;
    result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing rejected: VTOL state feed is stale");
    CHECK(connection.command_history.empty());
}

void test_quadplane_transition_propagates_ack_failure() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    connection.complete_transition_on_command = false;
    connection.acknowledgement = nomad::mavlink::CommandAck{3000, 4};
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing rejected by ArduPilot");
    CHECK(connection.command_history.size() == 1);
    CHECK(connection.state->vtol_state == nomad::telemetry::VtolState::Multicopter);
}

void test_quadplane_transition_fails_when_state_disappears_after_ack() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    connection.transition_state_unavailable_on_command = true;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing verification failed: VTOL state is unavailable");
    CHECK(connection.command_history.size() == 1);
}

void test_quadplane_transition_ack_without_completion_times_out() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    connection.complete_transition_on_command = false;
    auto vehicle = make_short_transition_vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message ==
          "transition to fixed wing acknowledgement received but fixed-wing state verification timed out");
    CHECK(connection.state->vtol_state == nomad::telemetry::VtolState::Multicopter);
}

void test_quadplane_transition_intermediate_state_times_out() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    connection.transition_stays_intermediate = true;
    auto vehicle = make_short_transition_vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message ==
          "transition to fixed wing acknowledgement received but fixed-wing state verification timed out");
    CHECK(connection.state->vtol_state == nomad::telemetry::VtolState::TransitionToFixedWing);
}

void test_quadplane_transition_fails_closed_on_state_interruption() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    connection.transition_loses_link_on_command = true;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing verification failed: heartbeat is stale");
    CHECK(connection.command_history.size() == 1);
}

void test_unsupported_transition_aircrafts_reject_before_transmission() {
    const std::array aircraft_types{
        nomad::telemetry::kQuadrotor,
        nomad::telemetry::kFixedWing,
        std::uint8_t{0},
    };
    for (const auto vehicle_type : aircraft_types) {
        FakeConnection connection;
        connection.connect();
        if (vehicle_type == 0) {
            connection.state->identity = {};
        } else {
            connection.state->identity =
                nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot, vehicle_type);
        }
        nomad::vehicle::Vehicle vehicle(connection);

        const auto result = vehicle.transition_to_fixed_wing();

        CHECK(!result.success);
        CHECK(connection.command_history.empty());
    }
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_quadplane_transition_constructs_command_and_verifies_fixed_wing_state();
        test_zero_transition_timeout_stops_before_post_ack_state_completion();
        test_quadplane_transition_rejects_same_system_id_new_session();
        test_quadplane_transition_rejects_changed_component();
        test_quadplane_transition_rejects_disarmed_completion();
        test_quadplane_transition_rejects_unexpected_mode_completion();
        test_quadplane_transition_does_not_send_into_new_session();
        test_quadplane_transition_does_not_accept_fixed_wing_state_before_ack();
        test_quadplane_transition_requires_auto_and_fresh_authoritative_state();
        test_quadplane_transition_propagates_ack_failure();
        test_quadplane_transition_fails_when_state_disappears_after_ack();
        test_quadplane_transition_ack_without_completion_times_out();
        test_quadplane_transition_intermediate_state_times_out();
        test_quadplane_transition_fails_closed_on_state_interruption();
        test_unsupported_transition_aircrafts_reject_before_transmission();
    });
}
