// SPDX-License-Identifier: Apache-2.0
#include "nomad/vehicle/vehicle.hpp"
#include "support/fake_connection.hpp"
#include "support/test_harness.hpp"

#include <chrono>
#include <string_view>
#include <thread>

namespace {
using Outcome = nomad::vehicle::CommandOutcome;
enum class Observation {
    Fresh, Session, System, Component, Class, Autopilot, VehicleType, Disconnected,
    Cached, ZeroTime, FutureTime, StaleTime, StaleFlag
};

class ArmConnection final : public FakeConnection {
  public:
    explicit ArmConnection(Observation value) : observation(value) {
        auto_stamp_fresh_fields = false;
        acknowledgement = nomad::mavlink::CommandAck{400, 0};
    }

    std::optional<nomad::mavlink::CommandAck> send_command(
        const nomad::mavlink::Command &command, std::chrono::milliseconds timeout) override {
        const auto result = FakeConnection::send_command(command, timeout);
        return result && result->status != nomad::mavlink::CommandAck::Status::AdmissionCancelled ?
            acknowledgement : result;
    }

    std::optional<nomad::telemetry::VehicleState> wait_for_state(std::chrono::milliseconds) override {
        ++polls;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::lock_guard lock(state_mutex);
        if (observation != Observation::Cached) {
            state->heartbeat_updated_at = std::chrono::steady_clock::now();
        }
        switch (observation) {
        case Observation::Session: ++state->session_id; break;
        case Observation::System: ++state->system_id; break;
        case Observation::Component: ++state->component_id; break;
        case Observation::Class: state->identity.aircraft_class = nomad::telemetry::AircraftClass::Plane; break;
        case Observation::Autopilot: ++state->identity.autopilot_type; break;
        case Observation::VehicleType: ++state->identity.vehicle_type; break;
        case Observation::Disconnected: state->connected = false; break;
        case Observation::ZeroTime: state->heartbeat_updated_at = {}; break;
        case Observation::FutureTime: state->heartbeat_updated_at += std::chrono::seconds(10); break;
        case Observation::StaleTime: state->heartbeat_updated_at -= std::chrono::seconds(10); break;
        case Observation::StaleFlag: state->heartbeat_fresh = false; break;
        default: break;
        }
        // Invalid samples must not finish arming; then expose independent context loss.
        if (polls == 2) {
            state->connected = false;
        }
        return *state;
    }

    Observation observation;
    int polls{};
};

void test_arm_rejects_replaced_context(Observation observation) {
    ArmConnection connection(observation);
    CHECK(connection.connect());
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = vehicle.arm();
    CHECK(!result.success);
    CHECK(result.acknowledged);
    CHECK(result.outcome == Outcome::Interrupted);
    CHECK(connection.polls == 1);
    CHECK(connection.command_history.size() == 1);
}

void test_arm_requires_fresh_post_ack_sample(Observation observation) {
    ArmConnection connection(observation);
    CHECK(connection.connect());
    connection.state->armed = true;
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = vehicle.arm();
    CHECK(!result.success);
    CHECK(result.acknowledged);
    CHECK(result.outcome == Outcome::Interrupted);
    CHECK(connection.polls == 2);
    CHECK(connection.command_history.size() == 1);
}

void test_arm_rejects_unusable_initial_context() {
    for (int fault = 0; fault < 10; ++fault) {
        ArmConnection connection(Observation::Fresh);
        CHECK(connection.connect());
        switch (fault) {
        case 0: connection.state->session_id = 0; break;
        case 1: connection.state->system_id = 0; break;
        case 2: connection.state->component_id = 0; break;
        case 3: connection.state->connected = false; break;
        case 4: connection.state->heartbeat_fresh = false; break;
        case 5: connection.state->heartbeat_updated_at = {}; break;
        case 6: connection.state->heartbeat_updated_at += std::chrono::seconds(10); break;
        case 7: connection.state->heartbeat_updated_at -= std::chrono::seconds(10); break;
        case 8: connection.state->identity.autopilot_type = 0; break;
        case 9: connection.connected = false; break;
        }
        nomad::vehicle::Vehicle vehicle(connection);
        const auto result = vehicle.arm();
        CHECK(!result.success);
        CHECK(!result.acknowledged);
        CHECK(result.outcome == Outcome::Rejected);
        CHECK(connection.command_history.empty());
    }
}

void test_arm_rechecks_context_before_transmission() {
    ArmConnection connection(Observation::Fresh);
    CHECK(connection.connect());
    connection.before_command_send = [&] {
        auto state = connection.get_state();
        ++state.session_id;
        connection.set_state(state);
    };
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = vehicle.arm();
    CHECK(connection.command_history.empty());
    CHECK(!result.success);
    CHECK(!result.acknowledged);
    CHECK(result.outcome == Outcome::Interrupted);
}

void test_arm_preserves_accepted_ack_and_new_observation() {
    for (const bool already_armed : {false, true}) {
        ArmConnection connection(Observation::Fresh);
        CHECK(connection.connect());
        connection.state->armed = already_armed;
        nomad::vehicle::Vehicle vehicle(connection);
        const auto result = vehicle.arm();
        CHECK(result.success);
        CHECK(result.acknowledged);
        CHECK(result.outcome == Outcome::Success);
        CHECK(connection.polls == 1);
        CHECK(connection.command_history.size() == 1);
        CHECK(connection.last_command.id == 400);
        CHECK(connection.last_command.parameters[0] == 1.0F);
        CHECK(connection.last_command.parameters[1] == 0.0F);
    }
}

void test_arm_preserves_negative_missing_and_wrong_ack() {
    for (int fault = 0; fault < 3; ++fault) {
        ArmConnection connection(Observation::Fresh);
        CHECK(connection.connect());
        connection.acknowledgement = fault == 0 ? std::nullopt :
            std::optional{nomad::mavlink::CommandAck{static_cast<std::uint16_t>(fault == 1 ? 401 : 400),
                                                   static_cast<std::uint8_t>(fault == 2 ? 4 : 0)}};
        nomad::vehicle::Vehicle vehicle(connection);
        const auto result = vehicle.arm();
        CHECK(!result.success);
        CHECK(result.acknowledged == (fault == 2));
        CHECK(result.outcome == (fault == 2 ? Outcome::Failed : Outcome::Unknown));
        CHECK(connection.polls == 0);
        CHECK(connection.command_history.size() == 1);
    }
}
}

int main(int argc, char **argv) {
    return nomad::test::run_tests([argc, argv] {
        if (argc == 2 && std::string_view(argv[1]) == "--arm-session") {
            nomad::test::run_scenario("arm_replaced_session", [] {
                test_arm_rejects_replaced_context(Observation::Session);
            });
            return;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--arm-cached") {
            nomad::test::run_scenario("arm_cached_sample", [] {
                test_arm_requires_fresh_post_ack_sample(Observation::Cached);
            });
            return;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--arm-send-context") {
            nomad::test::run_scenario("arm_final_send_context", test_arm_rechecks_context_before_transmission);
            return;
        }
        nomad::test::run_scenario("arm_initial_context", test_arm_rejects_unusable_initial_context);
        for (const auto observation : {Observation::Session, Observation::System, Observation::Component,
                Observation::Class, Observation::Autopilot, Observation::VehicleType, Observation::Disconnected}) {
            test_arm_rejects_replaced_context(observation);
        }
        for (const auto observation : {Observation::Cached, Observation::ZeroTime, Observation::FutureTime,
                Observation::StaleTime, Observation::StaleFlag}) {
            test_arm_requires_fresh_post_ack_sample(observation);
        }
        test_arm_rechecks_context_before_transmission();
        test_arm_preserves_accepted_ack_and_new_observation();
        test_arm_preserves_negative_missing_and_wrong_ack();
    });
}
