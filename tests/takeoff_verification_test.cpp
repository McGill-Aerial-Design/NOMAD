// SPDX-License-Identifier: Apache-2.0
#include "nomad/vehicle/vehicle.hpp"
#include "support/fake_connection.hpp"
#include "support/test_harness.hpp"

#include <chrono>
#include <limits>
#include <string_view>
#include <thread>

namespace {
using Outcome = nomad::vehicle::CommandOutcome;
enum class Observation {
    Fresh, Session, System, Component, Class, Autopilot, VehicleType, Disconnected,
    Cached, ZeroPositionTime, FuturePositionTime, StalePositionTime, InvalidPosition,
    ZeroHeartbeatTime, FutureHeartbeatTime, StaleHeartbeatTime, StaleHeartbeatFlag,
    NanAltitude, InfiniteAltitude, BelowTarget
};

class TakeoffConnection final : public FakeConnection {
  public:
    explicit TakeoffConnection(Observation value) : observation(value) {
        auto_stamp_fresh_fields = false;
        acknowledgement = nomad::mavlink::CommandAck{22, 0};
        state->position_valid = true;
        state->position.relative_altitude_m = 10.0F;
        state->position_updated_at = std::chrono::steady_clock::now();
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
        const auto now = std::chrono::steady_clock::now();
        if (observation != Observation::Cached) {
            state->heartbeat_updated_at = now;
            state->position_updated_at = now;
        }
        switch (observation) {
        case Observation::Session: ++state->session_id; break;
        case Observation::System: ++state->system_id; break;
        case Observation::Component: ++state->component_id; break;
        case Observation::Class: state->identity.aircraft_class = nomad::telemetry::AircraftClass::Plane; break;
        case Observation::Autopilot: ++state->identity.autopilot_type; break;
        case Observation::VehicleType: ++state->identity.vehicle_type; break;
        case Observation::Disconnected: state->connected = false; break;
        case Observation::ZeroPositionTime: state->position_updated_at = {}; break;
        case Observation::FuturePositionTime: state->position_updated_at += std::chrono::seconds(10); break;
        case Observation::StalePositionTime: state->position_updated_at -= std::chrono::seconds(10); break;
        case Observation::InvalidPosition: state->position_valid = false; break;
        case Observation::ZeroHeartbeatTime: state->heartbeat_updated_at = {}; break;
        case Observation::FutureHeartbeatTime: state->heartbeat_updated_at += std::chrono::seconds(10); break;
        case Observation::StaleHeartbeatTime: state->heartbeat_updated_at -= std::chrono::seconds(10); break;
        case Observation::StaleHeartbeatFlag: state->heartbeat_fresh = false; break;
        case Observation::NanAltitude: state->position.relative_altitude_m = std::numeric_limits<float>::quiet_NaN(); break;
        case Observation::InfiniteAltitude: state->position.relative_altitude_m = std::numeric_limits<float>::infinity(); break;
        case Observation::BelowTarget: state->position.relative_altitude_m = 7.9F; break;
        default: break;
        }
        // Invalid evidence cannot complete takeoff; independent link loss ends the next poll.
        if (polls == 2) {
            state->connected = false;
        }
        return *state;
    }

    Observation observation;
    int polls{};
};

void test_takeoff_rejects_replaced_context(Observation observation) {
    TakeoffConnection connection(observation);
    CHECK(connection.connect());
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = vehicle.takeoff(10.0F);
    CHECK(!result.success);
    CHECK(result.acknowledged);
    CHECK(result.outcome == Outcome::Interrupted);
    CHECK(connection.polls == 1);
    CHECK(connection.command_history.size() == 1);
}

void test_takeoff_requires_new_valid_evidence(Observation observation) {
    TakeoffConnection connection(observation);
    CHECK(connection.connect());
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = vehicle.takeoff(10.0F);
    CHECK(!result.success);
    CHECK(result.acknowledged);
    CHECK(result.outcome != Outcome::Success);
    CHECK(connection.polls <= 2);
    CHECK(connection.command_history.size() == 1);
}

void test_takeoff_rechecks_context_before_transmission() {
    TakeoffConnection connection(Observation::Fresh);
    CHECK(connection.connect());
    connection.before_command_send = [&] {
        auto state = connection.get_state();
        ++state.session_id;
        connection.set_state(state);
    };
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = vehicle.takeoff(10.0F);
    CHECK(connection.command_history.empty());
    CHECK(!result.success);
    CHECK(!result.acknowledged);
    CHECK(result.outcome == Outcome::Interrupted);
}

void test_takeoff_rejects_unusable_initial_context() {
    for (int fault = 0; fault < 10; ++fault) {
        TakeoffConnection connection(Observation::Fresh);
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
        const auto result = vehicle.takeoff(10.0F);
        CHECK(!result.success);
        CHECK(!result.acknowledged);
        CHECK(result.outcome == Outcome::Rejected);
        CHECK(connection.command_history.empty());
    }
}

void test_takeoff_preserves_command_and_acknowledgement() {
    TakeoffConnection connection(Observation::Fresh);
    CHECK(connection.connect());
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = vehicle.takeoff(10.0F);
    CHECK(result.success);
    CHECK(result.acknowledged);
    CHECK(result.outcome == Outcome::Success);
    CHECK(connection.polls == 1);
    CHECK(connection.command_history.size() == 1);
    CHECK(connection.last_command.id == 22);
    CHECK(connection.last_command.parameters[6] == 10.0F);
}

void test_takeoff_preserves_negative_missing_and_wrong_ack() {
    for (int fault = 0; fault < 3; ++fault) {
        TakeoffConnection connection(Observation::Fresh);
        CHECK(connection.connect());
        connection.acknowledgement = fault == 0 ? std::nullopt :
            std::optional{nomad::mavlink::CommandAck{static_cast<std::uint16_t>(fault == 1 ? 23 : 22),
                                                   static_cast<std::uint8_t>(fault == 2 ? 4 : 0)}};
        nomad::vehicle::Vehicle vehicle(connection);
        const auto result = vehicle.takeoff(10.0F);
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
        if (argc == 2 && std::string_view(argv[1]) == "--takeoff-session") {
            nomad::test::run_scenario("takeoff_replaced_session", [] {
                test_takeoff_rejects_replaced_context(Observation::Session);
            });
            return;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--takeoff-cached") {
            nomad::test::run_scenario("takeoff_cached_position", [] {
                test_takeoff_requires_new_valid_evidence(Observation::Cached);
            });
            return;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--takeoff-send-context") {
            nomad::test::run_scenario("takeoff_final_send_context", test_takeoff_rechecks_context_before_transmission);
            return;
        }
        for (const auto observation : {Observation::Session, Observation::System, Observation::Component,
                                      Observation::Class, Observation::Autopilot, Observation::VehicleType,
                                      Observation::Disconnected}) {
            test_takeoff_rejects_replaced_context(observation);
        }
        for (const auto observation : {Observation::Cached, Observation::ZeroPositionTime,
                                      Observation::FuturePositionTime, Observation::StalePositionTime,
                                      Observation::InvalidPosition, Observation::ZeroHeartbeatTime,
                                      Observation::FutureHeartbeatTime, Observation::StaleHeartbeatTime,
                                      Observation::StaleHeartbeatFlag, Observation::NanAltitude,
                                      Observation::InfiniteAltitude, Observation::BelowTarget}) {
            test_takeoff_requires_new_valid_evidence(observation);
        }
        test_takeoff_rechecks_context_before_transmission();
        test_takeoff_rejects_unusable_initial_context();
        test_takeoff_preserves_command_and_acknowledgement();
        test_takeoff_preserves_negative_missing_and_wrong_ack();
    });
}
