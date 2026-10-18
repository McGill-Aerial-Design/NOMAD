// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
#include "nomad/vehicle/vehicle.hpp"
#include "support/fake_connection.hpp"
#include "support/test_harness.hpp"

#include <chrono>
#include <string_view>
#include <thread>

namespace {
class LateVerdictConnection final : public FakeConnection {
  public:
    bool is_connected() const override {
        if (verdict_pending_) {
            verdict_pending_ = false;
            verdict_entered = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1700));
        }
        return FakeConnection::is_connected();
    }

    std::optional<nomad::telemetry::VehicleState> wait_for_state(std::chrono::milliseconds) override {
        // Return before the ten-second observation deadline, then delay its verdict.
        std::this_thread::sleep_for(std::chrono::milliseconds(8500));
        std::lock_guard lock(state_mutex);
        state->heartbeat_updated_at = std::chrono::steady_clock::now();
        verdict_pending_ = true;
        return *state;
    }

    mutable bool verdict_entered{false};

  private:
    mutable bool verdict_pending_{false};
};

enum class Fault { None, Session, System, Component, Class, VehicleType, Disconnected, OldSample };

class RtlConnection final : public FakeConnection {
  public:
    explicit RtlConnection(Fault value) : fault(value) {}

    std::optional<nomad::mavlink::CommandAck> send_command(
        const nomad::mavlink::Command &command, std::chrono::milliseconds timeout) override {
        const auto ack = FakeConnection::send_command(command, timeout);
        std::lock_guard lock(state_mutex);
        if (fault == Fault::Session) {
            ++state->session_id;
        }
        if (fault == Fault::System) {
            ++state->system_id;
        }
        if (fault == Fault::Component) {
            ++state->component_id;
        }
        if (fault == Fault::Class) {
            state->identity = nomad::telemetry::identify_vehicle(
                nomad::telemetry::kArduPilotAutopilot, nomad::telemetry::kFixedWing);
            state->custom_mode = 11;
        }
        if (fault == Fault::VehicleType) {
            ++state->identity.vehicle_type;
        }
        if (fault == Fault::Disconnected) {
            state->connected = false;
        }
        // The shared fake rewrites ACK command IDs; preserve the injected wire
        // reply here so the wrong-command control exercises production validation.
        if (ack && ack->status != nomad::mavlink::CommandAck::Status::AdmissionCancelled) {
            return acknowledgement;
        }
        return ack;
    }

    std::optional<nomad::telemetry::VehicleState> wait_for_state(std::chrono::milliseconds) override {
        if (fault == Fault::OldSample) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        } else {
            std::lock_guard lock(state_mutex);
            state->heartbeat_updated_at = std::chrono::steady_clock::now();
        }
        return get_state();
    }
    Fault fault;
};

void test_rtl_cannot_verify_a_replaced_aircraft() {
    for (const auto fault : {Fault::Session, Fault::System, Fault::Component, Fault::Class,
                             Fault::VehicleType, Fault::Disconnected}) {
        RtlConnection connection(fault);
        CHECK(connection.connect());
        connection.acknowledgement = nomad::mavlink::CommandAck{20, 0};
        nomad::vehicle::Vehicle vehicle(connection);
        const auto result = vehicle.return_to_launch();
        CHECK(!result.success);
        CHECK(result.acknowledged);
        CHECK(result.outcome == nomad::vehicle::CommandOutcome::Interrupted);
        CHECK(connection.command_history.size() == 1);
    }
}

void test_rtl_requires_a_post_acknowledgement_sample() {
    RtlConnection connection(Fault::OldSample);
    CHECK(connection.connect());
    connection.acknowledgement = nomad::mavlink::CommandAck{20, 0};
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = vehicle.return_to_launch();
    CHECK(!result.success);
    CHECK(result.acknowledged);
    CHECK(result.outcome == nomad::vehicle::CommandOutcome::Unknown);
}

void test_rtl_preserves_ack_and_observed_mode() {
    RtlConnection connection(Fault::None);
    CHECK(connection.connect());
    connection.acknowledgement = nomad::mavlink::CommandAck{20, 0};
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = vehicle.return_to_launch();
    CHECK(result.success);
    CHECK(result.acknowledged);
    CHECK(result.outcome == nomad::vehicle::CommandOutcome::Success);
    CHECK(connection.command_history.size() == 1);
}

void test_rtl_rejects_unusable_initial_context() {
    for (int fault = 0; fault < 8; ++fault) {
        RtlConnection connection(Fault::None);
        CHECK(connection.connect());
        if (fault == 0) {
            connection.state->heartbeat_updated_at = {};
        }
        if (fault == 1) {
            connection.state->heartbeat_updated_at += std::chrono::seconds(10);
        }
        if (fault == 2) {
            connection.state->heartbeat_updated_at -= std::chrono::seconds(10);
        }
        if (fault == 3) {
            connection.state->heartbeat_fresh = false;
        }
        if (fault == 4) {
            connection.state->session_id = 0;
        }
        if (fault == 5) {
            connection.state->system_id = 0;
        }
        if (fault == 6) {
            connection.state->component_id = 0;
        }
        if (fault == 7) {
            connection.state->identity.autopilot_type = 0;
        }
        nomad::vehicle::Vehicle vehicle(connection);
        const auto result = vehicle.return_to_launch();
        CHECK(!result.success);
        CHECK(!result.acknowledged);
        CHECK(result.outcome == nomad::vehicle::CommandOutcome::Rejected);
        CHECK(connection.command_history.empty());
    }
}

void test_rtl_rechecks_context_at_send() {
    RtlConnection connection(Fault::None);
    CHECK(connection.connect());
    connection.before_command_send = [&] { ++connection.state->session_id; };
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = vehicle.return_to_launch();
    CHECK(!result.success);
    CHECK(!result.acknowledged);
    CHECK(result.outcome == nomad::vehicle::CommandOutcome::Interrupted);
    CHECK(connection.command_history.empty());
}

void test_rtl_preserves_negative_missing_and_wrong_acknowledgements() {
    for (int fault = 0; fault < 3; ++fault) {
        RtlConnection connection(Fault::None);
        CHECK(connection.connect());
        connection.acknowledgement = fault == 0 ? std::nullopt :
            std::optional{nomad::mavlink::CommandAck{static_cast<std::uint16_t>(fault == 1 ? 21 : 20), 4}};
        nomad::vehicle::Vehicle vehicle(connection);
        const auto result = vehicle.return_to_launch();
        CHECK(!result.success);
        CHECK(result.acknowledged == (fault == 2));
        CHECK(result.outcome == (fault == 2 ? nomad::vehicle::CommandOutcome::Failed
                                          : nomad::vehicle::CommandOutcome::Unknown));
        CHECK(connection.command_history.size() == 1);
    }
}

void test_rtl_cannot_accept_a_verdict_after_the_observation_deadline() {
    LateVerdictConnection connection;
    CHECK(connection.connect());
    connection.acknowledgement = nomad::mavlink::CommandAck{20, 0};
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.return_to_launch();

    CHECK(connection.verdict_entered);
    CHECK(!result.success);
    CHECK(result.acknowledged);
    CHECK(result.outcome == nomad::vehicle::CommandOutcome::Unknown);
    CHECK(connection.command_history.size() == 1);
}
}

int main(int argc, char **argv) {
    return nomad::test::run_tests([argc, argv] {
        if (argc == 2 && std::string_view(argv[1]) == "--wait-verdict") {
            nomad::test::run_scenario("rtl_verdict_after_deadline",
                test_rtl_cannot_accept_a_verdict_after_the_observation_deadline);
            return;
        }
        test_rtl_cannot_verify_a_replaced_aircraft();
        test_rtl_requires_a_post_acknowledgement_sample();
        test_rtl_preserves_ack_and_observed_mode();
        test_rtl_rejects_unusable_initial_context();
        test_rtl_rechecks_context_at_send();
        test_rtl_preserves_negative_missing_and_wrong_acknowledgements();
        nomad::test::run_scenario("rtl_verdict_after_deadline",
            test_rtl_cannot_accept_a_verdict_after_the_observation_deadline);
    });
}
