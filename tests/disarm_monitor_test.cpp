// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
#include "nomad/vehicle/vehicle.hpp"
#include "support/fake_connection.hpp"
#include "support/test_harness.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

namespace {
using namespace std::chrono_literals;
enum class Observation {
    Fresh, Cached, Armed, ZeroTime, FutureTime, StaleTime, StaleFlag, NoSample, Late,
    Session, System, Component, Autopilot, VehicleType, Class, Disconnected, SlowInitialRead
};

class MonitorConnection final : public FakeConnection {
  public:
    explicit MonitorConnection(Observation value) : observation(value) {
        state->armed = true;
    }

    nomad::telemetry::VehicleState get_state() const override {
        if (observation == Observation::SlowInitialRead && first_read) {
            first_read = false;
            std::this_thread::sleep_for(80ms);
        }
        return FakeConnection::get_state();
    }

    std::optional<nomad::telemetry::VehicleState> wait_for_state(std::chrono::milliseconds timeout) override {
        ++polls;
        longest_wait = (std::max)(longest_wait, timeout);
        if (observation == Observation::NoSample) {
            std::this_thread::sleep_for(timeout);
            return std::nullopt;
        }
        if (observation == Observation::Late) {
            std::this_thread::sleep_for(80ms);
        } else if (observation != Observation::Fresh && observation != Observation::Cached) {
            std::this_thread::sleep_for(5ms);
        }
        std::lock_guard lock(state_mutex);
        state->armed = observation == Observation::Armed;
        if (observation != Observation::Cached) {
            state->heartbeat_updated_at = std::chrono::steady_clock::now();
        }
        switch (observation) {
        case Observation::ZeroTime:
            state->heartbeat_updated_at = {};
            break;
        case Observation::FutureTime:
            state->heartbeat_updated_at += 10s;
            break;
        case Observation::StaleTime:
            state->heartbeat_updated_at -= 10s;
            break;
        case Observation::StaleFlag:
            state->heartbeat_fresh = false;
            break;
        case Observation::Session:
            ++state->session_id;
            break;
        case Observation::System:
            ++state->system_id;
            break;
        case Observation::Component:
            ++state->component_id;
            break;
        case Observation::Autopilot:
            ++state->identity.autopilot_type;
            break;
        case Observation::VehicleType:
            ++state->identity.vehicle_type;
            break;
        case Observation::Class:
            state->identity.aircraft_class = nomad::telemetry::AircraftClass::Plane;
            break;
        case Observation::Disconnected:
            state->connected = false;
            connected = false;
            break;
        default:
            break;
        }
        return *state;
    }

    Observation observation;
    int polls{};
    mutable bool first_read{true};
    std::chrono::milliseconds longest_wait{};
};

void test_missing_or_unusable_disarmed_sample_times_out() {
    for (const auto observation : {Observation::Cached, Observation::Armed, Observation::ZeroTime,
            Observation::FutureTime, Observation::StaleTime, Observation::StaleFlag,
            Observation::NoSample, Observation::Late}) {
        MonitorConnection connection(observation);
        CHECK(connection.connect());
        connection.state->armed = false;
        nomad::vehicle::Vehicle vehicle(connection);
        const auto started = std::chrono::steady_clock::now();
        const auto result = vehicle.wait_until_disarmed(60ms);
        const auto elapsed = std::chrono::steady_clock::now() - started;
        CHECK(!result.success);
        CHECK(!result.acknowledged);
        CHECK(result.outcome == nomad::vehicle::CommandOutcome::Unknown);
        CHECK(elapsed >= 60ms);
        CHECK(connection.longest_wait <= 60ms);
        if (observation == Observation::Cached) {
            CHECK(connection.polls <= 40);
        }
        CHECK(connection.command_history.empty());
    }
}

void test_replaced_aircraft_cannot_finish_disarm_monitoring() {
    for (const auto observation : {Observation::Session, Observation::System, Observation::Component,
            Observation::Autopilot, Observation::VehicleType, Observation::Class, Observation::Disconnected}) {
        MonitorConnection connection(observation);
        CHECK(connection.connect());
        nomad::vehicle::Vehicle vehicle(connection);
        const auto result = vehicle.wait_until_disarmed(500ms);
        CHECK(!result.success);
        CHECK(!result.acknowledged);
        CHECK(result.outcome == nomad::vehicle::CommandOutcome::Interrupted);
        CHECK(connection.polls == 1);
        CHECK(connection.command_history.empty());
    }
}

void test_fresh_disarmed_heartbeat_completes_observation() {
    for (const bool initially_armed : {true, false}) {
        MonitorConnection connection(Observation::Fresh);
        CHECK(connection.connect());
        connection.state->armed = initially_armed;
        nomad::vehicle::Vehicle vehicle(connection);
        const auto result = vehicle.wait_until_disarmed(500ms);
        CHECK(result.success);
        CHECK(!result.acknowledged);
        CHECK(result.outcome == nomad::vehicle::CommandOutcome::Success);
        CHECK(connection.polls == 1);
        CHECK(!connection.get_state().armed);
        CHECK(connection.command_history.empty());
    }
}

void test_initial_context_read_consumes_the_same_budget() {
    MonitorConnection connection(Observation::SlowInitialRead);
    CHECK(connection.connect());
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = vehicle.wait_until_disarmed(60ms);
    CHECK(!result.success);
    CHECK(result.outcome == nomad::vehicle::CommandOutcome::Unknown);
    CHECK(connection.polls == 0);
    CHECK(connection.command_history.empty());
}

void test_missing_context_or_nonpositive_budget_is_rejected() {
    for (int fault = 0; fault < 7; ++fault) {
        MonitorConnection connection(Observation::Fresh);
        CHECK(connection.connect());
        auto timeout = 500ms;
        switch (fault) {
        case 0:
            connection.state->session_id = 0;
            break;
        case 1:
            connection.state->system_id = 0;
            break;
        case 2:
            connection.state->component_id = 0;
            break;
        case 3:
            connection.state->connected = false;
            break;
        case 4:
            connection.connected = false;
            break;
        case 5:
            timeout = 0ms;
            break;
        case 6:
            timeout = -1ms;
            break;
        }
        nomad::vehicle::Vehicle vehicle(connection);
        const auto result = vehicle.wait_until_disarmed(timeout);
        CHECK(!result.success);
        CHECK(!result.acknowledged);
        CHECK(result.outcome == nomad::vehicle::CommandOutcome::Rejected);
        CHECK(connection.polls == 0);
        CHECK(connection.command_history.empty());
    }
}
} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_missing_or_unusable_disarmed_sample_times_out();
        test_replaced_aircraft_cannot_finish_disarm_monitoring();
        test_fresh_disarmed_heartbeat_completes_observation();
        test_initial_context_read_consumes_the_same_budget();
        test_missing_context_or_nonpositive_budget_is_rejected();
    });
}
