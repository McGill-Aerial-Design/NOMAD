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
using Location = nomad::vehicle::Location;
enum class Observation {
    Fresh, Session, System, Component, Class, Autopilot, VehicleType, Disconnected,
    CachedPosition, CachedHeartbeat, ZeroPositionTime, FuturePositionTime, StalePositionTime, InvalidPosition,
    ZeroHeartbeatTime, FutureHeartbeatTime, StaleHeartbeatTime, StaleFlag,
    NanLatitude, NanLongitude, NanAltitude, OffLatitude, OffLongitude, OffAltitude, LatitudeRange, LongitudeRange
};

class GotoConnection final : public FakeConnection {
  public:
    explicit GotoConnection(Observation value) : observation(value) {
        auto_stamp_fresh_fields = false;
        acknowledgement = nomad::mavlink::CommandAck{192, 0};
        state->position_valid = true;
        state->position_updated_at = std::chrono::steady_clock::now();
        if (observation == Observation::CachedPosition) {
            state->position.latitude_deg = 45.5;
            state->position.longitude_deg = -73.6;
            state->position.relative_altitude_m = 10.0F;
        }
    }

    bool goto_location_relative(double latitude, double longitude, float altitude,
                                const nomad::mavlink::StateAdmission &state_admission,
                                std::chrono::milliseconds timeout) override {
        ++goto_calls;
        if (before_goto_send) {
            before_goto_send();
        }
        return FakeConnection::goto_location_relative(latitude, longitude, altitude, state_admission, timeout);
    }

    std::optional<nomad::telemetry::VehicleState> wait_for_state(std::chrono::milliseconds) override {
        ++polls;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::lock_guard lock(state_mutex);
        const auto now = std::chrono::steady_clock::now();
        if (observation != Observation::CachedPosition) {
            state->position_updated_at = now;
        }
        if (observation != Observation::CachedHeartbeat) {
            state->heartbeat_updated_at = now;
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
        case Observation::StaleFlag: state->heartbeat_fresh = false; break;
        case Observation::NanLatitude: state->position.latitude_deg = std::numeric_limits<double>::quiet_NaN(); break;
        case Observation::NanLongitude: state->position.longitude_deg = std::numeric_limits<double>::quiet_NaN(); break;
        case Observation::NanAltitude: state->position.relative_altitude_m = std::numeric_limits<float>::quiet_NaN(); break;
        case Observation::OffLatitude: state->position.latitude_deg += 0.001; break;
        case Observation::OffLongitude: state->position.longitude_deg += 0.001; break;
        case Observation::OffAltitude: state->position.relative_altitude_m += 3.0F; break;
        case Observation::LatitudeRange: state->position.latitude_deg = 90.000001; break;
        case Observation::LongitudeRange: state->position.longitude_deg = 180.000001; break;
        default: break;
        }
        // Invalid evidence cannot complete goto; independent link loss ends the next poll.
        if (polls == 2) {
            state->connected = false;
        }
        return *state;
    }

    Observation observation;
    std::function<void()> before_goto_send;
    int polls{};
    int goto_calls{};
};

void test_goto_rejects_replaced_context(Observation observation) {
    GotoConnection connection(observation);
    CHECK(connection.connect());
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = vehicle.goto_location({45.5, -73.6, 10.0F});
    CHECK(!result.success);
    CHECK(result.acknowledged);
    CHECK(result.outcome == Outcome::Interrupted);
    CHECK(connection.polls == 1);
    CHECK(connection.last_goto.has_value());
}

void test_goto_requires_new_valid_evidence(Observation observation) {
    GotoConnection connection(observation);
    CHECK(connection.connect());
    nomad::vehicle::Vehicle vehicle(connection);
    Location target{45.5, -73.6, 10.0F};
    if (observation == Observation::LatitudeRange) {
        target.latitude_deg = 90.0;
    }
    if (observation == Observation::LongitudeRange) {
        target.longitude_deg = 180.0;
    }
    const auto result = vehicle.goto_location(target);
    CHECK(!result.success);
    CHECK(result.acknowledged);
    CHECK(result.outcome != Outcome::Success);
    CHECK(connection.polls <= 2);
    CHECK(connection.last_goto.has_value());
}

void test_goto_rechecks_context_before_transmission(int fault) {
    GotoConnection connection(Observation::Fresh);
    CHECK(connection.connect());
    connection.before_goto_send = [&] {
        auto state = connection.get_state();
        if (fault == 0) {
            ++state.session_id;
        } else if (fault == 1) {
            state.heartbeat_fresh = false;
        } else {
            ++state.system_id;
        }
        connection.set_state(state);
    };
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = vehicle.goto_location({45.5, -73.6, 10.0F});
    CHECK(!connection.last_goto.has_value());
    CHECK(!result.success);
    CHECK(!result.acknowledged);
    CHECK(result.outcome == Outcome::Unknown);
    CHECK(connection.polls == 0);
}

void test_goto_rejects_unusable_initial_context() {
    for (int fault = 0; fault < 10; ++fault) {
        GotoConnection connection(Observation::Fresh);
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
        const auto result = vehicle.goto_location({45.5, -73.6, 10.0F});
        CHECK(!result.success);
        CHECK(!result.acknowledged);
        CHECK(result.outcome == Outcome::Rejected);
        CHECK(connection.goto_calls == 0);
        CHECK(!connection.last_goto.has_value());
    }
}

void test_goto_preserves_target_and_accepted_observation(const Location &target) {
    GotoConnection connection(Observation::Fresh);
    CHECK(connection.connect());
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = vehicle.goto_location(target);
    CHECK(result.success);
    CHECK(result.acknowledged);
    CHECK(result.outcome == Outcome::Success);
    CHECK(connection.polls == 1);
    CHECK(connection.goto_calls == 1);
    CHECK(connection.last_goto->latitude_deg == target.latitude_deg);
    CHECK(connection.last_goto->longitude_deg == target.longitude_deg);
    CHECK(connection.last_goto->relative_altitude_m == target.altitude_m);
}

void test_goto_reports_unknown_on_unsuccessful_action() {
    for (int fault = 0; fault < 3; ++fault) {
        GotoConnection connection(Observation::Fresh);
        CHECK(connection.connect());
        connection.acknowledgement = fault == 0 ? std::nullopt :
            std::optional{nomad::mavlink::CommandAck{static_cast<std::uint16_t>(fault == 1 ? 193 : 192),
                                                   static_cast<std::uint8_t>(fault == 2 ? 4 : 0)}};
        nomad::vehicle::Vehicle vehicle(connection);
        const auto result = vehicle.goto_location({45.5, -73.6, 10.0F});
        CHECK(!result.success);
        CHECK(!result.acknowledged);
        CHECK(result.outcome == Outcome::Unknown);
        CHECK(connection.polls == 0);
        CHECK(connection.goto_calls == 1);
    }
}
}

int main(int argc, char **argv) {
    return nomad::test::run_tests([argc, argv] {
        if (argc == 2 && std::string_view(argv[1]) == "--goto-session") {
            nomad::test::run_scenario("goto_replaced_session", [] {
                test_goto_rejects_replaced_context(Observation::Session);
            });
            return;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--goto-cached") {
            nomad::test::run_scenario("goto_cached_position", [] {
                test_goto_requires_new_valid_evidence(Observation::CachedPosition);
            });
            return;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--goto-send-context") {
            nomad::test::run_scenario("goto_final_send_context", [] { test_goto_rechecks_context_before_transmission(0); });
            return;
        }
        for (const auto observation : {Observation::Session, Observation::System, Observation::Component,
                Observation::Class, Observation::Autopilot, Observation::VehicleType, Observation::Disconnected}) {
            test_goto_rejects_replaced_context(observation);
        }
        for (const auto observation : {Observation::CachedPosition, Observation::CachedHeartbeat,
                Observation::ZeroPositionTime, Observation::FuturePositionTime, Observation::StalePositionTime,
                Observation::InvalidPosition, Observation::ZeroHeartbeatTime, Observation::FutureHeartbeatTime,
                Observation::StaleHeartbeatTime, Observation::StaleFlag, Observation::NanLatitude,
                Observation::NanLongitude, Observation::NanAltitude, Observation::OffLatitude,
                Observation::OffLongitude, Observation::OffAltitude, Observation::LatitudeRange,
                Observation::LongitudeRange}) {
            test_goto_requires_new_valid_evidence(observation);
        }
        for (int fault = 0; fault < 3; ++fault) {
            test_goto_rechecks_context_before_transmission(fault);
        }
        test_goto_rejects_unusable_initial_context();
        for (const auto &target : {Location{45.5, -73.6, 10.0F}, Location{90.0, 180.0, 10.0F},
                                  Location{-90.0, -180.0, 10.0F}}) {
            test_goto_preserves_target_and_accepted_observation(target);
        }
        test_goto_reports_unknown_on_unsuccessful_action();
    });
}
