// SPDX-License-Identifier: Apache-2.0
#include "nomad/vehicle/vehicle.hpp"
#include "support/copter_land_connection.hpp"
#include "support/test_harness.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using Outcome = nomad::vehicle::CommandOutcome;
using Observation = CopterLandConnection::Observation;

void check_result(const nomad::vehicle::CommandResult &result, Outcome expected, bool acknowledged) {
    CHECK(result.outcome == expected);
    CHECK(result.success == (expected == Outcome::Success));
    CHECK(result.acknowledged == acknowledged);
}

void test_land_observes_engagement_without_touchdown() {
    CopterLandConnection connection;
    CHECK(connection.connect());
    connection.state->armed = true;
    connection.state->landed_state = nomad::telemetry::LandedState::InAir;
    connection.state->landed_state_valid = true;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.engage_copter_land();

    check_result(result, Outcome::Success, true);
    CHECK(result.message == "LAND mode observed; touchdown not verified");
    CHECK(connection.command_count() == 1);
    CHECK(connection.last_command.id == 21);
    CHECK(connection.get_state().landed_state == nomad::telemetry::LandedState::InAir);
}

void test_land_rejects_invalid_initial_state() {
    using State = nomad::telemetry::VehicleState;
    const std::vector<std::function<void(State &)>> invalid_states{
        [](State &state) { state.session_id = 0; },
        [](State &state) { state.system_id = 0; },
        [](State &state) { state.component_id = 0; },
        [](State &state) { state.connected = false; },
        [](State &state) { state.heartbeat_fresh = false; },
        [](State &state) { state.heartbeat_updated_at = {}; },
        [](State &state) { state.heartbeat_updated_at = Clock::now() - std::chrono::seconds(4); },
        [](State &state) { state.heartbeat_updated_at = Clock::now() + std::chrono::seconds(1); },
        [](State &state) { state.identity = nomad::telemetry::identify_vehicle(3, 1); },
        [](State &state) { state.identity.aircraft_class = nomad::telemetry::AircraftClass::QuadPlane; },
        [](State &state) { state.identity.autopilot_type = 0; },
    };
    for (const auto &invalidate : invalid_states) {
        CopterLandConnection connection;
        CHECK(connection.connect());
        invalidate(*connection.state);
        nomad::vehicle::Vehicle vehicle(connection);
        check_result(vehicle.engage_copter_land(), Outcome::Rejected, false);
        CHECK(connection.command_count() == 0);
    }
}

void test_land_acknowledgement_outcomes() {
    for (const bool missing : {false, true}) {
        CopterLandConnection connection;
        CHECK(connection.connect());
        connection.acknowledgement = nomad::mavlink::CommandAck{21, 2};
        if (missing) {
            connection.acknowledgement.reset();
        }
        nomad::vehicle::Vehicle vehicle(connection);
        check_result(vehicle.engage_copter_land(), missing ? Outcome::Unknown : Outcome::Failed, !missing);
        CHECK(connection.command_count() == 1);
    }
}

void test_land_rechecks_identity_at_final_send() {
    using State = nomad::telemetry::VehicleState;
    const std::vector<std::function<void(State &)>> changes{
        [](State &state) { state.identity = nomad::telemetry::identify_vehicle(3, 1); },
        [](State &state) { ++state.session_id; },
        [](State &state) { ++state.system_id; },
        [](State &state) { ++state.component_id; },
    };
    for (const auto &change : changes) {
        CopterLandConnection connection;
        CHECK(connection.connect());
        connection.before_command_send = [&] { change(*connection.state); };
        nomad::vehicle::Vehicle vehicle(connection);
        check_result(vehicle.engage_copter_land(), Outcome::Interrupted, false);
        CHECK(connection.command_count() == 0);
    }
}

void test_land_expired_budget_prevents_final_send() {
    CopterLandConnection connection;
    CHECK(connection.connect());
    connection.command_delay = std::chrono::milliseconds(3050);
    nomad::vehicle::Vehicle vehicle(connection);
    check_result(vehicle.engage_copter_land(), Outcome::Unknown, false);
    CHECK(connection.command_count() == 0);
}

void test_land_stale_heartbeat_prevents_final_send() {
    CopterLandConnection connection;
    CHECK(connection.connect());
    connection.before_command_send = [&] {
        connection.state->heartbeat_updated_at = Clock::now() - std::chrono::seconds(4);
    };
    nomad::vehicle::Vehicle vehicle(connection);
    check_result(vehicle.engage_copter_land(), Outcome::Unknown, false);
    CHECK(connection.command_count() == 0);
}

void test_land_lost_authority_prevents_final_send() {
    CopterLandConnection connection;
    CHECK(connection.connect());
    bool authorized = true;
    connection.before_command_send = [&] { authorized = false; };
    connection.set_transmission_admission_factory([&] {
        return nomad::mavlink::TransmissionAdmission([&](const auto &send) {
            if (!authorized) {
                return false;
            }
            send();
            return true;
        });
    });
    nomad::vehicle::Vehicle vehicle(connection);
    check_result(vehicle.engage_copter_land([&] { return authorized; }), Outcome::Interrupted, false);
    CHECK(connection.command_count() == 0);
}

void test_land_rejects_changed_identity_after_ack() {
    for (const auto observation : {Observation::ChangedSession, Observation::ChangedIdentity,
        Observation::ChangedSystem, Observation::ChangedComponent, Observation::LostLink}) {
        CopterLandConnection connection;
        CHECK(connection.connect());
        connection.observation = observation;
        nomad::vehicle::Vehicle vehicle(connection);
        check_result(vehicle.engage_copter_land(), Outcome::Interrupted, true);
        CHECK(connection.command_count() == 1);
    }
}

void test_land_does_not_accept_cached_or_future_heartbeat() {
    for (const auto observation : {Observation::PreAckLand, Observation::FutureHeartbeat}) {
        CopterLandConnection connection;
        CHECK(connection.connect());
        connection.observation = observation;
        nomad::vehicle::Vehicle vehicle(connection);
        const auto started = Clock::now();
        const auto result = vehicle.engage_copter_land();
        const auto elapsed = Clock::now() - started;
        check_result(result, Outcome::Unknown, true);
        CHECK(elapsed >= std::chrono::milliseconds(2950));
        CHECK(elapsed < std::chrono::milliseconds(3400));
        CHECK(connection.command_count() == 1);
    }
}

void test_land_ack_and_verification_share_three_seconds() {
    CopterLandConnection connection;
    CHECK(connection.connect());
    connection.ack_delay = std::chrono::milliseconds(2400);
    connection.heartbeat_delay = std::chrono::milliseconds(800);
    nomad::vehicle::Vehicle vehicle(connection);
    const auto started = Clock::now();

    const auto result = vehicle.engage_copter_land();
    const auto elapsed = Clock::now() - started;

    check_result(result, Outcome::Unknown, true);
    CHECK(elapsed >= std::chrono::milliseconds(2950));
    CHECK(elapsed < std::chrono::milliseconds(3400));
    CHECK(connection.supplied_timeout <= std::chrono::seconds(3));
    CHECK(connection.command_count() == 1);
}

void test_land_uses_remaining_budget_for_late_ack() {
    CopterLandConnection connection;
    CHECK(connection.connect());
    connection.ack_delay = std::chrono::milliseconds(2200);
    connection.heartbeat_delay = std::chrono::milliseconds(350);
    nomad::vehicle::Vehicle vehicle(connection);
    const auto started = Clock::now();
    const auto result = vehicle.engage_copter_land();
    check_result(result, Outcome::Success, true);
    CHECK(Clock::now() - started < std::chrono::milliseconds(2950));
    CHECK(connection.command_count() == 1);
}

void test_land_cancels_verification_promptly() {
    CopterLandConnection connection;
    CHECK(connection.connect());
    connection.observation = Observation::OtherMode;
    nomad::vehicle::Vehicle vehicle(connection);
    const auto started = Clock::now();
    const auto authorized = [&] { return Clock::now() - started < std::chrono::milliseconds(100); };
    check_result(vehicle.engage_copter_land(authorized), Outcome::Interrupted, true);
    CHECK(Clock::now() - started < std::chrono::milliseconds(400));
    CHECK(connection.command_count() == 1);
}

class DelayedLandStateConnection : public CopterLandConnection {
  public:
    nomad::telemetry::VehicleState get_state() const override {
        if (land_sent.load() && !read_delayed_.exchange(true)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(3050));
        }
        return CopterLandConnection::get_state();
    }

  private:
    mutable std::atomic_bool read_delayed_{false};
};

void test_land_read_crossing_deadline_cannot_succeed() {
    DelayedLandStateConnection connection;
    CHECK(connection.connect());
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.engage_copter_land();

    check_result(result, Outcome::Unknown, true);
    CHECK(connection.command_count() == 1);
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_land_observes_engagement_without_touchdown();
        test_land_rejects_invalid_initial_state();
        test_land_acknowledgement_outcomes();
        test_land_rechecks_identity_at_final_send();
        test_land_expired_budget_prevents_final_send();
        test_land_stale_heartbeat_prevents_final_send();
        test_land_lost_authority_prevents_final_send();
        test_land_rejects_changed_identity_after_ack();
        test_land_does_not_accept_cached_or_future_heartbeat();
        test_land_ack_and_verification_share_three_seconds();
        test_land_uses_remaining_budget_for_late_ack();
        test_land_cancels_verification_promptly();
        test_land_read_crossing_deadline_cannot_succeed();
    });
}
