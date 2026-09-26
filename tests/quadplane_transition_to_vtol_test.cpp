// SPDX-License-Identifier: Apache-2.0

#include "fake_connection.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "test_harness.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace {

using nomad::telemetry::AircraftClass;
using nomad::telemetry::VtolState;
using nomad::vehicle::RecoveryPoint;
using nomad::vehicle::Vehicle;

constexpr RecoveryPoint kTransitionPoint{45.0, -73.0, 20.0F};

enum class BackTransitionBehavior { MulticopterAfterAck, NoChange, IntermediateOnly, MulticopterBeforeAck,
                                    ChangeSession, LoseHeartbeat, Disarm, ChangeMode, UnstableSpeed };

class VtolTransitionFakeConnection final : public FakeConnection {
  public:
    BackTransitionBehavior behavior{BackTransitionBehavior::MulticopterAfterAck};
    std::optional<float> post_transition_altitude_m;

    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                           std::chrono::milliseconds timeout) override {
        const bool is_back_transition = command.id == 3000 && command.parameters[0] == 3.0F;
        if (is_back_transition && behavior == BackTransitionBehavior::MulticopterBeforeAck) {
            auto_stamp_fresh_fields = false;
            set_vtol_state(VtolState::Multicopter);
        }
        const auto acknowledgement = FakeConnection::send_command(command, timeout);
        if (!is_back_transition || !acknowledgement || acknowledgement->result != 0) {
            return acknowledgement;
        }

        switch (behavior) {
        case BackTransitionBehavior::MulticopterAfterAck: complete_after_ack_on_poll_ = true; break;
        case BackTransitionBehavior::NoChange: break;
        case BackTransitionBehavior::IntermediateOnly: set_vtol_state(VtolState::TransitionToMulticopter); break;
        case BackTransitionBehavior::MulticopterBeforeAck: break;
        case BackTransitionBehavior::ChangeSession: ++state->session_id; complete_after_ack_on_poll_ = true; break;
        case BackTransitionBehavior::LoseHeartbeat:
            state->connected = false;
            state->heartbeat_fresh = false;
            complete_after_ack_on_poll_ = true;
            break;
        case BackTransitionBehavior::Disarm: state->armed = false; complete_after_ack_on_poll_ = true; break;
        case BackTransitionBehavior::ChangeMode:
            state->custom_mode = 21;
            complete_after_ack_on_poll_ = true;
            break;
        case BackTransitionBehavior::UnstableSpeed: break;
        }
        return acknowledgement;
    }

    std::optional<nomad::telemetry::VehicleState> wait_for_state(std::chrono::milliseconds timeout) override {
        const auto sample = FakeConnection::wait_for_state(timeout);
        if (behavior == BackTransitionBehavior::UnstableSpeed) {
            state->velocity.groundspeed_mps = speed_sample_++ % 2 == 0 ? 0.0F : 4.0F;
            state->velocity_updated_at = std::chrono::steady_clock::now();
            return get_state();
        }
        if (!complete_after_ack_on_poll_) {
            return sample;
        }
        set_vtol_state(VtolState::Multicopter);
        if (post_transition_altitude_m.has_value()) {
            state->position.relative_altitude_m = *post_transition_altitude_m;
            state->position_updated_at = std::chrono::steady_clock::now();
        }
        complete_after_ack_on_poll_ = false;
        return get_state();
    }

  private:
    int speed_sample_{0};
    bool complete_after_ack_on_poll_{false};

    void set_vtol_state(VtolState vtol_state) {
        state->vtol_state = vtol_state;
        state->vtol_state_valid = true;
        state->vtol_state_updated_at = std::chrono::steady_clock::now();
    }
};

void configure_transition_ready_state(FakeConnection &connection) {
    connection.connect();
    connection.parameters = {{"Q_ENABLE", 2.0F},       {"Q_FRAME_CLASS", 7.0F}, {"Q_TILT_ENABLE", 1.0F},
                             {"Q_TILT_MASK", 3.0F},   {"Q_TILT_TYPE", 0.0F},  {"Q_TILT_RATE_UP", 40.0F},
                             {"Q_TILT_MAX", 45.0F}};
    auto &state = *connection.state;
    state.identity = {nomad::telemetry::kArduPilotAutopilot, nomad::telemetry::kFixedWing,
                      AircraftClass::QuadPlane};
    state.connected = true;
    state.heartbeat_fresh = true;
    state.armed = true;
    state.system_id = 1;
    state.component_id = 1;
    state.custom_mode = 10;
    state.position = {kTransitionPoint.latitude_deg, kTransitionPoint.longitude_deg, 120.0F, 20.0F};
    state.position_valid = true;
    state.position_updated_at = std::chrono::steady_clock::now();
    state.velocity = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    state.velocity_updated_at = std::chrono::steady_clock::now();
    state.gps = {3, 12};
    state.gps_valid = true;
    state.gps_updated_at = std::chrono::steady_clock::now();
    state.vtol_state = VtolState::FixedWing;
    state.vtol_state_valid = true;
    state.vtol_state_updated_at = std::chrono::steady_clock::now();
}

Vehicle short_vehicle(FakeConnection &connection, std::chrono::milliseconds timeout = std::chrono::milliseconds(300),
                     std::chrono::milliseconds dwell = std::chrono::milliseconds(10)) {
    nomad::vehicle::VehicleConfig config{};
    config.timeouts.transition_state = timeout;
    config.timeouts.transition_ready_dwell = dwell;
    return Vehicle(connection, config);
}

void check_rejects_before_transition(FakeConnection &connection, const RecoveryPoint &point = kTransitionPoint) {
    auto vehicle = short_vehicle(connection);
    CHECK(!vehicle.transition_to_vtol(point).success);
    CHECK(connection.command_history.empty());
}

void test_transition_to_vtol_requires_quadplane_and_reviewed_ready_state() {
    constexpr std::array unsupported_classes{AircraftClass::Copter, AircraftClass::Plane, AircraftClass::Unknown};
    for (const auto aircraft_class : unsupported_classes) {
        FakeConnection connection;
        configure_transition_ready_state(connection);
        connection.state->identity.aircraft_class = aircraft_class;
        check_rejects_before_transition(connection);
    }

    for (int failure = 0; failure < 13; ++failure) {
        FakeConnection connection;
        configure_transition_ready_state(connection);
        const auto stale = std::chrono::steady_clock::now() - std::chrono::seconds(4);
        switch (failure) {
        case 0: connection.state->armed = false; break;
        case 1: connection.state->connected = false; break;
        case 2: connection.state->heartbeat_fresh = false; break;
        case 3: connection.state->position_updated_at = stale; break;
        case 4: connection.state->gps_updated_at = stale; break;
        case 5: connection.state->vtol_state_updated_at = stale; break;
        case 6: connection.state->vtol_state = VtolState::Multicopter; break;
        case 7: connection.state->custom_mode = 15; break;
        case 8: connection.state->gps.fix_type = 2; break;
        case 9: connection.state->velocity_updated_at = stale; break;
        case 10: connection.state->velocity.groundspeed_mps = 29.0F; break;
        case 11: connection.state->position.relative_altitude_m = 12.0F; break;
        case 12: connection.state->system_id = 0; break;
        }
        connection.auto_stamp_fresh_fields = false;
        check_rejects_before_transition(connection);
    }

    FakeConnection outside_region;
    configure_transition_ready_state(outside_region);
    outside_region.state->position.latitude_deg += 0.00051;
    auto outside_vehicle = short_vehicle(outside_region, std::chrono::milliseconds(80));
    const auto outside_result = outside_vehicle.transition_to_vtol(kTransitionPoint);
    CHECK(!outside_result.success);
    CHECK(outside_region.command_history.empty());

    FakeConnection not_dwelled;
    configure_transition_ready_state(not_dwelled);
    not_dwelled.auto_stamp_fresh_fields = false;
    auto not_dwelled_vehicle = short_vehicle(not_dwelled, std::chrono::milliseconds(80));
    const auto not_dwelled_result = not_dwelled_vehicle.transition_to_vtol(kTransitionPoint);
    CHECK(!not_dwelled_result.success);
    CHECK(not_dwelled.command_history.empty());

    VtolTransitionFakeConnection unstable_speed;
    configure_transition_ready_state(unstable_speed);
    unstable_speed.behavior = BackTransitionBehavior::UnstableSpeed;
    auto unstable_speed_vehicle = short_vehicle(unstable_speed, std::chrono::milliseconds(80));
    const auto unstable_speed_result = unstable_speed_vehicle.transition_to_vtol(kTransitionPoint);
    CHECK(!unstable_speed_result.success);
    CHECK(unstable_speed.command_history.empty());

    constexpr std::array<const char *, 7> profile_parameters{
        "Q_ENABLE", "Q_FRAME_CLASS", "Q_TILT_ENABLE", "Q_TILT_MASK", "Q_TILT_TYPE", "Q_TILT_RATE_UP",
        "Q_TILT_MAX"};
    for (const auto *parameter : profile_parameters) {
        FakeConnection wrong_profile;
        configure_transition_ready_state(wrong_profile);
        wrong_profile.parameters[parameter] = 99.0F;
        check_rejects_before_transition(wrong_profile);
    }

    FakeConnection unknown_profile;
    configure_transition_ready_state(unknown_profile);
    unknown_profile.parameters.clear();
    check_rejects_before_transition(unknown_profile);

    FakeConnection changed_session;
    configure_transition_ready_state(changed_session);
    changed_session.change_session_after_param_read = true;
    check_rejects_before_transition(changed_session);
}

void test_transition_to_vtol_enforces_pre_transition_altitude_band() {
    FakeConnection outside_band;
    configure_transition_ready_state(outside_band);
    outside_band.state->position.relative_altitude_m = 12.0F;
    auto outside_band_vehicle = short_vehicle(outside_band);
    const auto outside_band_result = outside_band_vehicle.transition_to_vtol(kTransitionPoint);
    CHECK(!outside_band_result.success);
    CHECK(outside_band_result.message.find("outside the reviewed 15-25 m transition band") != std::string::npos);
    CHECK(outside_band.command_history.empty());

    FakeConnection above_band;
    configure_transition_ready_state(above_band);
    above_band.state->position.relative_altitude_m = 25.5F;
    auto above_band_vehicle = short_vehicle(above_band);
    const auto above_band_result = above_band_vehicle.transition_to_vtol(kTransitionPoint);
    CHECK(!above_band_result.success);
    CHECK(above_band_result.message.find("outside the reviewed 15-25 m transition band") != std::string::npos);
    CHECK(above_band.command_history.empty());

    VtolTransitionFakeConnection altitude_differs_from_recovery_point;
    configure_transition_ready_state(altitude_differs_from_recovery_point);
    altitude_differs_from_recovery_point.state->position.relative_altitude_m = 22.5F;
    auto stable_altitude_vehicle =
        short_vehicle(altitude_differs_from_recovery_point, std::chrono::seconds(3), std::chrono::milliseconds(10));
    const auto stable_altitude_result = stable_altitude_vehicle.transition_to_vtol(kTransitionPoint);
    CHECK(stable_altitude_result.success);
    CHECK(altitude_differs_from_recovery_point.command_history.size() == 1);
    CHECK(altitude_differs_from_recovery_point.state->vtol_state == VtolState::Multicopter);
}

void test_transition_to_vtol_allows_transition_climb_but_keeps_altitude_floor() {
    VtolTransitionFakeConnection climbs_during_transition;
    configure_transition_ready_state(climbs_during_transition);
    climbs_during_transition.post_transition_altitude_m = 25.197F;
    auto climb_vehicle = short_vehicle(climbs_during_transition, std::chrono::seconds(3),
                                       std::chrono::milliseconds(10));
    const auto climbed_result = climb_vehicle.transition_to_vtol(kTransitionPoint);
    CHECK(climbed_result.success);
    CHECK(climbs_during_transition.command_history.size() == 1);
    CHECK(climbs_during_transition.state->position.relative_altitude_m == 25.197F);
    CHECK(climbs_during_transition.state->vtol_state == VtolState::Multicopter);

    VtolTransitionFakeConnection falls_below_floor;
    configure_transition_ready_state(falls_below_floor);
    falls_below_floor.post_transition_altitude_m = 14.9F;
    auto floor_vehicle = short_vehicle(falls_below_floor, std::chrono::seconds(3),
                                       std::chrono::milliseconds(10));
    const auto floor_result = floor_vehicle.transition_to_vtol(kTransitionPoint);
    CHECK(!floor_result.success);
    CHECK(floor_result.message.find("fell below the 15 m transition safety floor") != std::string::npos);
    CHECK(falls_below_floor.command_history.size() == 1);
}

void test_transition_to_vtol_sends_multicopter_target_and_verifies_stability() {
    VtolTransitionFakeConnection connection;
    configure_transition_ready_state(connection);
    connection.state->position.latitude_deg += 0.00045;
    auto vehicle = short_vehicle(connection, std::chrono::seconds(3), std::chrono::milliseconds(10));

    const auto result = vehicle.transition_to_vtol(kTransitionPoint);

    CHECK(result.success);
    CHECK(result.message == "transition to VTOL verified: armed multicopter state is stable");
    CHECK(connection.command_history.size() == 1);
    CHECK(connection.command_history.front().id == 3000);
    CHECK(connection.command_history.front().parameters[0] == 3.0F);
    CHECK(connection.state->custom_mode == 10);
    CHECK(connection.state->armed);
    CHECK(connection.state->vtol_state == VtolState::Multicopter);
}

void test_readiness_dwell_does_not_replace_final_multicopter_stability_dwell() {
    VtolTransitionFakeConnection connection;
    configure_transition_ready_state(connection);
    nomad::vehicle::VehicleConfig config{};
    config.timeouts.transition_state = std::chrono::milliseconds(100);
    config.timeouts.transition_ready_dwell = std::chrono::milliseconds(10);
    nomad::vehicle::Vehicle vehicle(connection, config);
    const auto started_at = std::chrono::steady_clock::now();

    const auto result = vehicle.transition_to_vtol(kTransitionPoint);
    const auto elapsed = std::chrono::steady_clock::now() - started_at;

    CHECK(!result.success);
    CHECK(result.message ==
          "transition to VTOL acknowledgement received but stable multicopter verification timed out");
    CHECK(connection.command_history.size() == 1);
    CHECK(elapsed < std::chrono::milliseconds(500));
}

void test_transition_to_vtol_ack_and_state_are_independent() {
    VtolTransitionFakeConnection missing_ack;
    configure_transition_ready_state(missing_ack);
    missing_ack.acknowledgement.reset();
    auto missing_ack_vehicle = short_vehicle(missing_ack, std::chrono::seconds(2), std::chrono::milliseconds(5));
    const auto missing = missing_ack_vehicle.transition_to_vtol(kTransitionPoint);
    CHECK(!missing.success);
    CHECK(missing.message == "transition to VTOL timed out waiting for acknowledgement");
    CHECK(missing_ack.command_history.size() == 1);

    VtolTransitionFakeConnection denied_ack;
    configure_transition_ready_state(denied_ack);
    denied_ack.acknowledgement = nomad::mavlink::CommandAck{3000, 4};
    auto denied_ack_vehicle = short_vehicle(denied_ack, std::chrono::seconds(2), std::chrono::milliseconds(5));
    const auto denied = denied_ack_vehicle.transition_to_vtol(kTransitionPoint);
    CHECK(!denied.success);
    CHECK(denied.message == "transition to VTOL command was rejected by ArduPilot");

    VtolTransitionFakeConnection ack_only;
    configure_transition_ready_state(ack_only);
    ack_only.behavior = BackTransitionBehavior::NoChange;
    auto ack_only_vehicle = short_vehicle(ack_only, std::chrono::milliseconds(100), std::chrono::milliseconds(5));
    const auto ack_without_change = ack_only_vehicle.transition_to_vtol(kTransitionPoint);
    CHECK(!ack_without_change.success);
    CHECK(ack_without_change.message ==
          "transition to VTOL acknowledgement received but stable multicopter verification timed out");
    CHECK(ack_only.state->vtol_state == VtolState::FixedWing);

    VtolTransitionFakeConnection stale_multicopter;
    configure_transition_ready_state(stale_multicopter);
    stale_multicopter.behavior = BackTransitionBehavior::MulticopterBeforeAck;
    auto stale_vehicle = short_vehicle(stale_multicopter, std::chrono::milliseconds(100), std::chrono::milliseconds(5));
    const auto pre_ack_state = stale_vehicle.transition_to_vtol(kTransitionPoint);
    CHECK(!pre_ack_state.success);
    CHECK(pre_ack_state.message ==
          "transition to VTOL acknowledgement received but stable multicopter verification timed out");
}

void test_transition_to_vtol_accepts_only_safe_state_progression() {
    VtolTransitionFakeConnection intermediate;
    configure_transition_ready_state(intermediate);
    intermediate.behavior = BackTransitionBehavior::IntermediateOnly;
    auto intermediate_vehicle = short_vehicle(intermediate, std::chrono::milliseconds(100),
                                              std::chrono::milliseconds(5));
    const auto intermediate_result = intermediate_vehicle.transition_to_vtol(kTransitionPoint);
    CHECK(!intermediate_result.success);
    CHECK(intermediate.state->vtol_state == VtolState::TransitionToMulticopter);
    CHECK(intermediate.command_history.front().parameters[0] == 3.0F);

    const std::array failure_cases{
        BackTransitionBehavior::ChangeSession,
        BackTransitionBehavior::LoseHeartbeat,
        BackTransitionBehavior::Disarm,
        BackTransitionBehavior::ChangeMode,
    };
    for (const auto behavior : failure_cases) {
        VtolTransitionFakeConnection connection;
        configure_transition_ready_state(connection);
        connection.behavior = behavior;
        auto vehicle = short_vehicle(connection, std::chrono::seconds(2), std::chrono::milliseconds(5));
        const auto result = vehicle.transition_to_vtol(kTransitionPoint);
        CHECK(!result.success);
        CHECK(connection.command_history.size() == 1);
    }
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_transition_to_vtol_requires_quadplane_and_reviewed_ready_state();
        test_transition_to_vtol_enforces_pre_transition_altitude_band();
        test_transition_to_vtol_allows_transition_climb_but_keeps_altitude_floor();
        test_transition_to_vtol_sends_multicopter_target_and_verifies_stability();
        test_readiness_dwell_does_not_replace_final_multicopter_stability_dwell();
        test_transition_to_vtol_ack_and_state_are_independent();
        test_transition_to_vtol_accepts_only_safe_state_progression();
    });
}
