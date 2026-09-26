// SPDX-License-Identifier: Apache-2.0

#include "fake_connection.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "test_harness.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace {

using nomad::telemetry::AircraftClass;
using nomad::telemetry::LandedState;
using nomad::telemetry::VtolState;
using nomad::vehicle::LandingPoint;
using nomad::vehicle::Vehicle;

constexpr LandingPoint kLandingPoint{45.0, -73.0};
constexpr std::array<std::pair<const char *, float>, 12> kLandingParameters{{
    {"Q_ENABLE", 2.0F},         {"Q_FRAME_CLASS", 7.0F},     {"Q_TILT_ENABLE", 1.0F},
    {"Q_TILT_MASK", 3.0F},      {"Q_TILT_TYPE", 0.0F},       {"Q_TILT_RATE_UP", 40.0F},
    {"Q_TILT_MAX", 45.0F},      {"Q_ASSIST_SPEED", 6.0F},    {"Q_OPTIONS", 0.0F},
    {"Q_LAND_FINAL_SPD", 0.5F}, {"Q_LAND_FINAL_ALT", 6.0F}, {"Q_LAND_ALTCHG", 0.2F},
}};

enum class LandingBehavior { SuccessfulLanding, AckWithoutDescent, DescentWithoutTouchdown, UnstableReadiness,
                              UnstableTouchdown, GroundBeforeAck, StaleTouchdown, SessionChangeBeforeSend,
                              SessionChange, LinkLoss, WrongMode, WrongVtol, EarlyDisarm };

class QuadplaneLandingConnection final : public FakeConnection {
  public:
    LandingBehavior behavior{LandingBehavior::SuccessfulLanding};

    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                           std::uint64_t expected_session_id,
                                                           std::chrono::milliseconds timeout) override {
        if (command.id == 176 && behavior == LandingBehavior::SessionChangeBeforeSend) {
            ++state->session_id;
        }
        if (expected_session_id == 0 || state->session_id != expected_session_id) {
            return std::nullopt;
        }
        return send_command(command, timeout);
    }

    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                           std::chrono::milliseconds timeout) override {
        if (command.id == 176 && behavior == LandingBehavior::GroundBeforeAck) {
            auto_stamp_fresh_fields = false;
            state->landed_state = LandedState::OnGround;
            state->landed_state_updated_at = std::chrono::steady_clock::now();
        }
        const auto ack = FakeConnection::send_command(command, timeout);
        if (command.id != 176) {
            return ack;
        }
        landing_started = true;
        switch (behavior) {
        case LandingBehavior::SessionChange: ++state->session_id; break;
        case LandingBehavior::LinkLoss:
            state->connected = false;
            state->heartbeat_fresh = false;
            break;
        case LandingBehavior::WrongMode: state->custom_mode = 21; break;
        case LandingBehavior::WrongVtol:
            state->vtol_state = VtolState::FixedWing;
            break;
        case LandingBehavior::EarlyDisarm:
            state->armed = false;
            break;
        default: break;
        }
        return ack;
    }

    std::optional<nomad::telemetry::VehicleState> wait_for_state(std::chrono::milliseconds timeout) override {
        std::this_thread::sleep_for(std::min(timeout, std::chrono::milliseconds(5)));
        const auto result = FakeConnection::wait_for_state(timeout);
        if (!landing_started) {
            if (behavior == LandingBehavior::UnstableReadiness) {
                state->position.relative_altitude_m = readiness_sample_++ % 2 == 0 ? 20.0F : 22.0F;
                state->position_updated_at = std::chrono::steady_clock::now();
                state->velocity_updated_at = state->position_updated_at;
                return get_state();
            }
            return result;
        }
        observe_landing_sample();
        return get_state();
    }

  private:
    bool landing_started{false};
    std::size_t landing_sample_{0};
    std::size_t readiness_sample_{0};

    void refresh_post_command_sample() {
        const auto now = std::chrono::steady_clock::now();
        state->position_updated_at = now;
        state->velocity_updated_at = now;
        state->gps_updated_at = now;
        state->vtol_state_updated_at = now;
        if (behavior != LandingBehavior::GroundBeforeAck && behavior != LandingBehavior::StaleTouchdown) {
            state->landed_state_updated_at = now;
        }
    }

    void observe_landing_sample() {
        ++landing_sample_;
        if (behavior == LandingBehavior::AckWithoutDescent) {
            refresh_post_command_sample();
            return;
        }
        if (behavior == LandingBehavior::GroundBeforeAck) {
            refresh_post_command_sample();
            state->position.relative_altitude_m = 0.5F;
            state->velocity = {};
            state->armed = false;
            return;
        }
        if (behavior == LandingBehavior::DescentWithoutTouchdown ||
            behavior == LandingBehavior::UnstableTouchdown) {
            if (behavior == LandingBehavior::UnstableTouchdown && landing_sample_ >= 7) {
                state->position.relative_altitude_m = 0.8F;
                state->landed_state = LandedState::OnGround;
                state->armed = false;
                state->velocity.groundspeed_mps = 0.8F;
                refresh_post_command_sample();
                return;
            }
            state->position.relative_altitude_m =
                landing_sample_ < 6 ? 20.0F - static_cast<float>(landing_sample_) * 2.0F : 8.0F;
            state->landed_state = LandedState::InAir;
            refresh_post_command_sample();
            return;
        }

        if (behavior == LandingBehavior::StaleTouchdown) {
            state->position.relative_altitude_m = 0.8F;
            state->landed_state = LandedState::OnGround;
            state->armed = false;
            refresh_post_command_sample();
            state->landed_state_updated_at = std::chrono::steady_clock::now() - std::chrono::seconds(5);
            return;
        }

        const auto altitude = landing_sample_ < 7 ? 20.0F - static_cast<float>(landing_sample_) * 3.0F : 0.8F;
        state->position.relative_altitude_m = altitude;
        state->velocity = {};
        if (altitude <= 1.5F) {
            state->landed_state = LandedState::OnGround;
            state->armed = false;
        } else {
            state->landed_state = LandedState::InAir;
        }
        refresh_post_command_sample();
    }
};

void configure_landing_ready_state(FakeConnection &connection) {
    connection.connect();
    for (const auto &[name, value] : kLandingParameters) {
        connection.parameters[name] = value;
    }
    auto &state = *connection.state;
    state.identity = {nomad::telemetry::kArduPilotAutopilot, nomad::telemetry::kFixedWing, AircraftClass::QuadPlane};
    state.connected = true;
    state.heartbeat_fresh = true;
    state.armed = true;
    state.system_id = 1;
    state.component_id = 1;
    state.custom_mode = 10;
    state.position = {kLandingPoint.latitude_deg, kLandingPoint.longitude_deg, 120.0F, 20.0F};
    state.position_valid = true;
    state.position_updated_at = std::chrono::steady_clock::now();
    state.velocity = {};
    state.velocity_updated_at = std::chrono::steady_clock::now();
    state.gps = {3, 12};
    state.gps_valid = true;
    state.gps_updated_at = std::chrono::steady_clock::now();
    state.vtol_state = VtolState::Multicopter;
    state.vtol_state_valid = true;
    state.vtol_state_updated_at = std::chrono::steady_clock::now();
    state.landed_state = LandedState::InAir;
    state.landed_state_valid = true;
    state.landed_state_updated_at = std::chrono::steady_clock::now();
}

Vehicle short_vehicle(FakeConnection &connection, std::chrono::milliseconds timeout = std::chrono::milliseconds(120),
                     std::chrono::milliseconds dwell = std::chrono::milliseconds(15)) {
    nomad::vehicle::VehicleConfig config{};
    config.timeouts.transition_ready_dwell = dwell;
    config.timeouts.quadplane_landing = timeout;
    return Vehicle(connection, config);
}

void test_valid_landing_requires_command_and_physical_post_ack_evidence() {
    QuadplaneLandingConnection connection;
    configure_landing_ready_state(connection);
    auto vehicle = short_vehicle(connection, std::chrono::milliseconds(1000), std::chrono::milliseconds(20));
    const auto result = vehicle.quadplane_vtol_land(kLandingPoint);

    if (!result.success) {
        throw std::runtime_error(result.message);
    }
    CHECK(result.success);
    CHECK(connection.command_history.size() == 1);
    CHECK(connection.last_command.id == 176);
    CHECK(connection.last_command.parameters[0] == 1.0F);
    CHECK(connection.last_command.parameters[1] == 20.0F);
    CHECK(connection.last_command.parameters[2] == 0.0F);
    CHECK(connection.last_command.parameters[3] == 0.0F);
    CHECK(connection.last_command.parameters[4] == 0.0F);
    CHECK(connection.last_command.parameters[5] == 0.0F);
    CHECK(connection.last_command.parameters[6] == 0.0F);
    CHECK(connection.state->landed_state == LandedState::OnGround);
    CHECK(!connection.state->armed);
    CHECK(connection.version_read_count == 1);
}

void test_zero_landing_budget_rejects_before_profile_read_or_command() {
    QuadplaneLandingConnection connection;
    configure_landing_ready_state(connection);
    auto vehicle = short_vehicle(connection, std::chrono::milliseconds::zero(), std::chrono::milliseconds(10));
    const auto started = std::chrono::steady_clock::now();

    const auto result = vehicle.quadplane_vtol_land(kLandingPoint);
    const auto elapsed = std::chrono::steady_clock::now() - started;

    CHECK(!result.success);
    CHECK(result.message == "QuadPlane VTOL landing rejected: landing deadline and readiness dwell must be positive");
    CHECK(elapsed < std::chrono::milliseconds(500));
    CHECK(connection.version_read_count == 0);
    CHECK(connection.command_history.empty());
}

void test_unsupported_aircraft_and_wrong_profile_never_send_landing_command() {
    constexpr std::array unsupported_classes{AircraftClass::Copter, AircraftClass::Plane, AircraftClass::Unknown};
    for (const auto aircraft_class : unsupported_classes) {
        FakeConnection connection;
        configure_landing_ready_state(connection);
        connection.state->identity.aircraft_class = aircraft_class;
        auto vehicle = short_vehicle(connection);
        CHECK(!vehicle.quadplane_vtol_land(kLandingPoint).success);
        CHECK(connection.command_history.empty());
        CHECK(connection.version_read_count == 0);
    }

    for (const auto &[name, unused_value] : kLandingParameters) {
        (void)unused_value;
        FakeConnection connection;
        configure_landing_ready_state(connection);
        connection.parameters[name] = 99.0F;
        auto vehicle = short_vehicle(connection);
        CHECK(!vehicle.quadplane_vtol_land(kLandingPoint).success);
        CHECK(connection.command_history.empty());
    }

    FakeConnection incompatible_version;
    configure_landing_ready_state(incompatible_version);
    incompatible_version.autopilot_version = nomad::mavlink::AutopilotVersion{4, 6, 1, "dbe79216"};
    auto version_vehicle = short_vehicle(incompatible_version);
    CHECK(!version_vehicle.quadplane_vtol_land(kLandingPoint).success);
    CHECK(incompatible_version.command_history.empty());

    FakeConnection wrong_firmware_hash;
    configure_landing_ready_state(wrong_firmware_hash);
    wrong_firmware_hash.autopilot_version = nomad::mavlink::AutopilotVersion{4, 7, 1, "00000000"};
    auto hash_vehicle = short_vehicle(wrong_firmware_hash);
    CHECK(!hash_vehicle.quadplane_vtol_land(kLandingPoint).success);
    CHECK(wrong_firmware_hash.command_history.empty());
}

void test_initial_state_and_telemetry_fail_closed() {
    using namespace std::chrono_literals;
    const auto stale = std::chrono::steady_clock::now() - 5s;
    for (int failure = 0; failure < 15; ++failure) {
        FakeConnection connection;
        configure_landing_ready_state(connection);
        switch (failure) {
        case 0: connection.state->vtol_state = VtolState::FixedWing; break;
        case 1: connection.state->armed = false; break;
        case 2: connection.state->custom_mode = 15; break;
        case 3: connection.state->heartbeat_fresh = false; break;
        case 4: connection.state->position_valid = false; break;
        case 5: connection.state->position_updated_at = stale; break;
        case 6: connection.state->velocity_updated_at = stale; break;
        case 7: connection.state->vtol_state_updated_at = stale; break;
        case 8: connection.state->landed_state_updated_at = stale; break;
        case 9: connection.state->gps.fix_type = 2; break;
        case 10: connection.state->gps_valid = false; break;
        case 11: connection.state->position.relative_altitude_m = 12.0F; break;
        case 12: connection.state->velocity.groundspeed_mps = 1.5F; break;
        case 13: connection.state->connected = false; break;
        case 14: connection.state->gps_updated_at = stale; break;
        }
        connection.auto_stamp_fresh_fields = false;
        auto vehicle = short_vehicle(connection);
        CHECK(!vehicle.quadplane_vtol_land(kLandingPoint).success);
        CHECK(connection.command_history.empty());
    }

    for (const auto [system_id, component_id] : {std::pair{0, 1}, std::pair{1, 2}}) {
        FakeConnection connection;
        configure_landing_ready_state(connection);
        connection.state->system_id = static_cast<std::uint8_t>(system_id);
        connection.state->component_id = static_cast<std::uint8_t>(component_id);
        auto vehicle = short_vehicle(connection);
        CHECK(!vehicle.quadplane_vtol_land(kLandingPoint).success);
        CHECK(connection.command_history.empty());
    }

    FakeConnection missing_parameter;
    configure_landing_ready_state(missing_parameter);
    missing_parameter.parameters.erase("Q_LAND_ALTCHG");
    auto missing_vehicle = short_vehicle(missing_parameter);
    CHECK(!missing_vehicle.quadplane_vtol_land(kLandingPoint).success);
    CHECK(missing_parameter.command_history.empty());

    FakeConnection outside_region;
    configure_landing_ready_state(outside_region);
    outside_region.state->position.longitude_deg += 0.0001;
    auto outside_vehicle = short_vehicle(outside_region);
    CHECK(!outside_vehicle.quadplane_vtol_land(kLandingPoint).success);
    CHECK(outside_region.command_history.empty());

    FakeConnection excessive_climb;
    configure_landing_ready_state(excessive_climb);
    excessive_climb.state->velocity.climb_rate_mps = 0.5F;
    auto climb_vehicle = short_vehicle(excessive_climb);
    CHECK(!climb_vehicle.quadplane_vtol_land(kLandingPoint).success);
    CHECK(excessive_climb.command_history.empty());

    FakeConnection changed_session;
    configure_landing_ready_state(changed_session);
    changed_session.change_session_after_param_read = true;
    auto session_vehicle = short_vehicle(changed_session);
    CHECK(!session_vehicle.quadplane_vtol_land(kLandingPoint).success);
    CHECK(changed_session.command_history.empty());

    QuadplaneLandingConnection session_before_send;
    configure_landing_ready_state(session_before_send);
    session_before_send.behavior = LandingBehavior::SessionChangeBeforeSend;
    auto changed_at_send_vehicle = short_vehicle(session_before_send, std::chrono::milliseconds(200),
                                                 std::chrono::milliseconds(10));
    CHECK(!changed_at_send_vehicle.quadplane_vtol_land(kLandingPoint).success);
    CHECK(session_before_send.command_history.empty());
}

void test_readiness_requires_stable_fresh_samples_and_missing_ack_fails() {
    QuadplaneLandingConnection unstable;
    configure_landing_ready_state(unstable);
    unstable.behavior = LandingBehavior::UnstableReadiness;
    auto unstable_vehicle = short_vehicle(unstable, std::chrono::milliseconds(80), std::chrono::milliseconds(20));
    CHECK(!unstable_vehicle.quadplane_vtol_land(kLandingPoint).success);
    CHECK(unstable.command_history.empty());

    FakeConnection missing_ack;
    configure_landing_ready_state(missing_ack);
    missing_ack.acknowledgement.reset();
    auto missing_ack_vehicle = short_vehicle(missing_ack, std::chrono::milliseconds(300),
                                             std::chrono::milliseconds(10));
    CHECK(!missing_ack_vehicle.quadplane_vtol_land(kLandingPoint).success);
    CHECK(missing_ack.command_history.size() == 1);
    CHECK(missing_ack.last_command.id == 176);

    FakeConnection denied_ack;
    configure_landing_ready_state(denied_ack);
    denied_ack.acknowledgement = nomad::mavlink::CommandAck{176, 2};
    auto denied_vehicle = short_vehicle(denied_ack);
    CHECK(!denied_vehicle.quadplane_vtol_land(kLandingPoint).success);
    CHECK(denied_ack.command_history.size() == 1);
}

void test_ack_without_descent_or_descent_without_touchdown_never_succeed() {
    for (const auto behavior : {LandingBehavior::AckWithoutDescent, LandingBehavior::DescentWithoutTouchdown,
                                LandingBehavior::UnstableTouchdown}) {
        QuadplaneLandingConnection connection;
        configure_landing_ready_state(connection);
        connection.behavior = behavior;
        auto vehicle = short_vehicle(connection, std::chrono::milliseconds(150), std::chrono::milliseconds(10));
        CHECK(!vehicle.quadplane_vtol_land(kLandingPoint).success);
        CHECK(connection.command_history.size() == 1);
        CHECK(connection.state->custom_mode == 20);
    }
}

void test_pre_ack_touchdown_and_post_ack_state_changes_fail_closed() {
    for (const auto behavior : {LandingBehavior::GroundBeforeAck, LandingBehavior::StaleTouchdown,
                                LandingBehavior::SessionChange,
                                LandingBehavior::LinkLoss, LandingBehavior::WrongMode, LandingBehavior::WrongVtol,
                                LandingBehavior::EarlyDisarm}) {
        QuadplaneLandingConnection connection;
        configure_landing_ready_state(connection);
        connection.behavior = behavior;
        auto vehicle = short_vehicle(connection, std::chrono::milliseconds(150), std::chrono::milliseconds(10));
        CHECK(!vehicle.quadplane_vtol_land(kLandingPoint).success);
        CHECK(connection.command_history.size() == 1);
    }
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_valid_landing_requires_command_and_physical_post_ack_evidence();
        test_zero_landing_budget_rejects_before_profile_read_or_command();
        test_unsupported_aircraft_and_wrong_profile_never_send_landing_command();
        test_initial_state_and_telemetry_fail_closed();
        test_readiness_requires_stable_fresh_samples_and_missing_ack_fails();
        test_ack_without_descent_or_descent_without_touchdown_never_succeed();
        test_pre_ack_touchdown_and_post_ack_state_changes_fail_closed();
    });
}
