// SPDX-License-Identifier: Apache-2.0
// QuadPlane fixed-wing-to-VTOL admission and stable-state verification.

#include "nomad/vehicle/vehicle.hpp"

#include "command_ids.hpp"
#include "geo.hpp"
#include "state_time.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>

namespace nomad::vehicle {
namespace {

using Clock = std::chrono::steady_clock;

constexpr double kTransitionRegionRadiusMeters = 55.0;
constexpr double kReadyMaxGroundspeedMps = 28.0;
constexpr double kReadyMaxGroundspeedVariationMps = 3.0;
constexpr double kReadyMaxClimbRateMps = 1.0;
constexpr double kReadyMaxRadialVariationMeters = 8.0;
constexpr float kTransitionAltitudeMinimumMeters = 15.0F;
constexpr float kTransitionAltitudeMaximumMeters = 25.0F;
constexpr float kReadyMaxAltitudeVariationMeters = 1.0F;
constexpr float kVtolMaxGroundspeedMps = 3.0F;
constexpr float kVtolMaxClimbRateMps = 0.5F;
constexpr double kVtolMaxPositionSpreadMeters = 4.0;
constexpr std::size_t kReadySampleCount = 5;
constexpr std::size_t kVtolSampleCount = 3;
constexpr std::size_t kVtolStateSampleCount = 2;
constexpr auto kVtolStateFreshnessTimeout = std::chrono::seconds(3);
constexpr auto kPollTimeout = std::chrono::milliseconds(50);
constexpr auto kAckTimeout = std::chrono::seconds(3);
constexpr auto kProfileReadbackTimeout = std::chrono::seconds(3);
// The pinned tiltrotor profile raises motors at 40 degrees per second; two
// seconds allows the tilt actuator to reach its configured VTOL limit.
constexpr auto kVtolStableDwell = std::chrono::seconds(2);

struct PinnedParameter {
    const char *name;
    float value;
};

constexpr std::array<PinnedParameter, 7> kPinnedTransitionParameters{{
    {"Q_ENABLE", 2.0F},
    {"Q_FRAME_CLASS", 7.0F},
    {"Q_TILT_ENABLE", 1.0F},
    {"Q_TILT_MASK", 3.0F},
    {"Q_TILT_TYPE", 0.0F},
    {"Q_TILT_RATE_UP", 40.0F},
    {"Q_TILT_MAX", 45.0F},
}};

struct TransitionReadyWindow {
    std::size_t sample_count{};
    Clock::time_point first_sample_at{};
    Clock::time_point last_sample_at{};
    double minimum_distance_m{};
    double maximum_distance_m{};
    float minimum_altitude_m{};
    float maximum_altitude_m{};
    float minimum_groundspeed_mps{};
    float maximum_groundspeed_mps{};
};

struct StableVtolWindow {
    std::size_t sample_count{};
    Clock::time_point first_sample_at{};
    Clock::time_point last_sample_at{};
    double anchor_latitude_deg{};
    double anchor_longitude_deg{};
    double maximum_anchor_distance_m{};
    float minimum_altitude_m{};
    float maximum_altitude_m{};
};

double distance_to_point(const telemetry::VehicleState &state, const RecoveryPoint &point) {
    return detail::distance_m(state.position.latitude_deg, state.position.longitude_deg,
                              point.latitude_deg, point.longitude_deg);
}

std::optional<std::string> validate_point(const RecoveryPoint &point,
                                          const safety::GlobalFencePolicy &fence_policy) {
    if (!std::isfinite(point.latitude_deg) || point.latitude_deg < -90.0 || point.latitude_deg > 90.0 ||
        !std::isfinite(point.longitude_deg) || point.longitude_deg < -180.0 || point.longitude_deg > 180.0 ||
        (point.latitude_deg == 0.0 && point.longitude_deg == 0.0)) {
        return "transition point coordinates are invalid";
    }
    if (!std::isfinite(point.relative_altitude_m) || point.relative_altitude_m < kTransitionAltitudeMinimumMeters ||
        point.relative_altitude_m > kTransitionAltitudeMaximumMeters) {
        return "transition altitude must be between 15 and 25 m above home";
    }
    const auto fence = safety::evaluate_global_position(fence_policy, {point.latitude_deg, point.longitude_deg});
    if (!fence.allowed) {
        return fence.message;
    }
    return {};
}

std::optional<std::string> transition_state_error(const telemetry::VehicleState &state, std::uint64_t expected_session,
                                                  std::uint8_t expected_system_id, std::uint8_t expected_component_id,
                                                  std::chrono::milliseconds telemetry_freshness) {
    if (state.identity.aircraft_class != telemetry::AircraftClass::QuadPlane ||
        state.identity.autopilot_type != telemetry::kArduPilotAutopilot ||
        state.identity.vehicle_type != telemetry::kFixedWing) {
        return "pinned ArduPilot QuadPlane identity changed";
    }
    if (expected_session == 0 || state.session_id != expected_session) {
        return "vehicle session changed";
    }
    if (!state.connected || !state.heartbeat_fresh || state.system_id == 0 || state.system_id != expected_system_id ||
        state.component_id == 0 || state.component_id != expected_component_id) {
        return "heartbeat is stale";
    }
    if (!state.armed) {
        return "vehicle disarmed";
    }
    if (!telemetry::is_auto_mode(telemetry::AircraftClass::QuadPlane, state.custom_mode)) {
        return "AUTO mode is required by the pinned transition handler";
    }
    const auto now = Clock::now();
    if (!state.position_valid || !detail::timestamp_is_fresh(state.position_updated_at, telemetry_freshness, now)) {
        return "position is invalid or stale";
    }
    if (!std::isfinite(state.position.latitude_deg) || state.position.latitude_deg < -90.0 ||
        state.position.latitude_deg > 90.0 || !std::isfinite(state.position.longitude_deg) ||
        state.position.longitude_deg < -180.0 || state.position.longitude_deg > 180.0 ||
        !std::isfinite(state.position.relative_altitude_m)) {
        return "position is invalid";
    }
    if (!state.gps_valid || state.gps.fix_type < 3 || state.gps.satellites == 0 ||
        !detail::timestamp_is_fresh(state.gps_updated_at, telemetry_freshness, now)) {
        return "a fresh 3D GPS fix is required";
    }
    if (!state.vtol_state_valid ||
        !detail::timestamp_is_fresh(state.vtol_state_updated_at,
                                    std::chrono::duration_cast<std::chrono::milliseconds>(kVtolStateFreshnessTimeout),
                                    now)) {
        return "VTOL state is unavailable or stale";
    }
    if (!detail::timestamp_is_fresh(state.velocity_updated_at, telemetry_freshness, now) ||
        !std::isfinite(state.velocity.groundspeed_mps) || !std::isfinite(state.velocity.climb_rate_mps)) {
        return "velocity telemetry is unavailable or stale";
    }
    return {};
}

std::optional<std::string> transition_ready_state_error(
    const telemetry::VehicleState &state, std::uint64_t expected_session, std::uint8_t expected_system_id,
    std::uint8_t expected_component_id, std::chrono::milliseconds telemetry_freshness) {
    if (const auto error = transition_state_error(state, expected_session, expected_system_id, expected_component_id,
                                                  telemetry_freshness);
        error.has_value()) {
        return error;
    }
    if (state.position.relative_altitude_m < kTransitionAltitudeMinimumMeters ||
        state.position.relative_altitude_m > kTransitionAltitudeMaximumMeters) {
        return "aircraft altitude " + std::to_string(state.position.relative_altitude_m) +
               " m is outside the reviewed 15-25 m transition band";
    }
    return {};
}

std::optional<std::string> multicopter_verification_state_error(
    const telemetry::VehicleState &state, std::uint64_t expected_session, std::uint8_t expected_system_id,
    std::uint8_t expected_component_id, std::chrono::milliseconds telemetry_freshness) {
    if (const auto error = transition_state_error(state, expected_session, expected_system_id, expected_component_id,
                                                  telemetry_freshness);
        error.has_value()) {
        return error;
    }
    if (state.position.relative_altitude_m < kTransitionAltitudeMinimumMeters) {
        return "aircraft altitude " + std::to_string(state.position.relative_altitude_m) +
               " m fell below the 15 m transition safety floor";
    }
    return {};
}

bool is_inside_transition_region(const telemetry::VehicleState &state, const RecoveryPoint &point) {
    return distance_to_point(state, point) <= kTransitionRegionRadiusMeters;
}

bool is_stable_ready_sample(const telemetry::VehicleState &state, const RecoveryPoint &point) {
    return is_inside_transition_region(state, point) &&
           state.velocity.groundspeed_mps <= kReadyMaxGroundspeedMps &&
           std::abs(state.velocity.climb_rate_mps) <= kReadyMaxClimbRateMps;
}

void start_ready_window(TransitionReadyWindow &window, const telemetry::VehicleState &state,
                        const RecoveryPoint &point) {
    const auto distance = distance_to_point(state, point);
    window = {1, state.position_updated_at, state.position_updated_at, distance, distance,
              state.position.relative_altitude_m, state.position.relative_altitude_m,
              state.velocity.groundspeed_mps, state.velocity.groundspeed_mps};
}

void add_ready_sample(TransitionReadyWindow &window, const telemetry::VehicleState &state,
                      const RecoveryPoint &point) {
    if (!is_stable_ready_sample(state, point)) {
        window = {};
        return;
    }
    if (window.sample_count > 0 && state.position_updated_at <= window.last_sample_at) {
        return;
    }
    if (window.sample_count == 0) {
        start_ready_window(window, state, point);
        return;
    }
    const auto distance = distance_to_point(state, point);
    const auto minimum_altitude = std::min(window.minimum_altitude_m, state.position.relative_altitude_m);
    const auto maximum_altitude = std::max(window.maximum_altitude_m, state.position.relative_altitude_m);
    const auto minimum_distance = std::min(window.minimum_distance_m, distance);
    const auto maximum_distance = std::max(window.maximum_distance_m, distance);
    const auto minimum_groundspeed = std::min(window.minimum_groundspeed_mps, state.velocity.groundspeed_mps);
    const auto maximum_groundspeed = std::max(window.maximum_groundspeed_mps, state.velocity.groundspeed_mps);
    if (maximum_altitude - minimum_altitude > kReadyMaxAltitudeVariationMeters ||
        maximum_distance - minimum_distance > kReadyMaxRadialVariationMeters ||
        maximum_groundspeed - minimum_groundspeed > kReadyMaxGroundspeedVariationMps) {
        start_ready_window(window, state, point);
        return;
    }
    ++window.sample_count;
    window.last_sample_at = state.position_updated_at;
    window.minimum_altitude_m = minimum_altitude;
    window.maximum_altitude_m = maximum_altitude;
    window.minimum_distance_m = minimum_distance;
    window.maximum_distance_m = maximum_distance;
    window.minimum_groundspeed_mps = minimum_groundspeed;
    window.maximum_groundspeed_mps = maximum_groundspeed;
}

bool is_transition_ready(const TransitionReadyWindow &window, std::chrono::milliseconds dwell) {
    return window.sample_count >= kReadySampleCount && window.last_sample_at - window.first_sample_at >= dwell;
}

bool is_stable_vtol_sample(const telemetry::VehicleState &state, const RecoveryPoint &point) {
    return is_inside_transition_region(state, point) &&
           state.velocity.groundspeed_mps <= kVtolMaxGroundspeedMps &&
           std::abs(state.velocity.climb_rate_mps) <= kVtolMaxClimbRateMps;
}

void start_stable_vtol_window(StableVtolWindow &window, const telemetry::VehicleState &state) {
    window = {1, state.position_updated_at, state.position_updated_at, state.position.latitude_deg,
              state.position.longitude_deg, 0.0, state.position.relative_altitude_m,
              state.position.relative_altitude_m};
}

void add_stable_vtol_position(StableVtolWindow &window, const telemetry::VehicleState &state,
                              const RecoveryPoint &point) {
    if (!is_stable_vtol_sample(state, point)) {
        window = {};
        return;
    }
    if (window.sample_count > 0 && state.position_updated_at <= window.last_sample_at) {
        return;
    }
    if (window.sample_count == 0) {
        start_stable_vtol_window(window, state);
        return;
    }
    const auto distance_from_anchor = detail::distance_m(window.anchor_latitude_deg, window.anchor_longitude_deg,
                                                         state.position.latitude_deg,
                                                         state.position.longitude_deg);
    const auto minimum_altitude = std::min(window.minimum_altitude_m, state.position.relative_altitude_m);
    const auto maximum_altitude = std::max(window.maximum_altitude_m, state.position.relative_altitude_m);
    if (distance_from_anchor > kVtolMaxPositionSpreadMeters / 2.0 ||
        maximum_altitude - minimum_altitude > kReadyMaxAltitudeVariationMeters) {
        start_stable_vtol_window(window, state);
        return;
    }
    ++window.sample_count;
    window.last_sample_at = state.position_updated_at;
    window.maximum_anchor_distance_m = std::max(window.maximum_anchor_distance_m, distance_from_anchor);
    window.minimum_altitude_m = minimum_altitude;
    window.maximum_altitude_m = maximum_altitude;
}

bool is_stable_vtol(const StableVtolWindow &window, std::size_t fresh_vtol_states) {
    return window.sample_count >= kVtolSampleCount && fresh_vtol_states >= kVtolStateSampleCount &&
           window.last_sample_at - window.first_sample_at >= kVtolStableDwell &&
           window.maximum_anchor_distance_m <= kVtolMaxPositionSpreadMeters / 2.0;
}

} // namespace

CommandResult Vehicle::transition_to_vtol(const RecoveryPoint &point) {
    const auto admission = require_operation(VehicleOperation::TransitionToVtol);
    if (!admission.success) {
        return admission;
    }
    if (const auto error = validate_point(point, fence_policy_); error.has_value()) {
        return {false, "transition to VTOL rejected: " + *error};
    }
    if (transition_state_timeout_ <= std::chrono::milliseconds::zero() ||
        transition_ready_dwell_ <= std::chrono::milliseconds::zero()) {
        return {false, "transition to VTOL rejected: transition deadline and stabilization dwell must be positive"};
    }

    const auto deadline = Clock::now() + transition_state_timeout_;
    const auto initial = connection_.get_state();
    const auto session_id = initial.session_id;
    const auto system_id = initial.system_id;
    const auto component_id = initial.component_id;
    if (const auto error = transition_ready_state_error(initial, session_id, system_id, component_id,
                                                       position_freshness_timeout_);
        error.has_value()) {
        return {false, "transition to VTOL rejected: " + *error};
    }
    if (initial.vtol_state != telemetry::VtolState::FixedWing) {
        return {false, "transition to VTOL rejected: authoritative VTOL state is not fixed wing"};
    }

    const auto profile = verify_transition_profile(session_id, system_id, component_id, deadline);
    if (!profile.success) {
        return profile;
    }

    const auto ready = wait_for_transition_ready(point, session_id, system_id, component_id, deadline);
    if (!ready.success) {
        return ready;
    }
    if (Clock::now() >= deadline) {
        return {false, "transition to VTOL timed out before command transmission"};
    }

    const auto command = make_command(
        kQuadplaneTransitionCommand,
        {static_cast<float>(static_cast<std::uint8_t>(telemetry::VtolState::Multicopter)), 0, 0, 0, 0, 0, 0});
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
    const auto ack_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(kAckTimeout);
    const auto ack = connection_.send_command(command, std::min(remaining, ack_timeout));
    if (!ack.has_value()) {
        return {false, "transition to VTOL timed out waiting for acknowledgement"};
    }
    if (ack->command != command.id) {
        return {false, "transition to VTOL received an acknowledgement for a different command"};
    }
    if (ack->result != kAcceptedResult) {
        return {false, "transition to VTOL command was rejected by ArduPilot"};
    }

    const auto ack_state = connection_.get_state();
    const auto ack_boundary = std::max(Clock::now(), ack_state.vtol_state_updated_at);
    return wait_for_multicopter_state(point, session_id, system_id, component_id, ack_boundary, deadline);
}

CommandResult Vehicle::verify_transition_profile(std::uint64_t session_id, std::uint8_t system_id,
                                                 std::uint8_t component_id,
                                                 Clock::time_point deadline) {
    const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(kProfileReadbackTimeout);
    for (const auto &parameter : kPinnedTransitionParameters) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
        if (remaining <= std::chrono::milliseconds::zero()) {
            return {false, "transition to VTOL timed out during profile readback"};
        }
        const auto value = connection_.read_param(parameter.name, std::min(remaining, timeout));
        if (!value.has_value() || *value != parameter.value) {
            return {false, "transition to VTOL rejected: pinned profile requires " + std::string(parameter.name) +
                               "=" + std::to_string(parameter.value)};
        }
    }

    const auto state = connection_.get_state();
    if (const auto error = transition_ready_state_error(state, session_id, system_id, component_id,
                                                       position_freshness_timeout_);
        error.has_value()) {
        return {false, "transition to VTOL rejected after profile readback: " + *error};
    }
    if (state.vtol_state != telemetry::VtolState::FixedWing) {
        return {false,
                "transition to VTOL rejected after profile readback: authoritative VTOL state is not "
                "fixed wing"};
    }
    return {true, "pinned QuadPlane profile verified"};
}

CommandResult Vehicle::wait_for_transition_ready(const RecoveryPoint &point, std::uint64_t session_id,
                                                 std::uint8_t system_id, std::uint8_t component_id,
                                                 Clock::time_point deadline) {
    TransitionReadyWindow samples;
    while (Clock::now() < deadline) {
        const auto update = connection_.wait_for_state(kPollTimeout);
        const auto sample = update.value_or(connection_.get_state());
        if (const auto error = transition_ready_state_error(sample, session_id, system_id, component_id,
                                                           position_freshness_timeout_);
            error.has_value()) {
            return {false, "transition to VTOL readiness failed: " + *error};
        }
        if (sample.vtol_state != telemetry::VtolState::FixedWing) {
            return {false, "transition to VTOL readiness failed: authoritative VTOL state is not fixed wing"};
        }
        add_ready_sample(samples, sample, point);
        if (is_transition_ready(samples, transition_ready_dwell_)) {
            return {true, "transition-ready state stabilized"};
        }
    }
    return {false, "transition to VTOL rejected: transition-ready stabilization timed out"};
}

CommandResult Vehicle::wait_for_multicopter_state(const RecoveryPoint &point, std::uint64_t session_id,
                                                  std::uint8_t system_id, std::uint8_t component_id,
                                                  Clock::time_point ack_boundary, Clock::time_point deadline) {
    StableVtolWindow stable_positions;
    auto last_vtol_state_at = ack_boundary;
    std::size_t fresh_vtol_states = 0;
    while (Clock::now() < deadline) {
        const auto update = connection_.wait_for_state(kPollTimeout);
        const auto sample = update.value_or(connection_.get_state());
        if (const auto error = multicopter_verification_state_error(sample, session_id, system_id, component_id,
                                                                    position_freshness_timeout_);
            error.has_value()) {
            return {false, "transition to VTOL verification failed: " + *error};
        }
        if (sample.vtol_state != telemetry::VtolState::FixedWing &&
            sample.vtol_state != telemetry::VtolState::TransitionToMulticopter &&
            sample.vtol_state != telemetry::VtolState::Multicopter) {
            return {false, "transition to VTOL verification failed: unexpected VTOL state"};
        }
        if (sample.vtol_state_updated_at > last_vtol_state_at) {
            last_vtol_state_at = sample.vtol_state_updated_at;
            if (sample.vtol_state == telemetry::VtolState::Multicopter) {
                ++fresh_vtol_states;
            }
        }
        if (sample.vtol_state != telemetry::VtolState::Multicopter) {
            stable_positions = {};
            continue;
        }
        if (sample.vtol_state_updated_at <= ack_boundary || sample.position_updated_at <= ack_boundary) {
            stable_positions = {};
            continue;
        }
        add_stable_vtol_position(stable_positions, sample, point);
        if (!is_stable_vtol(stable_positions, fresh_vtol_states)) {
            continue;
        }
        return verify_final_multicopter_state(point, session_id, system_id, component_id, ack_boundary);
    }
    return {false, "transition to VTOL acknowledgement received but stable multicopter verification timed out"};
}

CommandResult Vehicle::verify_final_multicopter_state(const RecoveryPoint &point, std::uint64_t session_id,
                                                      std::uint8_t system_id, std::uint8_t component_id,
                                                      Clock::time_point ack_boundary) const {
    const auto final = connection_.get_state();
    if (const auto error = multicopter_verification_state_error(final, session_id, system_id, component_id,
                                                                position_freshness_timeout_);
        error.has_value()) {
        return {false, "transition to VTOL verification failed: " + *error};
    }
    if (final.vtol_state != telemetry::VtolState::Multicopter || final.vtol_state_updated_at <= ack_boundary ||
        final.position_updated_at <= ack_boundary) {
        return {false, "transition to VTOL verification failed: final post-ACK multicopter state is unavailable"};
    }
    if (!is_stable_vtol_sample(final, point)) {
        return {false, "transition to VTOL verification failed: post-transition state is not stable"};
    }
    return {true, "transition to VTOL verified: armed multicopter state is stable"};
}

} // namespace nomad::vehicle
