// SPDX-License-Identifier: Apache-2.0
// Narrow, profile-gated QLAND admission and post-command touchdown verification.

#include "nomad/vehicle/vehicle.hpp"

#include "command_ids.hpp"
#include "geo.hpp"
#include "state_time.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>

namespace nomad::vehicle {
namespace {

using Clock = std::chrono::steady_clock;

constexpr auto kPollTimeout = std::chrono::milliseconds(50);
constexpr auto kAckTimeout = std::chrono::seconds(3);
constexpr auto kVersionTimeout = std::chrono::seconds(2);
constexpr auto kLandedStateFreshness = std::chrono::seconds(3);
constexpr double kLandingRegionRadiusMeters = 5.0;
constexpr double kLandingPositionSpreadMeters = 1.5;
constexpr float kMinimumEntryAltitudeMeters = 15.0F;
constexpr float kMaximumEntryAltitudeMeters = 25.0F;
constexpr float kMaximumEntryGroundspeedMps = 1.0F;
constexpr float kMaximumEntryClimbRateMps = 0.25F;
constexpr float kMaximumEntryAltitudeSpreadMeters = 1.0F;
constexpr std::size_t kReadySampleCount = 5;
constexpr float kRequiredDescentMeters = 5.0F;
constexpr float kMaximumFinalAltitudeMeters = 1.5F;
constexpr float kMinimumFinalAltitudeMeters = -1.0F;
constexpr float kMaximumFinalGroundspeedMps = 0.5F;
constexpr float kMaximumFinalClimbRateMps = 0.2F;

struct PinnedParameter {
    const char *name;
    float value;
};

constexpr std::array<PinnedParameter, 12> kPinnedLandingParameters{{
    {"Q_ENABLE", 2.0F},          {"Q_FRAME_CLASS", 7.0F},      {"Q_TILT_ENABLE", 1.0F},
    {"Q_TILT_MASK", 3.0F},       {"Q_TILT_TYPE", 0.0F},        {"Q_TILT_RATE_UP", 40.0F},
    {"Q_TILT_MAX", 45.0F},       {"Q_ASSIST_SPEED", 6.0F},     {"Q_OPTIONS", 0.0F},
    {"Q_LAND_FINAL_SPD", 0.5F},  {"Q_LAND_FINAL_ALT", 6.0F},   {"Q_LAND_ALTCHG", 0.2F},
}};

struct SampleWindow {
    std::size_t count{};
    Clock::time_point first{};
    Clock::time_point last{};
    Clock::time_point last_position{};
    Clock::time_point last_velocity{};
    double anchor_latitude{};
    double anchor_longitude{};
    float minimum_altitude{};
    float maximum_altitude{};
};

double distance_to_landing_point(const telemetry::VehicleState &state, const LandingPoint &point) {
    return detail::distance_m(state.position.latitude_deg, state.position.longitude_deg, point.latitude_deg,
                              point.longitude_deg);
}

std::optional<std::string> validate_landing_identity(const telemetry::VehicleState &state,
                                                     std::uint64_t expected_session,
                                                     std::uint8_t expected_system,
                                                     std::uint8_t expected_component) {
    if (state.identity.aircraft_class != telemetry::AircraftClass::QuadPlane ||
        state.identity.autopilot_type != telemetry::kArduPilotAutopilot ||
        state.identity.vehicle_type != telemetry::kFixedWing) {
        return "pinned ArduPilot QuadPlane identity changed";
    }
    if (expected_session == 0 || state.session_id != expected_session) {
        return "vehicle session changed";
    }
    if (expected_system == 0 || state.system_id != expected_system || expected_component != 1 ||
        state.component_id != expected_component) {
        return "vehicle system or component identity changed";
    }
    if (!state.connected || !state.heartbeat_fresh) {
        return "heartbeat or link was lost";
    }
    return {};
}

std::optional<std::string> validate_landing_telemetry(const telemetry::VehicleState &state,
                                                      std::uint64_t expected_session,
                                                      std::uint8_t expected_system,
                                                      std::uint8_t expected_component,
                                                      std::chrono::milliseconds freshness) {
    if (const auto error = validate_landing_identity(state, expected_session, expected_system, expected_component);
        error.has_value()) {
        return error;
    }
    const auto now = Clock::now();
    if (!state.position_valid || !detail::timestamp_is_fresh(state.position_updated_at, freshness, now) ||
        !std::isfinite(state.position.latitude_deg) || state.position.latitude_deg < -90.0 ||
        state.position.latitude_deg > 90.0 || !std::isfinite(state.position.longitude_deg) ||
        state.position.longitude_deg < -180.0 || state.position.longitude_deg > 180.0 ||
        !std::isfinite(state.position.relative_altitude_m)) {
        return "position is invalid or stale";
    }
    if (!detail::timestamp_is_fresh(state.velocity_updated_at, freshness, now) ||
        !std::isfinite(state.velocity.groundspeed_mps) ||
        !std::isfinite(state.velocity.climb_rate_mps)) {
        return "velocity is invalid or stale";
    }
    if (!state.gps_valid || state.gps.fix_type < 3 || state.gps.satellites == 0 ||
        !detail::timestamp_is_fresh(state.gps_updated_at, freshness, now)) {
        return "a fresh 3D GPS fix is required";
    }
    if (!state.vtol_state_valid || state.vtol_state != telemetry::VtolState::Multicopter ||
        !detail::timestamp_is_fresh(state.vtol_state_updated_at, kLandedStateFreshness, now)) {
        return "authoritative multicopter VTOL state is unavailable, stale, or changed";
    }
    if (!state.landed_state_valid ||
        !detail::timestamp_is_fresh(state.landed_state_updated_at, kLandedStateFreshness, now)) {
        return "landed-state telemetry is unavailable or stale";
    }
    return {};
}

std::optional<std::string> validate_landing_entry(const telemetry::VehicleState &state, const LandingPoint &point,
                                                  std::uint64_t session_id, std::uint8_t system_id,
                                                  std::uint8_t component_id,
                                                  std::chrono::milliseconds freshness) {
    if (const auto error = validate_landing_telemetry(state, session_id, system_id, component_id, freshness);
        error.has_value()) {
        return error;
    }
    if (!state.armed || state.custom_mode != 10 || state.vtol_state != telemetry::VtolState::Multicopter ||
        state.landed_state != telemetry::LandedState::InAir) {
        return "armed AUTO, multicopter state, and fresh IN_AIR telemetry are required";
    }
    if (state.position.relative_altitude_m < kMinimumEntryAltitudeMeters ||
        state.position.relative_altitude_m > kMaximumEntryAltitudeMeters) {
        return "landing entry altitude must be between 15 and 25 m above home";
    }
    if (state.velocity.groundspeed_mps > kMaximumEntryGroundspeedMps ||
        std::abs(state.velocity.climb_rate_mps) > kMaximumEntryClimbRateMps) {
        return "landing entry speed or vertical motion exceeds the reviewed SITL bound";
    }
    if (distance_to_landing_point(state, point) > kLandingRegionRadiusMeters) {
        return "landing point is more than 5 m from the aircraft";
    }
    return {};
}

std::optional<std::string> validate_landing_point(const LandingPoint &point,
                                                  const safety::GlobalFencePolicy &fence_policy) {
    if (!std::isfinite(point.latitude_deg) || point.latitude_deg < -90.0 || point.latitude_deg > 90.0 ||
        !std::isfinite(point.longitude_deg) || point.longitude_deg < -180.0 || point.longitude_deg > 180.0 ||
        (point.latitude_deg == 0.0 && point.longitude_deg == 0.0)) {
        return "landing-point coordinates are invalid";
    }
    const auto fence = safety::evaluate_global_position(fence_policy, {point.latitude_deg, point.longitude_deg});
    return fence.allowed ? std::nullopt : std::optional<std::string>{fence.message};
}

void reset_window(SampleWindow &window, const telemetry::VehicleState &state) {
    const auto sample_time = (std::max)(state.position_updated_at, state.velocity_updated_at);
    window = {1, sample_time, sample_time, state.position_updated_at, state.velocity_updated_at,
              state.position.latitude_deg, state.position.longitude_deg, state.position.relative_altitude_m,
              state.position.relative_altitude_m};
}

bool add_stable_sample(SampleWindow &window, const telemetry::VehicleState &state, const LandingPoint &point) {
    if (window.count > 0 && (state.position_updated_at <= window.last_position ||
                             state.velocity_updated_at <= window.last_velocity)) {
        return false;
    }
    if (distance_to_landing_point(state, point) > kLandingRegionRadiusMeters ||
        state.velocity.groundspeed_mps > kMaximumEntryGroundspeedMps ||
        std::abs(state.velocity.climb_rate_mps) > kMaximumEntryClimbRateMps) {
        window = {};
        return false;
    }
    if (window.count == 0) {
        reset_window(window, state);
        return true;
    }
    const auto spread = detail::distance_m(window.anchor_latitude, window.anchor_longitude,
                                           state.position.latitude_deg, state.position.longitude_deg);
    const auto minimum_altitude = std::min(window.minimum_altitude, state.position.relative_altitude_m);
    const auto maximum_altitude = std::max(window.maximum_altitude, state.position.relative_altitude_m);
    if (spread > kLandingPositionSpreadMeters ||
        maximum_altitude - minimum_altitude > kMaximumEntryAltitudeSpreadMeters) {
        reset_window(window, state);
        return true;
    }
    ++window.count;
    window.last = (std::max)(state.position_updated_at, state.velocity_updated_at);
    window.last_position = state.position_updated_at;
    window.last_velocity = state.velocity_updated_at;
    window.minimum_altitude = minimum_altitude;
    window.maximum_altitude = maximum_altitude;
    return true;
}

bool is_ready(const SampleWindow &window, std::chrono::milliseconds dwell) {
    return window.count >= kReadySampleCount && window.last - window.first >= dwell;
}

bool is_final_sample(const telemetry::VehicleState &state, const LandingPoint &point, float entry_altitude) {
    return state.landed_state == telemetry::LandedState::OnGround && !state.armed &&
           state.position.relative_altitude_m >= kMinimumFinalAltitudeMeters &&
           state.position.relative_altitude_m <= kMaximumFinalAltitudeMeters &&
           state.velocity.groundspeed_mps <= kMaximumFinalGroundspeedMps &&
           std::abs(state.velocity.climb_rate_mps) <= kMaximumFinalClimbRateMps &&
           distance_to_landing_point(state, point) <= kLandingRegionRadiusMeters &&
           entry_altitude - state.position.relative_altitude_m >= kRequiredDescentMeters;
}

struct LandingProgress {
    bool qland_seen{false};
    bool descent_seen{false};
    bool on_ground_seen{false};
    bool complete{false};
    SampleWindow final_window;
};

bool add_final_landing_sample(SampleWindow &window, const telemetry::VehicleState &state,
                              const LandingPoint &point, float entry_altitude,
                              std::chrono::milliseconds dwell) {
    if (!is_final_sample(state, point, entry_altitude)) {
        window = {};
        return false;
    }
    if (window.count == 0) {
        reset_window(window, state);
    } else if (state.position_updated_at > window.last_position && state.velocity_updated_at > window.last_velocity) {
        const auto spread = detail::distance_m(window.anchor_latitude, window.anchor_longitude,
                                               state.position.latitude_deg, state.position.longitude_deg);
        const auto minimum_altitude = std::min(window.minimum_altitude, state.position.relative_altitude_m);
        const auto maximum_altitude = std::max(window.maximum_altitude, state.position.relative_altitude_m);
        if (spread > kLandingPositionSpreadMeters || maximum_altitude - minimum_altitude > 0.5F) {
            reset_window(window, state);
        } else {
            ++window.count;
            window.last = (std::max)(state.position_updated_at, state.velocity_updated_at);
            window.last_position = state.position_updated_at;
            window.last_velocity = state.velocity_updated_at;
            window.minimum_altitude = minimum_altitude;
            window.maximum_altitude = maximum_altitude;
        }
    }
    return window.count >= kReadySampleCount && window.last - window.first >= dwell;
}

std::optional<std::string> observe_landing_progress(LandingProgress &progress,
                                                   const telemetry::VehicleState &state,
                                                   const LandingPoint &point, float entry_altitude,
                                                   Clock::time_point ack_boundary,
                                                   std::chrono::milliseconds dwell) {
    if (state.custom_mode == kQuadplaneQlandMode) {
        progress.qland_seen = true;
    } else if (progress.qland_seen || state.custom_mode != 10) {
        return "vehicle left the reviewed QLAND/AUTO mode path";
    }
    if (state.landed_state_updated_at > ack_boundary && state.landed_state == telemetry::LandedState::OnGround) {
        progress.on_ground_seen = true;
    }
    const bool new_post_ack_sample = state.position_updated_at > ack_boundary &&
                                     state.velocity_updated_at > ack_boundary &&
                                     state.vtol_state_updated_at > ack_boundary &&
                                     state.landed_state_updated_at > ack_boundary;
    if (!new_post_ack_sample) {
        return {};
    }
    if (state.landed_state == telemetry::LandedState::OnGround && !progress.descent_seen) {
        return "ON_GROUND occurred before NOMAD observed post-command descent";
    }
    if (!state.armed && state.landed_state != telemetry::LandedState::OnGround) {
        return "vehicle disarmed before landed-state confirmation";
    }
    if (progress.qland_seen && entry_altitude - state.position.relative_altitude_m >= kRequiredDescentMeters) {
        progress.descent_seen = true;
    }
    progress.complete = progress.descent_seen && state.landed_state == telemetry::LandedState::OnGround &&
                        add_final_landing_sample(progress.final_window, state, point, entry_altitude, dwell);
    if (!progress.descent_seen || state.landed_state != telemetry::LandedState::OnGround) {
        progress.final_window = {};
    }
    return {};
}

std::string landing_timeout_reason(const LandingProgress &progress) {
    if (!progress.qland_seen) {
        return "QuadPlane did not enter QLAND after acknowledgement";
    }
    if (!progress.descent_seen) {
        return "QuadPlane QLAND acknowledgement was not followed by verified descent";
    }
    if (!progress.on_ground_seen) {
        return "QuadPlane landing deadline expired without post-command ON_GROUND telemetry";
    }
    return "QuadPlane touchdown did not meet the stable final-state envelope before timeout";
}

} // namespace

CommandResult Vehicle::quadplane_vtol_land(const LandingPoint &point) {
    const auto capability = require_operation(VehicleOperation::QuadplaneVtolLand);
    if (!capability.success) {
        return capability;
    }
    if (const auto error = validate_landing_point(point, fence_policy_); error.has_value()) {
        return {false, "QuadPlane VTOL landing rejected: " + *error};
    }
    if (quadplane_landing_timeout_ <= std::chrono::milliseconds::zero() ||
        transition_ready_dwell_ <= std::chrono::milliseconds::zero()) {
        return {false, "QuadPlane VTOL landing rejected: landing deadline and readiness dwell must be positive"};
    }

    const auto deadline = Clock::now() + quadplane_landing_timeout_;
    std::uint64_t session_id{};
    std::uint8_t system_id{};
    std::uint8_t component_id{};
    float entry_altitude_m{};
    const auto readiness = verify_quadplane_landing_admission(point, deadline, session_id, system_id, component_id,
                                                              entry_altitude_m);
    if (!readiness.success) {
        return readiness;
    }
    const auto command_result = send_quadplane_qland(session_id, deadline);
    if (!command_result.success) {
        return command_result;
    }
    const auto ack_boundary = Clock::now();
    return verify_quadplane_touchdown(point, session_id, system_id, component_id, ack_boundary, deadline,
                                      entry_altitude_m);
}

CommandResult Vehicle::verify_quadplane_landing_admission(const LandingPoint &point, Clock::time_point deadline,
                                                         std::uint64_t &session_id, std::uint8_t &system_id,
                                                         std::uint8_t &component_id, float &entry_altitude_m) {
    const auto initial = connection_.get_state();
    session_id = initial.session_id;
    system_id = initial.system_id;
    component_id = initial.component_id;
    if (const auto error = validate_landing_entry(initial, point, session_id, system_id, component_id,
                                                  position_freshness_timeout_);
        error.has_value()) {
        return {false, "QuadPlane VTOL landing rejected: " + *error};
    }
    const auto profile = verify_quadplane_profile(point, session_id, system_id, component_id, deadline);
    if (!profile.success) {
        return profile;
    }
    const auto ready = wait_for_quadplane_landing_ready(point, session_id, system_id, component_id, deadline);
    if (!ready.success) {
        return ready;
    }
    const auto ready_state = connection_.get_state();
    if (const auto error = validate_landing_entry(ready_state, point, session_id, system_id, component_id,
                                                  position_freshness_timeout_);
        error.has_value()) {
        return {false, "QuadPlane VTOL landing readiness changed before command transmission: " + *error};
    }
    entry_altitude_m = ready_state.position.relative_altitude_m;
    return {true, "QuadPlane VTOL landing admission and readiness verified"};
}

CommandResult Vehicle::send_quadplane_qland(std::uint64_t expected_session_id, Clock::time_point deadline) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
    if (remaining <= std::chrono::milliseconds::zero()) {
        return {false, "QuadPlane VTOL landing timed out before command transmission"};
    }
    const auto command = make_command(kSetModeCommand, {1.0F, static_cast<float>(kQuadplaneQlandMode), 0, 0, 0, 0, 0});
    const auto ack_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(kAckTimeout);
    const auto ack = connection_.send_command(command, expected_session_id, std::min(remaining, ack_timeout));
    if (!ack.has_value()) {
        return {false, "QuadPlane VTOL landing timed out waiting for QLAND mode acknowledgement"};
    }
    if (ack->command != command.id || ack->result != kAcceptedResult) {
        return {false, "QuadPlane VTOL landing QLAND request was rejected or mismatched"};
    }
    return {true, "QLAND request acknowledged"};
}

CommandResult Vehicle::verify_quadplane_profile(const LandingPoint &point, std::uint64_t session_id,
                                                std::uint8_t system_id, std::uint8_t component_id,
                                                Clock::time_point deadline) {
    const auto version_remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
    if (version_remaining <= std::chrono::milliseconds::zero()) {
        return {false, "QuadPlane VTOL landing timed out during firmware verification"};
    }
    const auto version_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(kVersionTimeout);
    const auto version = connection_.read_autopilot_version(std::min(version_remaining, version_timeout));
    if (!version || version->major != 4 || version->minor != 7 || version->patch != 1 ||
        version->git_hash.rfind("dbe79216", 0) != 0) {
        return {false, "QuadPlane VTOL landing requires pinned ArduPlane 4.7.1 at dbe792162d06"};
    }

    for (const auto &parameter : kPinnedLandingParameters) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
        if (remaining <= std::chrono::milliseconds::zero()) {
            return {false, "QuadPlane VTOL landing timed out during profile readback"};
        }
        const auto readback_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::seconds(2));
        const auto value = connection_.read_param(parameter.name, std::min(remaining, readback_timeout));
        if (!value || !std::isfinite(*value) || std::abs(*value - parameter.value) > 0.001F) {
            return {false, "QuadPlane VTOL landing requires profile parameter " + std::string(parameter.name) +
                               "=" + std::to_string(parameter.value)};
        }
    }
    const auto state = connection_.get_state();
    if (const auto error = validate_landing_entry(state, point, session_id, system_id, component_id,
                                                  position_freshness_timeout_);
        error.has_value()) {
        return {false, "QuadPlane VTOL landing admission changed during profile readback: " + *error};
    }
    return {true, "pinned QuadPlane landing profile verified"};
}

CommandResult Vehicle::wait_for_quadplane_landing_ready(const LandingPoint &point, std::uint64_t session_id,
                                                        std::uint8_t system_id, std::uint8_t component_id,
                                                        Clock::time_point deadline) {
    SampleWindow window;
    while (Clock::now() < deadline) {
        const auto update = connection_.wait_for_state(kPollTimeout);
        const auto state = update.value_or(connection_.get_state());
        if (const auto error = validate_landing_telemetry(state, session_id, system_id, component_id,
                                                           position_freshness_timeout_);
            error.has_value()) {
            return {false, "QuadPlane VTOL landing readiness failed: " + *error};
        }
        if (!state.armed || state.custom_mode != 10 || state.vtol_state != telemetry::VtolState::Multicopter ||
            state.landed_state != telemetry::LandedState::InAir ||
            state.position.relative_altitude_m < kMinimumEntryAltitudeMeters ||
            state.position.relative_altitude_m > kMaximumEntryAltitudeMeters) {
            return {false, "QuadPlane VTOL landing readiness changed during stabilization"};
        }
        add_stable_sample(window, state, point);
        if (is_ready(window, transition_ready_dwell_)) {
            return {true, "landing-ready region stabilized"};
        }
    }
    return {false, "QuadPlane VTOL landing timed out before a stable readiness dwell"};
}

CommandResult Vehicle::verify_quadplane_touchdown(const LandingPoint &point, std::uint64_t session_id,
                                                  std::uint8_t system_id, std::uint8_t component_id,
                                                  Clock::time_point ack_boundary, Clock::time_point deadline,
                                                  float entry_altitude_m) {
    LandingProgress progress;
    while (Clock::now() < deadline) {
        const auto update = connection_.wait_for_state(kPollTimeout);
        const auto state = update.value_or(connection_.get_state());
        if (const auto error = validate_landing_telemetry(state, session_id, system_id, component_id,
                                                           position_freshness_timeout_);
            error.has_value()) {
            return {false, "QuadPlane VTOL landing failed after acknowledgement: " + *error};
        }
        if (const auto error = observe_landing_progress(progress, state, point, entry_altitude_m,
                                                       ack_boundary, transition_ready_dwell_);
            error.has_value()) {
            return {false, "QuadPlane VTOL landing failed after acknowledgement: " + *error};
        }
        if (progress.complete) {
            return {true, "QuadPlane landing verified by post-command descent, ON_GROUND, disarm, and stable state"};
        }
    }
    return {false, landing_timeout_reason(progress)};
}

} // namespace nomad::vehicle
