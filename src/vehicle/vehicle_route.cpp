// SPDX-License-Identifier: Apache-2.0
// QuadPlane fixed-wing route admission, waypoint commands and state proof.

#include "nomad/vehicle/vehicle.hpp"

#include "command_ids.hpp"
#include "geo.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>

namespace nomad::vehicle {
namespace {

using Clock = std::chrono::steady_clock;

constexpr std::size_t kFixedWingRouteWaypointCount = 2;
constexpr double kRouteArrivalRadiusMeters = 45.0;
constexpr double kMinimumRoutePointSpacingMeters = 100.0;
constexpr float kRouteLoiterRadiusMeters = 30.0F;
constexpr float kRouteAltitudeMinimumMeters = 2.0F;
constexpr float kRouteAltitudeMaximumMeters = 100.0F;
constexpr float kRouteAltitudeToleranceMeters = 5.0F;
constexpr auto kRouteVtolFreshnessTimeout = std::chrono::seconds(3);
constexpr auto kRouteCommandTimeout = std::chrono::seconds(3);
constexpr auto kRouteStatePollTimeout = std::chrono::milliseconds(50);

std::optional<std::string> validate_route(const std::vector<RouteWaypoint> &route,
                                          const safety::GlobalFencePolicy &fence_policy) {
    if (route.size() != kFixedWingRouteWaypointCount) {
        return "exactly two waypoints are required";
    }
    for (const auto &waypoint : route) {
        if (!std::isfinite(waypoint.latitude_deg) || waypoint.latitude_deg < -90.0 ||
            waypoint.latitude_deg > 90.0 || !std::isfinite(waypoint.longitude_deg) ||
            waypoint.longitude_deg < -180.0 || waypoint.longitude_deg > 180.0) {
            return "waypoint coordinates are invalid";
        }
        if (!std::isfinite(waypoint.relative_altitude_m) ||
            waypoint.relative_altitude_m < kRouteAltitudeMinimumMeters ||
            waypoint.relative_altitude_m > kRouteAltitudeMaximumMeters) {
            return "waypoint relative altitude must be between 2 and 100 m";
        }
        const auto fence = safety::evaluate_global_position(
            fence_policy, {waypoint.latitude_deg, waypoint.longitude_deg});
        if (!fence.allowed) {
            return fence.message;
        }
    }
    const double spacing = detail::distance_m(route[0].latitude_deg, route[0].longitude_deg,
                                               route[1].latitude_deg, route[1].longitude_deg);
    if (spacing < kMinimumRoutePointSpacingMeters) {
        return "waypoints must be at least 100 m apart";
    }
    return {};
}

bool gps_is_fresh(const telemetry::VehicleState &state, std::chrono::milliseconds timeout) {
    return state.gps_valid && state.gps.fix_type >= 3 && state.gps.satellites > 0 &&
           state.gps_updated_at != Clock::time_point{} && Clock::now() - state.gps_updated_at <= timeout;
}

} // namespace

CommandResult Vehicle::fixed_wing_route(const std::vector<RouteWaypoint> &route) {
    const auto admission = require_operation(VehicleOperation::FixedWingRoute);
    if (!admission.success) {
        return admission;
    }
    if (const auto error = validate_route(route, fence_policy_); error.has_value()) {
        return {false, "fixed-wing route rejected: " + *error};
    }
    if (fixed_wing_route_timeout_ <= std::chrono::milliseconds::zero()) {
        return {false, "fixed-wing route rejected: route timeout must be positive"};
    }

    const auto initial_state = connection_.get_state();
    if (initial_state.session_id == 0) {
        return {false, "fixed-wing route rejected: vehicle session is unavailable"};
    }
    if (const auto error = fixed_wing_route_state_error(initial_state, initial_state.session_id, true);
        error.has_value()) {
        return {false, "fixed-wing route rejected: " + *error};
    }

    const auto deadline = Clock::now() + fixed_wing_route_timeout_;
    const auto guided_result = set_guided_mode();
    if (!guided_result.success) {
        return {false, "fixed-wing route setup failed: " + guided_result.message};
    }
    const auto guided_state = connection_.get_state();
    if (const auto error = fixed_wing_route_state_error(guided_state, initial_state.session_id, false);
        error.has_value()) {
        return {false, "fixed-wing route rejected after GUIDED setup: " + *error};
    }
    return execute_fixed_wing_route(route, initial_state.session_id, deadline);
}

CommandResult Vehicle::execute_fixed_wing_route(const std::vector<RouteWaypoint> &route,
                                                std::uint64_t expected_session_id, Clock::time_point deadline) {
    for (std::size_t index = 0; index < route.size(); ++index) {
        const auto state = connection_.get_state();
        if (const auto error = fixed_wing_route_state_error(state, expected_session_id, false); error.has_value()) {
            return {false, "fixed-wing route rejected before waypoint " + std::to_string(index + 1) + ": " + *error};
        }
        const auto &waypoint = route[index];
        const double initial_distance = detail::distance_m(state.position.latitude_deg, state.position.longitude_deg,
                                                           waypoint.latitude_deg, waypoint.longitude_deg);
        if (initial_distance <= kRouteArrivalRadiusMeters + 10.0) {
            return {false, "fixed-wing route rejected: waypoint is too close to prove new navigation"};
        }
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
        if (remaining <= std::chrono::milliseconds::zero()) {
            return {false, "fixed-wing route timed out before waypoint " + std::to_string(index + 1)};
        }
        const auto acknowledgement = connection_.send_fixed_wing_waypoint(
            {waypoint.latitude_deg, waypoint.longitude_deg, waypoint.relative_altitude_m, kRouteLoiterRadiusMeters},
            expected_session_id,
            std::min(remaining, std::chrono::duration_cast<std::chrono::milliseconds>(kRouteCommandTimeout)));
        if (!acknowledgement.has_value()) {
            return {false, "fixed-wing waypoint " + std::to_string(index + 1) + " received no command ACK"};
        }
        if (acknowledgement->command != kDoRepositionCommand || acknowledgement->result != kAcceptedResult) {
            return {false, "fixed-wing waypoint " + std::to_string(index + 1) + " was rejected by ArduPilot"};
        }
        const auto acknowledgement_received_at = Clock::now();
        const auto acknowledgement_state = connection_.get_state();
        if (const auto error = fixed_wing_route_state_error(acknowledgement_state, expected_session_id, false);
            error.has_value()) {
            return {false, "fixed-wing route verification failed after waypoint " + std::to_string(index + 1) +
                               " ACK: " + *error};
        }
        const double acknowledgement_distance = detail::distance_m(acknowledgement_state.position.latitude_deg,
                                                                   acknowledgement_state.position.longitude_deg,
                                                                   waypoint.latitude_deg, waypoint.longitude_deg);
        const auto acknowledgement_boundary =
            std::max(acknowledgement_received_at, acknowledgement_state.position_updated_at);
        const auto reached = wait_for_fixed_wing_waypoint(waypoint, expected_session_id, acknowledgement_boundary,
                                                          acknowledgement_distance, deadline);
        if (!reached.success) {
            return reached;
        }
    }
    return {true, "fixed-wing route verified: waypoints=2"};
}

std::optional<std::string> Vehicle::fixed_wing_route_state_error(const telemetry::VehicleState &state,
                                                                 std::uint64_t expected_session_id,
                                                                 bool require_auto_mode) const {
    if (state.identity.aircraft_class != telemetry::AircraftClass::QuadPlane ||
        state.identity.autopilot_type != telemetry::kArduPilotAutopilot ||
        state.identity.vehicle_type != telemetry::kFixedWing) {
        return "aircraft identity is no longer QuadPlane";
    }
    if (state.session_id == 0 || state.session_id != expected_session_id) {
        return "vehicle session changed";
    }
    if (!state.connected || !state.heartbeat_fresh || state.system_id == 0 || state.component_id == 0) {
        return "heartbeat is stale";
    }
    if (!state.position_valid) {
        return "position is invalid";
    }
    if (position_is_stale(state)) {
        return "position feed is stale";
    }
    if (!std::isfinite(state.position.latitude_deg) || !std::isfinite(state.position.longitude_deg) ||
        !std::isfinite(state.position.relative_altitude_m)) {
        return "position is invalid";
    }
    if (!gps_is_fresh(state, position_freshness_timeout_)) {
        return "a fresh 3D GPS fix is required";
    }
    if (!state.armed) {
        return "vehicle is disarmed";
    }
    if (!state.vtol_state_valid || state.vtol_state_updated_at == Clock::time_point{}) {
        return "VTOL state is unavailable";
    }
    if (Clock::now() - state.vtol_state_updated_at > kRouteVtolFreshnessTimeout) {
        return "VTOL state feed is stale";
    }
    if (state.vtol_state != telemetry::VtolState::FixedWing) {
        return "authoritative VTOL state is not fixed wing";
    }
    if (require_auto_mode && !telemetry::is_auto_mode(telemetry::AircraftClass::QuadPlane, state.custom_mode)) {
        return "AUTO mode is required after the fixed-wing transition";
    }
    if (!require_auto_mode && !telemetry::is_guided_mode(telemetry::AircraftClass::QuadPlane, state.custom_mode)) {
        return "GUIDED mode was lost";
    }
    return {};
}

CommandResult Vehicle::wait_for_fixed_wing_waypoint(const RouteWaypoint &waypoint,
                                                    std::uint64_t expected_session_id,
                                                    Clock::time_point acknowledgement_boundary,
                                                    double acknowledgement_distance_m,
                                                    Clock::time_point deadline) {
    while (Clock::now() < deadline) {
        const auto state = connection_.wait_for_state(kRouteStatePollTimeout);
        const auto sample = state.value_or(connection_.get_state());
        if (const auto error = fixed_wing_route_state_error(sample, expected_session_id, false); error.has_value()) {
            return {false, "fixed-wing route verification failed: " + *error};
        }
        if (sample.position_updated_at <= acknowledgement_boundary) {
            continue;
        }
        const double remaining_distance = detail::distance_m(sample.position.latitude_deg,
                                                             sample.position.longitude_deg, waypoint.latitude_deg,
                                                             waypoint.longitude_deg);
        const double altitude_error = std::abs(sample.position.relative_altitude_m - waypoint.relative_altitude_m);
        if (remaining_distance <= kRouteArrivalRadiusMeters &&
            acknowledgement_distance_m - remaining_distance >= 10.0 &&
            altitude_error <= kRouteAltitudeToleranceMeters) {
            return {true, "fixed-wing waypoint position verified"};
        }
    }
    return {false, "fixed-wing route ACK received but waypoint position verification timed out"};
}

} // namespace nomad::vehicle
