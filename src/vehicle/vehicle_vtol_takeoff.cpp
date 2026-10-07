// SPDX-License-Identifier: Apache-2.0
// QuadPlane takeoff admission and post-ACK climb verification.

#include "nomad/vehicle/vehicle.hpp"

#include "command_ids.hpp"

#include <chrono>
#include <cmath>
#include <optional>
#include <string>

namespace nomad::vehicle {
namespace {

// The pinned QuadPlane SITL profile settles at the requested target; keep a
// bounded 0.5 m margin for telemetry/control settling.
constexpr float kVtolTakeoffCompletionToleranceMeters = 0.5F;

} // namespace

CommandResult Vehicle::set_guided_mode_for_vtol_takeoff(std::uint64_t expected_session_id,
                                                        telemetry::AircraftClass aircraft_class) {
    const auto admission = require_operation(VehicleOperation::SetGuidedMode);
    if (!admission.success) {
        return admission;
    }
    const auto mode = telemetry::guided_mode_for(aircraft_class);
    if (!mode.has_value()) {
        return {false, "guided mode is unavailable for an unknown or unsupported aircraft"};
    }
    const auto result = send_command(
        make_command(kSetModeCommand, {1, static_cast<float>(*mode), 0, 0, 0, 0, 0}), "set guided mode",
        expected_session_id);
    if (!result.success) {
        return result;
    }
    return wait_for_mode(*mode, "set guided mode");
}

CommandResult Vehicle::arm_for_vtol_takeoff(std::uint64_t expected_session_id) {
    const auto admission = require_operation(VehicleOperation::Arm);
    if (!admission.success) {
        return admission;
    }
    const auto result = send_command(make_command(kArmDisarmCommand, {1, 0, 0, 0, 0, 0, 0}), "arm",
                                     expected_session_id);
    if (!result.success) {
        return result;
    }
    return wait_for_armed_state(true, "arm");
}

CommandResult Vehicle::prepare_vtol_takeoff(const telemetry::VehicleState &initial_state) {
    const auto guided_result =
        set_guided_mode_for_vtol_takeoff(initial_state.session_id, initial_state.identity.aircraft_class);
    if (!guided_result.success) {
        return guided_result;
    }
    const auto state_before_arm = connection_.get_state();
    if (state_before_arm.identity.aircraft_class != telemetry::AircraftClass::QuadPlane) {
        return {false, "vtol takeoff identity is no longer QuadPlane"};
    }
    if (const auto error = vtol_takeoff_identity_error(state_before_arm, initial_state); error.has_value()) {
        return {false, "vtol takeoff identity changed during preparation: " + *error};
    }
    if (state_before_arm.armed) {
        return {true, "vtol takeoff preparation verified"};
    }
    return arm_for_vtol_takeoff(initial_state.session_id);
}

CommandResult Vehicle::vtol_takeoff(float altitude_m) {
    if (!std::isfinite(altitude_m) || altitude_m <= 0.0F) {
        return {false, "vtol takeoff altitude must be finite and greater than zero"};
    }
    const auto admission = require_operation(VehicleOperation::VtolTakeoff);
    if (!admission.success) {
        return admission;
    }

    const auto initial_state = connection_.get_state();
    if (const auto error = vtol_takeoff_identity_error(initial_state, initial_state); error.has_value()) {
        return {false, "vtol takeoff rejected: " + *error};
    }
    if (!initial_state.position_valid) {
        return {false, "vtol takeoff requires a valid position"};
    }
    if (position_is_stale(initial_state)) {
        return {false, "vtol takeoff requires a fresh position"};
    }

    const auto preparation = prepare_vtol_takeoff(initial_state);
    if (!preparation.success) {
        return preparation;
    }

    const auto pre_takeoff_state = connection_.get_state();
    if (const auto error = vtol_takeoff_state_error(pre_takeoff_state, initial_state); error.has_value()) {
        return {false, *error};
    }
    const auto target_altitude_m = pre_takeoff_state.position.relative_altitude_m + altitude_m;
    if (!std::isfinite(target_altitude_m)) {
        return {false, "vtol takeoff target altitude is invalid"};
    }

    // ArduPlane's direct QuadPlane GUIDED path accepts NAV_TAKEOFF (22), not a
    // generic Copter interpretation. Its altitude parameter is a climb offset
    // from the final pre-command position, so completion uses the derived
    // absolute relative-altitude target below.
    const auto result = send_command(
        make_command(kQuadplaneGuidedTakeoffCommand, {0, 0, 0, 0, 0, 0, altitude_m}), "vtol takeoff",
        initial_state.session_id);
    if (!result.success) {
        return result;
    }
    const auto ack_boundary = std::chrono::steady_clock::now();
    return wait_for_vtol_takeoff(target_altitude_m, initial_state, ack_boundary);
}

std::optional<std::string> Vehicle::vtol_takeoff_identity_error(const telemetry::VehicleState &state,
                                                                const telemetry::VehicleState &expected_state) const {
    if (state.identity.aircraft_class != telemetry::AircraftClass::QuadPlane) {
        return "aircraft identity changed";
    }
    if (expected_state.session_id == 0 || state.session_id != expected_state.session_id) {
        return "vehicle session changed";
    }
    if (expected_state.system_id == 0 || state.system_id != expected_state.system_id) {
        return "vehicle system changed";
    }
    if (expected_state.component_id == 0 || state.component_id != expected_state.component_id) {
        return "vehicle component changed";
    }
    if (state.identity.autopilot_type != expected_state.identity.autopilot_type ||
        state.identity.vehicle_type != expected_state.identity.vehicle_type ||
        state.identity.aircraft_class != expected_state.identity.aircraft_class) {
        return "aircraft identity changed";
    }
    return {};
}

std::optional<std::string> Vehicle::vtol_takeoff_state_error(const telemetry::VehicleState &state,
                                                             const telemetry::VehicleState &expected_state) const {
    if (const auto error = vtol_takeoff_identity_error(state, expected_state); error.has_value()) {
        return error;
    }
    if (!state.connected || !state.heartbeat_fresh) {
        return "heartbeat is stale";
    }
    if (!state.position_valid) {
        return "position is invalid";
    }
    if (position_is_stale(state)) {
        return "position feed is stale";
    }
    if (!state.gps_valid || state.gps.fix_type < 3 || state.gps.satellites == 0) {
        return "a valid 3D GPS fix is required";
    }
    if (state.gps_updated_at == std::chrono::steady_clock::time_point{} ||
        std::chrono::steady_clock::now() - state.gps_updated_at > position_freshness_timeout_) {
        return "GPS feed is stale";
    }
    if (!state.armed) {
        return "vehicle disarmed";
    }
    if (!telemetry::is_guided_mode(telemetry::AircraftClass::QuadPlane, state.custom_mode)) {
        return "guided mode was lost";
    }
    if (!std::isfinite(state.position.relative_altitude_m)) {
        return "relative altitude is invalid";
    }
    return {};
}

CommandResult Vehicle::wait_for_vtol_takeoff(float target_altitude_m,
                                             const telemetry::VehicleState &expected_state,
                                             std::chrono::steady_clock::time_point ack_boundary) {
    const auto minimum_altitude_m = target_altitude_m - kVtolTakeoffCompletionToleranceMeters;
    const auto verdict = [this, minimum_altitude_m, &expected_state, ack_boundary](const telemetry::VehicleState &state)
        -> std::optional<CommandResult> {
        if (const auto error = vtol_takeoff_state_error(state, expected_state); error.has_value()) {
            return CommandResult{false, "vtol takeoff verification failed: " + *error};
        }
        if (state.position_updated_at <= ack_boundary) {
            return std::nullopt;
        }
        if (state.position.relative_altitude_m < minimum_altitude_m) {
            return std::nullopt;
        }
        return CommandResult{true, "vtol takeoff verified"};
    };
    return wait_for_state_until(vtol_takeoff_state_timeout_,
                                "vtol takeoff acknowledgement received but climb verification timed out", verdict);
}

} // namespace nomad::vehicle
