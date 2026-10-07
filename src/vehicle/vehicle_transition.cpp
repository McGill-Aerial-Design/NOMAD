// SPDX-License-Identifier: Apache-2.0
// QuadPlane VTOL-to-fixed-wing command admission and authoritative completion.

#include "nomad/vehicle/vehicle.hpp"

#include "command_ids.hpp"

#include <chrono>
#include <optional>
#include <string>

namespace nomad::vehicle {
namespace {

constexpr auto kVtolStateFreshnessTimeout = std::chrono::seconds(3);

} // namespace

CommandResult Vehicle::transition_to_fixed_wing() {
    const auto admission = require_operation(VehicleOperation::TransitionToFixedWing);
    if (!admission.success) {
        return admission;
    }

    const auto initial_state = connection_.get_state();
    if (const auto error = vtol_transition_state_error(initial_state, initial_state, true);
        error.has_value()) {
        return {false, "transition to fixed wing rejected: " + *error};
    }

    const auto result = send_command(
        make_command(kQuadplaneTransitionCommand,
                     {static_cast<float>(static_cast<std::uint8_t>(telemetry::VtolState::FixedWing)), 0, 0, 0, 0, 0,
                      0}),
        "transition to fixed wing", initial_state.session_id);
    if (!result.success) {
        return result;
    }
    // A fixed-wing observation received while waiting for the ACK is not post-ACK proof.
    const auto ack_boundary = std::chrono::steady_clock::now();
    return wait_for_fixed_wing_transition(initial_state, ack_boundary);
}

std::optional<std::string> Vehicle::vtol_transition_state_error(const telemetry::VehicleState &state,
                                                                const telemetry::VehicleState &expected_state,
                                                                bool require_precondition) const {
    if (state.identity.aircraft_class != telemetry::AircraftClass::QuadPlane) {
        return "aircraft identity changed";
    }
    if (state.identity.autopilot_type != expected_state.identity.autopilot_type ||
        state.identity.vehicle_type != expected_state.identity.vehicle_type ||
        state.identity.aircraft_class != expected_state.identity.aircraft_class) {
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
    if (!state.connected || !state.heartbeat_fresh) {
        return "heartbeat is stale";
    }
    if (!state.vtol_state_valid || state.vtol_state_updated_at == std::chrono::steady_clock::time_point{}) {
        return "VTOL state is unavailable";
    }
    if (std::chrono::steady_clock::now() - state.vtol_state_updated_at > kVtolStateFreshnessTimeout) {
        return "VTOL state feed is stale";
    }
    if (!state.armed) {
        return "vehicle disarmed";
    }
    if (!telemetry::is_auto_mode(telemetry::AircraftClass::QuadPlane, state.custom_mode)) {
        return "AUTO mode is required";
    }
    if (require_precondition && state.vtol_state != telemetry::VtolState::Multicopter) {
        return "vehicle is not in multicopter VTOL state";
    }
    return {};
}

CommandResult Vehicle::wait_for_fixed_wing_transition(
    const telemetry::VehicleState &expected_state, std::chrono::steady_clock::time_point ack_boundary) {
    const auto verdict = [this, expected_state, ack_boundary](const telemetry::VehicleState &state)
        -> std::optional<CommandResult> {
        if (const auto error = vtol_transition_state_error(state, expected_state, false); error.has_value()) {
            return CommandResult{false, "transition to fixed wing verification failed: " + *error};
        }
        if (state.vtol_state_updated_at <= ack_boundary) {
            return std::nullopt;
        }
        switch (state.vtol_state) {
        case telemetry::VtolState::FixedWing:
            return CommandResult{true, "transition to fixed wing verified"};
        case telemetry::VtolState::TransitionToFixedWing:
        case telemetry::VtolState::Multicopter:
            return std::nullopt;
        case telemetry::VtolState::TransitionToMulticopter:
            return CommandResult{false, "transition to fixed wing verification failed: vehicle transitioned back"};
        case telemetry::VtolState::Undefined:
            return CommandResult{false, "transition to fixed wing verification failed: VTOL state is undefined"};
        }
        return CommandResult{false, "transition to fixed wing verification failed: unknown VTOL state"};
    };
    return wait_for_state_until(
        transition_state_timeout_,
        "transition to fixed wing acknowledgement received but fixed-wing state verification timed out", verdict);
}

} // namespace nomad::vehicle
