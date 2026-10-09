// SPDX-License-Identifier: Apache-2.0
// Bounded Copter LAND engagement; touchdown remains an independent observation.
#include "nomad/vehicle/vehicle.hpp"

#include "command_ids.hpp"
#include "state_time.hpp"

#include <algorithm>
#include <chrono>
#include <optional>
#include <thread>

namespace nomad::vehicle {
namespace {

using Clock = std::chrono::steady_clock;
constexpr auto kLandEngagementTimeout = std::chrono::seconds(3);
constexpr auto kLandPollInterval = std::chrono::milliseconds(20);

CommandResult land_result(CommandOutcome outcome, const char *message, bool acknowledged = false) {
    return {outcome == CommandOutcome::Success, message, acknowledged, outcome};
}

bool land_identity_matches(const telemetry::VehicleState &state, const telemetry::VehicleState &expected) {
    return expected.session_id != 0 && expected.system_id != 0 && expected.component_id != 0 &&
           telemetry::identify_vehicle(expected.identity.autopilot_type, expected.identity.vehicle_type)
               .aircraft_class == telemetry::AircraftClass::Copter &&
           state.session_id == expected.session_id && state.system_id == expected.system_id &&
           state.component_id == expected.component_id &&
           state.identity.aircraft_class == telemetry::AircraftClass::Copter &&
           state.identity.aircraft_class == expected.identity.aircraft_class &&
           state.identity.autopilot_type == expected.identity.autopilot_type &&
           state.identity.vehicle_type == expected.identity.vehicle_type;
}

bool land_heartbeat_is_fresh(const telemetry::VehicleState &state) {
    return state.connected && state.heartbeat_fresh &&
           detail::timestamp_is_fresh(state.heartbeat_updated_at, kLandEngagementTimeout, Clock::now());
}

CommandResult validate_land_acknowledgement(const std::optional<mavlink::CommandAck> &ack) {
    if (!ack) {
        return land_result(CommandOutcome::Unknown, "LAND acknowledgement unavailable; aircraft outcome unknown");
    }
    if (ack->status == mavlink::CommandAck::Status::AdmissionCancelled) {
        return land_result(CommandOutcome::Interrupted, "LAND admission interrupted; aircraft outcome unknown");
    }
    if (ack->command != kLandCommand) {
        return land_result(CommandOutcome::Unknown, "LAND acknowledgement did not match; aircraft outcome unknown");
    }
    if (ack->result != kAcceptedResult) {
        return land_result(CommandOutcome::Failed, "LAND rejected by ArduPilot", true);
    }
    return {true, "LAND acknowledged", true};
}

} // namespace

CommandResult Vehicle::engage_copter_land(const std::function<bool()> &still_authorized) {
    const auto deadline = Clock::now() + kLandEngagementTimeout;
    const auto initial_state = connection_.get_state();
    if (!connection_.is_connected() || !land_identity_matches(initial_state, initial_state) ||
        !land_heartbeat_is_fresh(initial_state)) {
        return land_result(CommandOutcome::Rejected, "LAND requires a fresh identified Copter session");
    }
    if (still_authorized && !still_authorized()) {
        return land_result(CommandOutcome::Rejected, "LAND authority unavailable before send");
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
    if (remaining <= std::chrono::milliseconds::zero()) {
        return land_result(CommandOutcome::Rejected, "LAND engagement budget expired before send");
    }
    auto command = make_command(kLandCommand);
    command.state_admission = [initial_state, deadline](const telemetry::VehicleState &state) {
        return Clock::now() < deadline && land_identity_matches(state, initial_state) && land_heartbeat_is_fresh(state);
    };
    const auto ack = connection_.send_command(command, initial_state.session_id, remaining);
    const auto ack_boundary = Clock::now();
    const auto result = validate_land_acknowledgement(ack);
    if (result.outcome == CommandOutcome::Interrupted && connection_.is_connected() &&
        land_identity_matches(connection_.get_state(), initial_state) &&
        (!still_authorized || still_authorized())) {
        return land_result(CommandOutcome::Unknown, "LAND admission ended; aircraft outcome unknown");
    }
    if (!result.success) {
        return result;
    }
    return wait_for_copter_land(initial_state, ack_boundary, deadline, still_authorized);
}

CommandResult Vehicle::wait_for_copter_land(const telemetry::VehicleState &initial_state,
                                           Clock::time_point ack_boundary, Clock::time_point deadline,
                                           const std::function<bool()> &still_authorized) {
    while (Clock::now() < deadline) {
        if (still_authorized && !still_authorized()) {
            return land_result(CommandOutcome::Interrupted,
                "LAND authority interrupted; aircraft outcome unknown", true);
        }
        const auto state = connection_.get_state();
        const auto observed_at = Clock::now();
        if (observed_at >= deadline) {
            break;
        }
        if (!land_identity_matches(state, initial_state) || !connection_.is_connected() || !state.connected) {
            return land_result(CommandOutcome::Interrupted,
                "LAND session or identity changed; aircraft outcome unknown", true);
        }
        if (land_heartbeat_is_fresh(state) && state.heartbeat_updated_at > ack_boundary &&
            state.heartbeat_updated_at <= deadline &&
            telemetry::is_landing_mode(telemetry::AircraftClass::Copter, state.custom_mode)) {
            return land_result(CommandOutcome::Success, "LAND mode observed; touchdown not verified", true);
        }
        std::this_thread::sleep_until((std::min)(deadline, Clock::now() + kLandPollInterval));
    }
    return land_result(CommandOutcome::Unknown,
        "LAND acknowledged but engagement not observed within three seconds; aircraft outcome unknown", true);
}

} // namespace nomad::vehicle
