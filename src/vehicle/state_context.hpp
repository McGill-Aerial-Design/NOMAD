// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/telemetry/state.hpp"
#include "state_time.hpp"

namespace nomad::vehicle::detail {

inline bool same_vehicle_session(const telemetry::VehicleState &state, const telemetry::VehicleState &expected) {
    return state.session_id == expected.session_id && state.system_id == expected.system_id &&
           state.component_id == expected.component_id &&
           state.identity.aircraft_class == expected.identity.aircraft_class &&
           state.identity.autopilot_type == expected.identity.autopilot_type &&
           state.identity.vehicle_type == expected.identity.vehicle_type;
}

inline bool identified_copter_session(const telemetry::VehicleState &state, const telemetry::VehicleState &expected) {
    return expected.session_id != 0 && expected.system_id != 0 && expected.component_id != 0 &&
           expected.identity.aircraft_class == telemetry::AircraftClass::Copter &&
           telemetry::identify_vehicle(expected.identity.autopilot_type, expected.identity.vehicle_type)
               .aircraft_class == telemetry::AircraftClass::Copter &&
           same_vehicle_session(state, expected);
}

inline bool heartbeat_is_fresh(const telemetry::VehicleState &state, std::chrono::milliseconds timeout,
                               std::chrono::steady_clock::time_point now) {
    return state.connected && state.heartbeat_fresh && timestamp_is_fresh(state.heartbeat_updated_at, timeout, now);
}

} // namespace nomad::vehicle::detail
