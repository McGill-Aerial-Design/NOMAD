// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/telemetry/state.hpp"

#include <chrono>
#include <cstdint>

namespace nomad::test {

// Starts with invalid telemetry. Each sample setter marks only its own field
// valid and copies the supplied timestamp without refreshing it.
class VehicleStateBuilder {
  public:
    void set_identity(telemetry::VehicleIdentity identity) {
        state_.identity = identity;
    }

    void set_link_state(bool connected, bool heartbeat_fresh) {
        state_.connected = connected;
        state_.heartbeat_fresh = heartbeat_fresh;
    }

    void set_armed(bool armed) {
        state_.armed = armed;
    }

    void set_mode(std::uint32_t mode) {
        state_.custom_mode = mode;
    }

    void set_session(std::uint8_t system_id, std::uint8_t component_id, std::uint64_t session_id) {
        state_.system_id = system_id;
        state_.component_id = component_id;
        state_.session_id = session_id;
    }

    void set_position(telemetry::Position position, std::chrono::steady_clock::time_point updated_at) {
        state_.position = position;
        state_.position_valid = true;
        state_.position_updated_at = updated_at;
    }

    void set_gps(telemetry::Gps gps, std::chrono::steady_clock::time_point updated_at) {
        state_.gps = gps;
        state_.gps_valid = true;
        state_.gps_updated_at = updated_at;
    }

    void set_vtol_state(telemetry::VtolState vtol_state,
                        std::chrono::steady_clock::time_point updated_at) {
        state_.vtol_state = vtol_state;
        state_.vtol_state_valid = true;
        state_.vtol_state_updated_at = updated_at;
    }

    telemetry::VehicleState build() const {
        return state_;
    }

  private:
    telemetry::VehicleState state_{};
};

} // namespace nomad::test
