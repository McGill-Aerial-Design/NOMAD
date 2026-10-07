// SPDX-License-Identifier: Apache-2.0

#include "mavsdk_vtol_state.hpp"

namespace nomad::mavlink {

telemetry::VtolState to_nomad_vtol_state(mavsdk::Telemetry::VtolState state) {
    switch (state) {
    case mavsdk::Telemetry::VtolState::Undefined:
        return telemetry::VtolState::Undefined;
    case mavsdk::Telemetry::VtolState::TransitionToFw:
        return telemetry::VtolState::TransitionToFixedWing;
    case mavsdk::Telemetry::VtolState::TransitionToMc:
        return telemetry::VtolState::TransitionToMulticopter;
    case mavsdk::Telemetry::VtolState::Mc:
        return telemetry::VtolState::Multicopter;
    case mavsdk::Telemetry::VtolState::Fw:
        return telemetry::VtolState::FixedWing;
    }
    return telemetry::VtolState::Undefined;
}

} // namespace nomad::mavlink
