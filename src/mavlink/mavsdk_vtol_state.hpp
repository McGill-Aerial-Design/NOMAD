// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/telemetry/state.hpp"

#include <plugins/telemetry/telemetry.hpp>

namespace nomad::mavlink {

telemetry::VtolState to_nomad_vtol_state(mavsdk::Telemetry::VtolState state);

} // namespace nomad::mavlink
