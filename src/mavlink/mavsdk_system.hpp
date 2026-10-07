// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

#pragma once

// Shared MAVSDK peer discovery for the connectivity smoke and the
// MavlinkConnection transport, so "which MAVSDK system is our autopilot" is
// answered in exactly one place.

#include "nomad/mavlink/mavsdk_validation.hpp"

#include <mavsdk/mavsdk.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace nomad::mavlink::mavsdk_system {

// Connected systems that advertise an autopilot, in MAVSDK discovery order.
std::vector<std::shared_ptr<mavsdk::System>> connected_autopilots(const mavsdk::Mavsdk &sdk);

// Returns the single autopilot whose system ID matches expected_system_id, or
// nullptr when no autopilot is visible, more than one is visible, or the only
// autopilot is not the expected peer. `selection` always reports which of those
// cases applied so callers can set a specific error.
std::shared_ptr<mavsdk::System> select_expected_autopilot(const mavsdk::Mavsdk &sdk, std::uint8_t expected_system_id,
                                                          mavsdk_phase_a::SystemSelection &selection);

} // namespace nomad::mavlink::mavsdk_system
