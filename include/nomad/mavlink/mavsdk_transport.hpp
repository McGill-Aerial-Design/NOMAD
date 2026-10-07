// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mavlink/connection.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace nomad::mavlink {

// Creates the MAVSDK-backed MavlinkConnection for the given UDP endpoint and
// expected autopilot system ID. MAVSDK is the only transport, so this always
// returns a connection: a link that never appears is reported by connect() and
// wait_for_heartbeat(), not by a missing object.
std::unique_ptr<MavlinkConnection>
make_mavsdk_connection(const std::string &endpoint, std::uint8_t expected_system_id,
                       std::chrono::milliseconds discovery_timeout);

} // namespace nomad::mavlink
