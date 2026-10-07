// SPDX-License-Identifier: Apache-2.0
// MAVSDK command-result translation shared by the command and typed route paths.

#include "mavsdk_mavlink_connection.hpp"

namespace nomad::mavlink {

std::optional<std::uint8_t> mavsdk_command_result_code(mavsdk::MavlinkPassthrough::Result result) {
    using Result = mavsdk::MavlinkPassthrough::Result;
    switch (result) {
    case Result::Success:
        return 0; // MAV_RESULT_ACCEPTED
    case Result::CommandTemporarilyRejected:
    case Result::CommandBusy:
        return 1; // MAV_RESULT_TEMPORARILY_REJECTED
    case Result::CommandDenied:
        return 2; // MAV_RESULT_DENIED
    case Result::CommandUnsupported:
        return 3; // MAV_RESULT_UNSUPPORTED
    case Result::CommandFailed:
        return 4; // MAV_RESULT_FAILED
    default:
        return std::nullopt;
    }
}

} // namespace nomad::mavlink
