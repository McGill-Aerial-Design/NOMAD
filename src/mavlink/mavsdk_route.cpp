// SPDX-License-Identifier: Apache-2.0
// Fixed-wing GUIDED reposition requests use MAVSDK's typed COMMAND_INT path.

#include "mavsdk_mavlink_connection.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace nomad::mavlink {

std::optional<CommandAck> MavsdkMavlinkConnection::send_fixed_wing_waypoint(
    const FixedWingWaypointCommand &waypoint, std::uint64_t expected_session_id,
    std::chrono::milliseconds timeout) {
    const auto admission = capture_transmission_admission();
    if (admission && !admission([] {})) {
        return CommandAck{static_cast<std::uint16_t>(MAV_CMD_DO_REPOSITION), 0,
                          CommandAck::Status::AdmissionCancelled};
    }
    std::shared_lock lifetime_lock(plugin_lifetime_mutex_);
    if (!is_connected_unlocked() || !passthrough_ || timeout <= std::chrono::milliseconds::zero()) {
        return std::nullopt;
    }
    if (!std::isfinite(waypoint.latitude_deg) || waypoint.latitude_deg < -90.0 || waypoint.latitude_deg > 90.0 ||
        !std::isfinite(waypoint.longitude_deg) || waypoint.longitude_deg < -180.0 || waypoint.longitude_deg > 180.0 ||
        !std::isfinite(waypoint.relative_altitude_m) || !std::isfinite(waypoint.loiter_radius_m) ||
        waypoint.loiter_radius_m <= 0.0F) {
        return std::nullopt;
    }

    mavsdk::MavlinkPassthrough::CommandInt command{};
    command.target_sysid = target_system_;
    command.target_compid = target_component_;
    command.command = MAV_CMD_DO_REPOSITION;
    command.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    command.param1 = 0.0F;
    // Keep CHANGE_MODE clear: the core has already verified QuadPlane GUIDED.
    command.param2 = 0.0F;
    command.param3 = waypoint.loiter_radius_m;
    command.param4 = std::numeric_limits<float>::quiet_NaN();
    command.x = static_cast<std::int32_t>(std::round(waypoint.latitude_deg * 1.0e7));
    command.y = static_cast<std::int32_t>(std::round(waypoint.longitude_deg * 1.0e7));
    command.z = waypoint.relative_altitude_m;

    {
        std::lock_guard observation_lock(observation_mutex_);
        const auto state = state_locked();
        if (expected_session_id == 0 || state.session_id != expected_session_id || !state.connected ||
            !state.heartbeat_fresh) {
            return std::nullopt;
        }
    }
    mavsdk::OperationOptions options{timeout};
    options.transmission_admission = admission;
    const auto result = passthrough_->send_command_int(command, options);
    if (result == mavsdk::MavlinkPassthrough::Result::CommandAdmissionCancelled) {
        return CommandAck{static_cast<std::uint16_t>(MAV_CMD_DO_REPOSITION), 0,
                          CommandAck::Status::AdmissionCancelled};
    }
    const auto result_code = mavsdk_command_result_code(result);
    if (!result_code.has_value()) {
        return std::nullopt;
    }
    return CommandAck{static_cast<std::uint16_t>(MAV_CMD_DO_REPOSITION), *result_code};
}

} // namespace nomad::mavlink
