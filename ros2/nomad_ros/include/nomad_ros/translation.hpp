// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mavlink/mavlink_observation.hpp"

#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <optional>

namespace nomad_ros {

std::optional<sensor_msgs::msg::NavSatFix>
fix_from_status(const nomad::mavlink::MavlinkObservationSnapshot &snapshot);

std::optional<sensor_msgs::msg::BatteryState>
battery_from_status(const nomad::mavlink::MavlinkObservationSnapshot &snapshot);

} // namespace nomad_ros
