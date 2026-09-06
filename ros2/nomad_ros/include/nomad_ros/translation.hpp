// SPDX-License-Identifier: Apache-2.0
#pragma once

// Pure translation between standard ROS messages and NOMAD core types. No
// vehicle decisions live here; the core remains the only owner of validation,
// gates, and the watchdog.

#include "nomad/safety/velocity.hpp"
#include "nomad/telemetry/state.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <optional>

namespace nomad_ros {

// GPS fix from the core telemetry state. Valid only when the state carries a
// usable position; the returned message still has covariance "unknown" if not.
sensor_msgs::msg::NavSatFix fix_from_state(const nomad::telemetry::VehicleState &state);

// Battery voltage/percentage when the state has battery telemetry.
sensor_msgs::msg::BatteryState battery_from_state(const nomad::telemetry::VehicleState &state);

// Vehicle state as an odometry message. The frame is NED at the vehicle:
//   pose.position.z  = relative altitude (up positive, opposite NED down)
//   twist.linear     = north/east/down velocity in m/s
//   pose.orientation = roll/pitch/yaw from the flight controller
nav_msgs::msg::Odometry odom_from_state(const nomad::telemetry::VehicleState &state);

// Convert a TwistStamped velocity command into the core's FRD velocity
// command. The topic convention is vehicle FLU (x forward, y left, z up,
// yaw counter-clockwise positive); the core uses FRD (x forward, y right,
// z down, yaw clockwise positive). Returns nullopt for any non-finite input.
std::optional<nomad::safety::VelocityCommand> velocity_from_twist(const geometry_msgs::msg::TwistStamped &twist);

} // namespace nomad_ros
