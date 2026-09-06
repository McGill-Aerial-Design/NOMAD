// SPDX-License-Identifier: Apache-2.0
#include "nomad_ros/translation.hpp"

#include <cmath>

namespace nomad_ros {

sensor_msgs::msg::NavSatFix fix_from_state(const nomad::telemetry::VehicleState &state) {
    sensor_msgs::msg::NavSatFix fix;
    if (!state.position_valid) {
        fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        return fix;
    }
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.latitude = state.position.latitude_deg;
    fix.longitude = state.position.longitude_deg;
    fix.altitude = state.position.altitude_m;
    return fix;
}

sensor_msgs::msg::BatteryState battery_from_state(const nomad::telemetry::VehicleState &state) {
    sensor_msgs::msg::BatteryState battery;
    if (!state.battery_valid) {
        return battery;
    }
    battery.present = true;
    battery.voltage = state.battery.voltage_v;
    battery.percentage = state.battery.remaining_percent / 100.0F;
    return battery;
}

namespace {

// Roll, pitch, yaw in degrees become a ZYX quaternion in the message frame.
// Only called when the state has a valid attitude.
void fill_orientation(double roll_deg, double pitch_deg, double yaw_deg,
                      geometry_msgs::msg::Quaternion &orientation) {
    constexpr double kDegreesToRadians = 0.017453292519943295;
    const double roll = roll_deg * kDegreesToRadians;
    const double pitch = pitch_deg * kDegreesToRadians;
    const double yaw = yaw_deg * kDegreesToRadians;

    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);

    orientation.w = cy * cp * cr + sy * sp * sr;
    orientation.x = cy * cp * sr - sy * sp * cr;
    orientation.y = cy * sp * cr + sy * cp * sr;
    orientation.z = sy * cp * cr - cy * sp * sr;
}

} // namespace

nav_msgs::msg::Odometry odom_from_state(const nomad::telemetry::VehicleState &state) {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "ned";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.z = state.position.relative_altitude_m;
    if (state.attitude_valid) {
        fill_orientation(state.attitude.roll_deg, state.attitude.pitch_deg, state.attitude.yaw_deg,
                         odom.pose.pose.orientation);
    }
    odom.twist.twist.linear.x = state.velocity.north_mps;
    odom.twist.twist.linear.y = state.velocity.east_mps;
    odom.twist.twist.linear.z = state.velocity.down_mps;
    return odom;
}

std::optional<nomad::safety::VelocityCommand> velocity_from_twist(const geometry_msgs::msg::TwistStamped &twist) {
    // ROS TwistStamped is FLU: x forward, y left, z up, yaw CCW positive.
    // The core command is FRD: x forward, y right, z down, yaw CW positive.
    const float vx = static_cast<float>(twist.twist.linear.x);
    const float vy = static_cast<float>(-twist.twist.linear.y);
    const float vz = static_cast<float>(-twist.twist.linear.z);
    const float yaw_rate = static_cast<float>(-twist.twist.angular.z);
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz) || !std::isfinite(yaw_rate)) {
        return std::nullopt;
    }
    return nomad::safety::VelocityCommand{vx, vy, vz, yaw_rate};
}

} // namespace nomad_ros
