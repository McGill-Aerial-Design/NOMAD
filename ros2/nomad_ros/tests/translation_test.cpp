// SPDX-License-Identifier: Apache-2.0
#include "nomad_ros/translation.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

nomad::telemetry::VehicleState make_state() {
    nomad::telemetry::VehicleState state;
    state.connected = true;
    state.heartbeat_fresh = true;
    state.armed = true;
    state.position.latitude_deg = 42.3898;
    state.position.longitude_deg = -71.1476;
    state.position.altitude_m = 14.1F;
    state.position.relative_altitude_m = 5.0F;
    state.velocity.north_mps = 1.0F;
    state.velocity.east_mps = -2.0F;
    state.velocity.down_mps = 0.5F;
    state.battery.voltage_v = 12.6F;
    state.battery.remaining_percent = 50.0F;
    state.position_valid = true;
    state.battery_valid = true;
    state.attitude_valid = true;
    return state;
}

TEST(Translation, fix_carries_global_position) {
    const auto fix = nomad_ros::fix_from_state(make_state());

    ASSERT_EQ(fix.status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
    EXPECT_DOUBLE_EQ(fix.latitude, 42.3898);
    EXPECT_DOUBLE_EQ(fix.longitude, -71.1476);
    EXPECT_FLOAT_EQ(fix.altitude, 14.1F);
}

TEST(Translation, fix_reports_no_fix_without_position) {
    auto state = make_state();
    state.position_valid = false;

    const auto fix = nomad_ros::fix_from_state(state);

    EXPECT_EQ(fix.status.status, sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX);
}

TEST(Translation, battery_carries_voltage_and_percentage) {
    const auto battery = nomad_ros::battery_from_state(make_state());

    ASSERT_TRUE(battery.present);
    EXPECT_FLOAT_EQ(battery.voltage, 12.6F);
    EXPECT_FLOAT_EQ(battery.percentage, 0.5F);
}

TEST(Translation, odom_carries_ned_velocity_altitude_and_orientation) {
    auto state = make_state();
    state.attitude.roll_deg = 0.0F;
    state.attitude.pitch_deg = 0.0F;
    state.attitude.yaw_deg = 0.0F;

    const auto odom = nomad_ros::odom_from_state(state);

    EXPECT_EQ(odom.header.frame_id, "ned");
    EXPECT_FLOAT_EQ(odom.pose.pose.position.z, 5.0F);
    EXPECT_FLOAT_EQ(odom.twist.twist.linear.x, 1.0F);
    EXPECT_FLOAT_EQ(odom.twist.twist.linear.y, -2.0F);
    EXPECT_FLOAT_EQ(odom.twist.twist.linear.z, 0.5F);
    EXPECT_NEAR(odom.pose.pose.orientation.w, 1.0, 1e-6);
}

TEST(Translation, odom_applies_yaw_to_orientation) {
    auto state = make_state();
    state.attitude.roll_deg = 0.0F;
    state.attitude.pitch_deg = 0.0F;
    state.attitude.yaw_deg = 180.0F;

    const auto odom = nomad_ros::odom_from_state(state);

    EXPECT_NEAR(odom.pose.pose.orientation.z, 1.0, 1e-6);
    EXPECT_NEAR(odom.pose.pose.orientation.w, 0.0, 1e-6);
}

TEST(Translation, velocity_converts_flu_to_frd) {
    geometry_msgs::msg::TwistStamped twist;
    twist.twist.linear.x = 1.0;
    twist.twist.linear.y = 2.0;
    twist.twist.linear.z = -3.0;
    twist.twist.angular.z = 0.5;

    const auto command = nomad_ros::velocity_from_twist(twist);

    ASSERT_TRUE(command.has_value());
    EXPECT_FLOAT_EQ(command->vx, 1.0F);
    EXPECT_FLOAT_EQ(command->vy, -2.0F);
    EXPECT_FLOAT_EQ(command->vz, 3.0F);
    EXPECT_FLOAT_EQ(command->yaw_rate, -0.5F);
}

TEST(Translation, velocity_rejects_nonfinite_input) {
    geometry_msgs::msg::TwistStamped twist;
    twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();

    EXPECT_FALSE(nomad_ros::velocity_from_twist(twist).has_value());
}

} // namespace
