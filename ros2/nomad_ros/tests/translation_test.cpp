// SPDX-License-Identifier: Apache-2.0
#include "nomad_ros/translation.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace {

nomad::mavlink::MavlinkObservationSnapshot make_snapshot() {
    nomad::mavlink::MavlinkObservationSnapshot snapshot;
    snapshot.values.latitude_deg = 42.3898;
    snapshot.values.longitude_deg = -71.1476;
    snapshot.values.absolute_altitude_m = 14.1;
    snapshot.values.relative_altitude_m = 8.0;
    snapshot.values.gps_fix_type = 3;
    snapshot.values.satellites = 10;
    snapshot.values.battery_voltage_v = 12.6;
    snapshot.values.battery_remaining_percent = 50.0;
    snapshot.values.flight_mode = 1;
    snapshot.position_updates = 20;
    snapshot.observation_ms = 1900;
    snapshot.age_ms = 50;
    snapshot.gps_age_ms = 75;
    snapshot.battery_age_ms = 100;
    return snapshot;
}

} // namespace

TEST(Translation, fix_carries_valid_fresh_global_position) {
    const auto fix = nomad_ros::fix_from_status(make_snapshot());

    ASSERT_TRUE(fix.has_value());
    EXPECT_EQ(fix->status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
    EXPECT_EQ(fix->status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);
    EXPECT_DOUBLE_EQ(fix->latitude, 42.3898);
    EXPECT_DOUBLE_EQ(fix->longitude, -71.1476);
    EXPECT_DOUBLE_EQ(fix->altitude, 14.1);
}

TEST(Translation, fix_rejects_stale_or_incomplete_navigation_source) {
    auto snapshot = make_snapshot();
    snapshot.age_ms = 1501;
    EXPECT_FALSE(nomad_ros::fix_from_status(snapshot).has_value());

    snapshot = make_snapshot();
    snapshot.gps_age_ms = 1501;
    EXPECT_FALSE(nomad_ros::fix_from_status(snapshot).has_value());

    snapshot = make_snapshot();
    snapshot.gps_age_ms.reset();
    EXPECT_FALSE(nomad_ros::fix_from_status(snapshot).has_value());

    snapshot = make_snapshot();
    snapshot.position_updates = 2;
    EXPECT_FALSE(nomad_ros::fix_from_status(snapshot).has_value());
}

TEST(Translation, fix_rejects_invalid_source_values) {
    auto snapshot = make_snapshot();
    snapshot.values.latitude_deg = 91.0;
    EXPECT_FALSE(nomad_ros::fix_from_status(snapshot).has_value());

    snapshot = make_snapshot();
    snapshot.values.absolute_altitude_m = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(nomad_ros::fix_from_status(snapshot).has_value());

    snapshot = make_snapshot();
    snapshot.values.gps_fix_type = 2;
    EXPECT_FALSE(nomad_ros::fix_from_status(snapshot).has_value());
}

TEST(Translation, battery_carries_valid_fresh_voltage_and_percentage) {
    const auto battery = nomad_ros::battery_from_status(make_snapshot());

    ASSERT_TRUE(battery.has_value());
    EXPECT_TRUE(battery->present);
    EXPECT_FLOAT_EQ(battery->voltage, 12.6F);
    EXPECT_FLOAT_EQ(battery->percentage, 0.5F);
    EXPECT_TRUE(std::isnan(battery->current));
    EXPECT_TRUE(std::isnan(battery->charge));
    EXPECT_TRUE(std::isnan(battery->capacity));
}

TEST(Translation, battery_rejects_missing_stale_and_invalid_source_values) {
    auto snapshot = make_snapshot();
    snapshot.battery_age_ms = 1501;
    EXPECT_FALSE(nomad_ros::battery_from_status(snapshot).has_value());

    snapshot = make_snapshot();
    snapshot.battery_age_ms.reset();
    EXPECT_FALSE(nomad_ros::battery_from_status(snapshot).has_value());

    snapshot = make_snapshot();
    snapshot.values.battery_remaining_percent = 101.0;
    EXPECT_FALSE(nomad_ros::battery_from_status(snapshot).has_value());

    snapshot = make_snapshot();
    snapshot.values.battery_voltage_v = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(nomad_ros::battery_from_status(snapshot).has_value());
}
