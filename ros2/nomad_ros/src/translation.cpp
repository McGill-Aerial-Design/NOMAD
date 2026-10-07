// SPDX-License-Identifier: Apache-2.0
#include "nomad_ros/translation.hpp"

#include <limits>

namespace nomad_ros {

std::optional<sensor_msgs::msg::NavSatFix>
fix_from_status(const nomad::mavlink::MavlinkObservationSnapshot &snapshot) {
    const auto &values = snapshot.values;
    if (!nomad::mavsdk_phase_a::has_valid_position(values) || !nomad::mavsdk_phase_a::has_valid_gps(values) ||
        !nomad::mavsdk_phase_a::has_fresh_position_stream(snapshot.position_updates, snapshot.observation_ms,
                                                          snapshot.age_ms) ||
        !snapshot.gps_age_ms.has_value() || !nomad::mavsdk_phase_a::has_fresh_telemetry(*snapshot.gps_age_ms)) {
        return std::nullopt;
    }

    sensor_msgs::msg::NavSatFix fix;
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.latitude = values.latitude_deg;
    fix.longitude = values.longitude_deg;
    fix.altitude = values.absolute_altitude_m;
    return fix;
}

std::optional<sensor_msgs::msg::BatteryState>
battery_from_status(const nomad::mavlink::MavlinkObservationSnapshot &snapshot) {
    if (!nomad::mavsdk_phase_a::has_valid_battery(snapshot.values) || !snapshot.battery_age_ms.has_value() ||
        !nomad::mavsdk_phase_a::has_fresh_telemetry(*snapshot.battery_age_ms)) {
        return std::nullopt;
    }

    sensor_msgs::msg::BatteryState battery;
    battery.present = true;
    const auto unknown = std::numeric_limits<float>::quiet_NaN();
    battery.temperature = unknown;
    battery.voltage = static_cast<float>(snapshot.values.battery_voltage_v);
    battery.current = unknown;
    battery.charge = unknown;
    battery.capacity = unknown;
    battery.design_capacity = unknown;
    battery.percentage = static_cast<float>(snapshot.values.battery_remaining_percent / 100.0);
    return battery;
}

} // namespace nomad_ros
