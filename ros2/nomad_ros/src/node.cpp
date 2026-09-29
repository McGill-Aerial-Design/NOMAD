// SPDX-License-Identifier: Apache-2.0
// ROS 2 adapter for read-only NOMAD telemetry observation.

#include "nomad/mavlink/mavlink_observation.hpp"
#include "nomad_ros/translation.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace {

constexpr auto kConnectRetryPeriod = std::chrono::milliseconds(1000);
constexpr auto kDiscoveryTimeout = std::chrono::seconds(1);
constexpr double kDefaultPublishRateHz = 10.0;

} // namespace

class NomadTelemetryNode final : public rclcpp::Node {
  public:
    NomadTelemetryNode() : Node("nomad_vehicle_node") {
        declare_parameter<std::string>("observation_endpoint", "udpin:0.0.0.0:14552");
        declare_parameter<int>("expected_system_id", 1);
        declare_parameter<double>("publish_rate_hz", kDefaultPublishRateHz);

        observation_endpoint_ = get_parameter("observation_endpoint").as_string();
        expected_system_id_ = parse_expected_system_id();
        const auto publish_period = get_publish_period();

        connect_timer_ = create_wall_timer(kConnectRetryPeriod, [this] { ensure_connected(); });
        telemetry_timer_ = create_wall_timer(publish_period, [this] { publish_telemetry(); });

        fix_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("/nomad/fix", 5);
        battery_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>("/nomad/battery", 5);
        connected_publisher_ = create_publisher<std_msgs::msg::Bool>("/nomad/connected", 1);
    }

  private:
    std::uint8_t parse_expected_system_id() {
        const auto configured_id = std::to_string(get_parameter("expected_system_id").as_int());
        const auto parsed_id = nomad::mavsdk_phase_a::parse_system_id(configured_id);
        if (!parsed_id.has_value()) {
            RCLCPP_ERROR(get_logger(), "expected_system_id must be between 1 and 255");
            return 0;
        }
        return *parsed_id;
    }

    std::chrono::milliseconds get_publish_period() {
        const auto rate_hz = get_parameter("publish_rate_hz").as_double();
        if (!std::isfinite(rate_hz) || rate_hz <= 0.0 || rate_hz > 1000.0) {
            RCLCPP_ERROR(get_logger(), "publish_rate_hz must be finite and in (0, 1000]; using %.1f",
                         kDefaultPublishRateHz);
            return std::chrono::milliseconds(100);
        }
        const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / rate_hz));
        return std::max(period, std::chrono::milliseconds(1));
    }

    void report_connect_failure() {
        const auto message = nomad::mavlink::mavlink_observation_error_message(observer_->last_error());
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "telemetry observer connection failed: %s",
                             std::string(message).c_str());
    }

    void ensure_connected() {
        if (expected_system_id_ == 0 || (observer_ && observer_->is_connected())) {
            return;
        }
        if (!observer_) {
            observer_ = std::make_unique<nomad::mavlink::MavlinkObservation>(
                nomad::mavlink::MavlinkObservationOptions{observation_endpoint_, expected_system_id_,
                                                          kDiscoveryTimeout});
        }
        if (!observer_->connect()) {
            report_connect_failure();
            return;
        }
        RCLCPP_INFO(get_logger(), "connected to the read-only MAVLink telemetry observer");
    }

    auto source_stamp(std::int64_t age_ms) const {
        const auto age_seconds = static_cast<double>(age_ms) / 1000.0;
        return now() - rclcpp::Duration::from_seconds(age_seconds);
    }

    void publish_telemetry() {
        std_msgs::msg::Bool connected;
        connected.data = observer_ && observer_->is_connected();
        connected_publisher_->publish(connected);
        if (!connected.data) {
            return;
        }

        const auto snapshot = observer_->get_status();
        if (auto fix = nomad_ros::fix_from_status(snapshot)) {
            const auto fix_age = std::max(snapshot.age_ms, *snapshot.gps_age_ms);
            fix->header.stamp = source_stamp(fix_age);
            fix_publisher_->publish(*fix);
        }
        if (auto battery = nomad_ros::battery_from_status(snapshot)) {
            battery->header.stamp = source_stamp(*snapshot.battery_age_ms);
            battery_publisher_->publish(*battery);
        }
    }

    std::string observation_endpoint_;
    std::uint8_t expected_system_id_{};
    std::unique_ptr<nomad::mavlink::MavlinkObservation> observer_;

    rclcpp::TimerBase::SharedPtr connect_timer_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NomadTelemetryNode>());
    rclcpp::shutdown();
    return 0;
}
