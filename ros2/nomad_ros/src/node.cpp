// SPDX-License-Identifier: Apache-2.0
// ROS 2 adapter node for the NOMAD C++ core.
//
// The node owns one UDP MAVLink connection and one core Vehicle. Callbacks are
// thin: validate input, translate, call the core API, and return. The core
// owns arming gates, velocity clamping, the watchdog, and telemetry modeling.
// This node deliberately contains no vehicle decisions.

#include "nomad/mavlink/udp_connection.hpp"
#include "nomad/safety/velocity.hpp"
#include "nomad/safety/watchdog.hpp"
#include "nomad/telemetry/state.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "nomad_ros/translation.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace {

constexpr auto kConnectRetryPeriod = std::chrono::milliseconds(1000);

} // namespace

class NomadVehicleNode final : public rclcpp::Node {
  public:
    NomadVehicleNode() : Node("nomad_vehicle_node") {
        declare_parameter<std::string>("endpoint", "udpin:0.0.0.0:14570");
        declare_parameter<double>("publish_rate_hz", 10.0);
        declare_parameter<double>("min_vio_confidence", 0.3);
        declare_parameter<int>("vio_timeout_ms", 1000);
        declare_parameter<int>("command_timeout_ms", 500);

        endpoint_ = get_parameter("endpoint").as_string();
        const auto publish_rate = get_parameter("publish_rate_hz").as_double();
        const auto min_vio_confidence = static_cast<float>(get_parameter("min_vio_confidence").as_double());
        vio_timeout_ = std::chrono::milliseconds(get_parameter("vio_timeout_ms").as_int());
        watchdog_policy_ = nomad::safety::WatchdogPolicy{
            std::chrono::milliseconds(get_parameter("command_timeout_ms").as_int()),
            vio_timeout_,
            std::chrono::milliseconds(50),
            min_vio_confidence,
        };

        const auto publish_period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / publish_rate));
        connect_timer_ = create_wall_timer(kConnectRetryPeriod, [this] { ensure_connected(); });
        telemetry_timer_ = create_wall_timer(publish_period, [this] { publish_telemetry(); });

        fix_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("/nomad/fix", 5);
        battery_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>("/nomad/battery", 5);
        odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/nomad/odom", 5);
        connected_publisher_ = create_publisher<std_msgs::msg::Bool>("/nomad/connected", 1);

        velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/nomad/cmd_vel", rclcpp::SensorDataQoS().keep_last(1),
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr twist) { on_velocity_command(twist); });
        vio_health_subscription_ = create_subscription<std_msgs::msg::Bool>(
            "/nomad/vio_health", rclcpp::SensorDataQoS().keep_last(1),
            [this](const std_msgs::msg::Bool::SharedPtr message) { on_vio_health(message); });
        vio_confidence_subscription_ = create_subscription<std_msgs::msg::Float32>(
            "/nomad/vio_confidence", rclcpp::SensorDataQoS().keep_last(1),
            [this](const std_msgs::msg::Float32::SharedPtr message) { on_vio_confidence(message); });

        create_trigger_service("arm", &nomad::vehicle::Vehicle::arm);
        create_trigger_service("disarm", &nomad::vehicle::Vehicle::disarm);
        create_trigger_service("land", &nomad::vehicle::Vehicle::land);
        create_trigger_service("rtl", &nomad::vehicle::Vehicle::return_to_launch);
    }

  private:
    using TriggerService = std_srvs::srv::Trigger;

    void ensure_connected() {
        if (vehicle_ != nullptr) {
            return;
        }
        if (connection_ == nullptr) {
            connection_ = std::make_unique<nomad::mavlink::UdpMavlinkConnection>(endpoint_);
        }
        if (!connection_->is_connected() && !connection_->connect()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "could not connect to %s", endpoint_.c_str());
            return;
        }
        if (!connection_->wait_for_heartbeat(std::chrono::seconds(1)).has_value()) {
            return;
        }
        vehicle_ = std::make_unique<nomad::vehicle::Vehicle>(*connection_, watchdog_policy_);
        RCLCPP_INFO(get_logger(), "connected to the NOMAD core vehicle");
    }

    void publish_telemetry() {
        if (vehicle_ == nullptr) {
            return;
        }
        const auto state = vehicle_->wait_for_state(std::chrono::milliseconds(100));
        if (!state.has_value()) {
            std_msgs::msg::Bool connected;
            connected.data = false;
            connected_publisher_->publish(connected);
            return;
        }
        if (state->position_valid) {
            auto fix = nomad_ros::fix_from_state(*state);
            fix.header.stamp = now();
            fix_publisher_->publish(fix);
        }
        if (state->battery_valid) {
            auto battery = nomad_ros::battery_from_state(*state);
            battery.header.stamp = now();
            battery_publisher_->publish(battery);
        }
        auto odom = nomad_ros::odom_from_state(*state);
        odom.header.stamp = now();
        odom_publisher_->publish(odom);

        std_msgs::msg::Bool connected;
        connected.data = state->connected;
        connected_publisher_->publish(connected);
    }

    void on_velocity_command(const geometry_msgs::msg::TwistStamped::SharedPtr twist) {
        if (vehicle_ == nullptr) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "velocity command before vehicle is connected");
            return;
        }
        const auto command = nomad_ros::velocity_from_twist(*twist);
        if (!command.has_value()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "velocity command contains non-finite values");
            return;
        }
        const auto vio = take_latest_vio_sample();
        if (!vio.updated) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "velocity command without a VIO feed");
            return;
        }
        const auto vio_result = vehicle_->update_vio(vio.healthy, vio.confidence);
        if (!vio_result.success) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "VIO update rejected: %s",
                                 vio_result.message.c_str());
            return;
        }
        const auto result = vehicle_->set_velocity(*command);
        if (!result.success) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "velocity rejected: %s", result.message.c_str());
        }
    }

    void on_vio_health(const std_msgs::msg::Bool::SharedPtr message) {
        std::lock_guard lock(vio_mutex_);
        vio_sample_.healthy = message->data;
        vio_sample_.updated = true;
        vio_sample_.updated_at = std::chrono::steady_clock::now();
    }

    void on_vio_confidence(const std_msgs::msg::Float32::SharedPtr message) {
        std::lock_guard lock(vio_mutex_);
        vio_sample_.confidence = message->data;
        vio_sample_.updated = true;
        vio_sample_.updated_at = std::chrono::steady_clock::now();
    }

    struct VioSample {
        bool healthy{false};
        float confidence{0.0F};
        bool updated{false};
        std::chrono::steady_clock::time_point updated_at{};
    };

    VioSample take_latest_vio_sample() {
        std::lock_guard lock(vio_mutex_);
        VioSample sample = vio_sample_;
        if (!sample.updated) {
            return sample;
        }
        const bool fresh = std::chrono::steady_clock::now() - sample.updated_at <= vio_timeout_;
        sample.updated = fresh;
        return sample;
    }

    void create_trigger_service(const std::string &name, nomad::vehicle::CommandResult (nomad::vehicle::Vehicle::*method)()) {
        services_.push_back(create_service<TriggerService>(
            "/nomad/" + name, [this, method](const TriggerService::Request::SharedPtr,
                                             TriggerService::Response::SharedPtr response) {
                if (vehicle_ == nullptr) {
                    response->success = false;
                    response->message = "vehicle is not connected";
                    return;
                }
                const auto result = (vehicle_.get()->*method)();
                response->success = result.success;
                response->message = result.message;
            }));
    }

    std::string endpoint_;
    std::chrono::milliseconds vio_timeout_;
    nomad::safety::WatchdogPolicy watchdog_policy_;

    std::unique_ptr<nomad::mavlink::UdpMavlinkConnection> connection_;
    std::unique_ptr<nomad::vehicle::Vehicle> vehicle_;

    rclcpp::TimerBase::SharedPtr connect_timer_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vio_health_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vio_confidence_subscription_;
    std::vector<rclcpp::Service<TriggerService>::SharedPtr> services_;

    std::mutex vio_mutex_;
    VioSample vio_sample_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NomadVehicleNode>());
    rclcpp::shutdown();
    return 0;
}
