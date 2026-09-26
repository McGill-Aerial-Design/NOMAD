// SPDX-License-Identifier: Apache-2.0
// ROS 2 adapter node for the NOMAD C++ core.
//
// The node owns one MAVSDK MAVLink connection and one core Vehicle. Callbacks
// are thin: validate input, translate, call the core API, and return. The core
// owns arming gates, velocity clamping, the watchdog, and telemetry modeling.
// This node deliberately contains no vehicle decisions.

#include "nomad/mavlink/mavsdk_transport.hpp"
#include "nomad/safety/velocity_config.hpp"
#include "nomad/safety/vio_source.hpp"
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
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace {

constexpr auto kConnectRetryPeriod = std::chrono::milliseconds(1000);
// MAVSDK's connect() scans for the expected autopilot on the calling thread, so
// the discovery window is kept to the bound the heartbeat wait already spent in
// this timer callback. A vehicle that appears later is caught by the retry.
constexpr auto kDiscoveryTimeout = std::chrono::seconds(1);

} // namespace

class NomadVehicleNode final : public rclcpp::Node {
  public:
    NomadVehicleNode() : Node("nomad_vehicle_node") {
        declare_parameter<std::string>("endpoint", "udpin:0.0.0.0:14570");
        declare_parameter<int>("system_id", 1);
        declare_parameter<double>("publish_rate_hz", 10.0);
        const auto reviewed_limits = nomad::safety::reviewed_velocity_limits();
        declare_parameter<double>("min_vio_confidence", 0.3);
        declare_parameter<double>("max_velocity_xy", reviewed_limits.max_velocity_xy);
        declare_parameter<double>("max_velocity_z", reviewed_limits.max_velocity_z);
        declare_parameter<double>("max_yaw_rate", reviewed_limits.max_yaw_rate);
        declare_parameter<int>("vio_timeout_ms", 1000);
        declare_parameter<int>("command_timeout_ms", 500);
        declare_parameter<std::string>("vio_source", "");
        declare_parameter<std::string>("vio_source_topic", "/nomad/vio_source");

        endpoint_ = get_parameter("endpoint").as_string();
        const auto system_id = static_cast<int>(get_parameter("system_id").as_int());
        if (system_id < 1 || system_id > 255) {
            RCLCPP_ERROR(get_logger(), "system_id %d is not a valid autopilot id; the node will not connect",
                         system_id);
        } else {
            system_id_ = static_cast<std::uint8_t>(system_id);
        }
        const auto publish_rate = get_parameter("publish_rate_hz").as_double();
        const auto min_vio_confidence = static_cast<float>(get_parameter("min_vio_confidence").as_double());
        vio_timeout_ = std::chrono::milliseconds(get_parameter("vio_timeout_ms").as_int());
        watchdog_policy_ = nomad::safety::WatchdogPolicy{
            std::chrono::milliseconds(get_parameter("command_timeout_ms").as_int()),
            vio_timeout_,
            std::chrono::milliseconds(50),
            min_vio_confidence,
        };
        velocity_limits_ = nomad::safety::load_velocity_limits(
            static_cast<float>(get_parameter("max_velocity_xy").as_double()),
            static_cast<float>(get_parameter("max_velocity_z").as_double()),
            static_cast<float>(get_parameter("max_yaw_rate").as_double()));
        vio_source_topic_ = get_parameter("vio_source_topic").as_string();
        vio_validator_ = std::make_unique<nomad::safety::VioSourceValidator>(
            get_parameter("vio_source").as_string());
        if (!nomad::safety::is_valid_velocity_limits(velocity_limits_)) {
            RCLCPP_ERROR(get_logger(), "velocity limits are invalid; velocity commands will be refused");
        }

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
        if (!vio_source_topic_.empty()) {
            vio_source_subscription_ = create_subscription<std_msgs::msg::String>(
                vio_source_topic_, rclcpp::SensorDataQoS().keep_last(1),
                [this](const std_msgs::msg::String::SharedPtr message) { on_vio_source(message); });
        } else {
            RCLCPP_ERROR(get_logger(), "VIO source topic is empty; velocity commands will be refused");
        }

        create_trigger_service("arm", &nomad::vehicle::Vehicle::arm);
        create_trigger_service("disarm", &nomad::vehicle::Vehicle::disarm);
        create_trigger_service("land", &nomad::vehicle::Vehicle::land);
        create_trigger_service("rtl", &nomad::vehicle::Vehicle::return_to_launch);
    }

  private:
    using TriggerService = std_srvs::srv::Trigger;

    // The transport reports which half of the link failed, so the warning names
    // the operator problem instead of one generic message.
    void report_connect_failure() {
        if (connection_->get_connect_failure() == nomad::mavlink::ConnectFailure::NoAutopilot) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no ArduPilot heartbeat on %s",
                                 endpoint_.c_str());
            return;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "could not connect to %s", endpoint_.c_str());
    }

    void ensure_connected() {
        if (vehicle_ != nullptr || system_id_ == 0) {
            return;
        }
        if (connection_ == nullptr) {
            connection_ =
                nomad::mavlink::make_mavsdk_connection(endpoint_, system_id_, kDiscoveryTimeout);
        }
        if (!connection_->is_connected() && !connection_->connect()) {
            report_connect_failure();
            return;
        }
        if (!connection_->wait_for_heartbeat(std::chrono::seconds(1)).has_value()) {
            return;
        }
        nomad::vehicle::VehicleConfig vehicle_config{};
        vehicle_config.watchdog = watchdog_policy_;
        vehicle_config.fence = nomad::safety::GlobalFencePolicy{};
        vehicle_config.velocity = velocity_limits_;
        vehicle_ = std::make_unique<nomad::vehicle::Vehicle>(*connection_, vehicle_config);
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
        if (!vio.has_value()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "velocity command without a fresh VIO feed");
            return;
        }
        const auto validation = vio_validator_->validate(*vio);
        if (!validation.valid) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "VIO sample rejected: %s",
                                 validation.message.c_str());
            return;
        }
        const auto vio_result = vehicle_->update_vio(vio->healthy, vio->confidence);
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
        vio_health_received_ = true;
        vio_health_updated_at_ = std::chrono::steady_clock::now();
    }

    void on_vio_confidence(const std_msgs::msg::Float32::SharedPtr message) {
        std::lock_guard lock(vio_mutex_);
        vio_sample_.confidence = message->data;
        vio_confidence_received_ = true;
        vio_confidence_updated_at_ = std::chrono::steady_clock::now();
    }

    void on_vio_source(const std_msgs::msg::String::SharedPtr message) {
        std::lock_guard lock(vio_mutex_);
        vio_sample_.source = message->data;
        vio_source_received_ = true;
        vio_source_updated_at_ = std::chrono::steady_clock::now();
    }

    std::optional<nomad::safety::VioSample> take_latest_vio_sample() {
        std::lock_guard lock(vio_mutex_);
        if (!vio_health_received_ || !vio_confidence_received_ || !vio_source_received_) {
            return std::nullopt;
        }
        const auto now = std::chrono::steady_clock::now();
        const auto oldest_update =
            std::min({vio_health_updated_at_, vio_confidence_updated_at_, vio_source_updated_at_});
        if (now - oldest_update > vio_timeout_) {
            return std::nullopt;
        }
        auto sample = vio_sample_;
        sample.timestamp = oldest_update;
        return sample;
    }

    void create_trigger_service(
        const std::string &name, nomad::vehicle::CommandResult (nomad::vehicle::Vehicle::*method)()) {
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
    std::string vio_source_topic_;
    std::chrono::milliseconds vio_timeout_;
    nomad::safety::WatchdogPolicy watchdog_policy_;
    nomad::safety::VelocityLimits velocity_limits_;
    std::unique_ptr<nomad::safety::VioSourceValidator> vio_validator_;

    std::uint8_t system_id_{0};
    std::unique_ptr<nomad::mavlink::MavlinkConnection> connection_;
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
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vio_source_subscription_;
    std::vector<rclcpp::Service<TriggerService>::SharedPtr> services_;

    std::mutex vio_mutex_;
    nomad::safety::VioSample vio_sample_;
    bool vio_health_received_{false};
    bool vio_confidence_received_{false};
    bool vio_source_received_{false};
    std::chrono::steady_clock::time_point vio_health_updated_at_{};
    std::chrono::steady_clock::time_point vio_confidence_updated_at_{};
    std::chrono::steady_clock::time_point vio_source_updated_at_{};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NomadVehicleNode>());
    rclcpp::shutdown();
    return 0;
}
