// SPDX-License-Identifier: Apache-2.0
#include "nomad/vehicle/vehicle.hpp"

#include "nomad/mavlink/protocol.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <utility>

namespace nomad::vehicle {
namespace {

constexpr std::uint16_t kArmDisarmCommand = 400;
constexpr std::uint16_t kSetModeCommand = 176;
constexpr std::uint16_t kTakeoffCommand = 22;
constexpr std::uint16_t kLandCommand = 21;
constexpr std::uint16_t kReturnToLaunchCommand = 20;
constexpr std::uint16_t kRepositionCommand = 192;
constexpr std::uint16_t kSetRelayCommand = 181;
constexpr std::uint8_t kAccepted = 0;
constexpr auto kCommandTimeout = std::chrono::seconds(3);
constexpr auto kStateTimeout = std::chrono::seconds(10);
constexpr auto kTakeoffStateTimeout = std::chrono::seconds(30);
constexpr auto kNavigationStateTimeout = std::chrono::seconds(60);
constexpr auto kStatePollTimeout = std::chrono::milliseconds(500);
constexpr auto kMinimumWatchdogPoll = std::chrono::milliseconds(1);
constexpr double kLocationToleranceDegrees = 0.00002;
constexpr float kAltitudeToleranceMeters = 2.0F;
constexpr std::uint32_t kLandMode = 9;
constexpr std::uint32_t kReturnToLaunchMode = 6;

mavlink::Command make_command(std::uint16_t id, std::array<float, 7> parameters = {}) {
    return mavlink::Command{id, parameters};
}

bool is_zero_velocity(const safety::VelocityCommand &command) {
    return command.vx == 0.0F && command.vy == 0.0F && command.vz == 0.0F && command.yaw_rate == 0.0F;
}

bool is_valid_vio_confidence(float confidence) {
    return std::isfinite(confidence) && confidence >= 0.0F && confidence <= 1.0F;
}

float get_monotonic_seconds() {
    static const auto start = std::chrono::steady_clock::now();
    return std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
}

} // namespace

Vehicle::Vehicle(mavlink::MavlinkConnection &connection, safety::WatchdogPolicy watchdog_policy,
                 safety::GlobalFencePolicy fence_policy)
    : connection_(connection), watchdog_policy_(watchdog_policy), fence_policy_(std::move(fence_policy)) {}

Vehicle::~Vehicle() {
    {
        std::lock_guard lock(velocity_mutex_);
        shutting_down_ = true;
        if (velocity_control_active_) {
            connection_.send_velocity({});
            velocity_control_active_ = false;
        }
    }
    velocity_condition_.notify_all();
    if (watchdog_thread_.joinable()) {
        watchdog_thread_.join();
    }
}

std::optional<telemetry::VehicleState> Vehicle::wait_for_state(std::chrono::milliseconds timeout) {
    if (!connection_.is_connected()) {
        return std::nullopt;
    }
    return connection_.wait_for_state(timeout);
}

CommandResult Vehicle::arm() {
    const auto result = send_command(make_command(kArmDisarmCommand, {1, 0, 0, 0, 0, 0, 0}), "arm");
    if (!result.success) {
        return result;
    }
    return wait_for_armed_state(true, "arm");
}

CommandResult Vehicle::disarm() {
    const auto result = send_command(make_command(kArmDisarmCommand), "disarm");
    if (!result.success) {
        return result;
    }
    return wait_for_armed_state(false, "disarm");
}

CommandResult Vehicle::set_mode(std::uint32_t custom_mode) {
    const auto result =
        send_command(make_command(kSetModeCommand, {1, static_cast<float>(custom_mode), 0, 0, 0, 0, 0}), "set mode");
    if (!result.success) {
        return result;
    }
    return wait_for_mode(custom_mode, "set mode");
}

CommandResult Vehicle::takeoff(float altitude_m) {
    if (!std::isfinite(altitude_m) || altitude_m <= 0.0F) {
        return {false, "takeoff altitude must be finite and greater than zero"};
    }
    const auto result = send_command(make_command(kTakeoffCommand, {0, 0, 0, 0, 0, 0, altitude_m}), "takeoff");
    if (!result.success) {
        return result;
    }
    return wait_for_altitude(altitude_m * 0.8F, "takeoff");
}

CommandResult Vehicle::update_vio(bool healthy, float confidence) {
    if (!is_valid_vio_confidence(confidence)) {
        return {false, "VIO confidence must be finite and between zero and one"};
    }
    {
        std::lock_guard lock(velocity_mutex_);
        vio_healthy_ = healthy;
        vio_confidence_ = confidence;
        last_vio_update_ = std::chrono::steady_clock::now();
    }
    velocity_condition_.notify_all();
    return {true, "VIO state updated"};
}

CommandResult Vehicle::set_velocity(const safety::VelocityCommand &command) {
    std::lock_guard lock(velocity_mutex_);
    if (!safety::is_valid_watchdog_policy(watchdog_policy_)) {
        return {false, "velocity watchdog policy is invalid"};
    }
    const auto now = std::chrono::steady_clock::now();
    const auto state = connection_.get_state();
    const auto conditions = get_flight_conditions(state, now);
    const auto decision = safety::evaluate_velocity({}, conditions, command);
    if (!decision.allowed || !decision.setpoint.has_value()) {
        return {false, decision.message};
    }

    const mavlink::VelocitySetpoint setpoint{
        decision.setpoint->vx,
        decision.setpoint->vy,
        decision.setpoint->vz,
        decision.setpoint->yaw_rate,
    };
    if (!connection_.send_velocity(setpoint)) {
        return {false, "velocity command was not sent"};
    }

    last_command_time_ = now;
    last_velocity_stop_reason_ = safety::WatchdogReason::none;
    if (is_zero_velocity(command)) {
        velocity_control_active_ = false;
        velocity_condition_.notify_all();
        return {true, "zero velocity command sent"};
    }
    velocity_control_active_ = true;
    start_watchdog_locked();
    velocity_condition_.notify_all();
    return {true, "velocity command sent"};
}

CommandResult Vehicle::arm_payload() {
    std::lock_guard lock(payload_mutex_);
    const auto result = payload_interlock_.arm(get_monotonic_seconds());
    if (!result.allowed) {
        return {false, result.message};
    }
    return {true, "payload release armed"};
}

CommandResult Vehicle::release_payload(int relay_number, float duration_seconds) {
    const auto duration = safety::clamp_release_duration(duration_seconds);
    if (!duration.has_value()) {
        return {false, "release duration must be finite"};
    }
    if (relay_number < 0 || relay_number > 15) {
        return {false, "relay number must be between zero and fifteen"};
    }

    std::lock_guard lock(payload_mutex_);
    const auto decision = payload_interlock_.evaluate_release(get_monotonic_seconds());
    if (!decision.allowed) {
        return {false, decision.message};
    }
    const auto on_result = send_command(
        make_command(kSetRelayCommand, {static_cast<float>(relay_number), 1.0F, 0, 0, 0, 0, 0}), "payload relay on");
    if (!on_result.success) {
        static_cast<void>(
            send_command(make_command(kSetRelayCommand, {static_cast<float>(relay_number), 0.0F, 0, 0, 0, 0, 0}),
                         "payload relay off"));
        return {false, "payload relay on command failed"};
    }
    std::this_thread::sleep_for(std::chrono::duration<float>(*duration));
    const auto off_result = send_command(
        make_command(kSetRelayCommand, {static_cast<float>(relay_number), 0.0F, 0, 0, 0, 0, 0}), "payload relay off");
    if (!off_result.success) {
        return {false, "payload relay off command failed"};
    }
    return {true, "payload release verified"};
}

CommandResult Vehicle::stop_velocity() {
    std::lock_guard lock(velocity_mutex_);
    const bool sent = connection_.send_velocity({});
    velocity_control_active_ = false;
    last_velocity_stop_reason_ = safety::WatchdogReason::none;
    velocity_condition_.notify_all();
    if (!sent) {
        return {false, "zero velocity command was not sent"};
    }
    return {true, "velocity control stopped"};
}

bool Vehicle::velocity_control_active() const {
    std::lock_guard lock(velocity_mutex_);
    return velocity_control_active_;
}

safety::WatchdogReason Vehicle::last_velocity_stop_reason() const {
    std::lock_guard lock(velocity_mutex_);
    return last_velocity_stop_reason_;
}

CommandResult Vehicle::goto_location(const Location &location) {
    if (!std::isfinite(location.latitude_deg) || location.latitude_deg < -90.0 || location.latitude_deg > 90.0) {
        return {false, "latitude must be finite and between -90 and 90 degrees"};
    }
    if (!std::isfinite(location.longitude_deg) || location.longitude_deg < -180.0 || location.longitude_deg > 180.0) {
        return {false, "longitude must be finite and between -180 and 180 degrees"};
    }
    if (!std::isfinite(location.altitude_m)) {
        return {false, "altitude must be finite"};
    }
    const auto fence_decision =
        safety::evaluate_global_position(fence_policy_, {location.latitude_deg, location.longitude_deg});
    if (!fence_decision.allowed) {
        return {false, fence_decision.message};
    }
    // Altitude is above home (relative), matching takeoff and the GCS guided
    // target conventions. ArduPilot accepts MAV_CMD_DO_REPOSITION only as
    // command_int; the command_long form is answered MAV_RESULT_UNSUPPORTED.
    auto command = make_command(kRepositionCommand, {0, 1, 0, 0, static_cast<float>(location.latitude_deg),
                                                     static_cast<float>(location.longitude_deg), location.altitude_m});
    command.use_command_int = true;
    const auto result = send_command(command, "goto location");
    if (!result.success) {
        return result;
    }
    return wait_for_location(location);
}

CommandResult Vehicle::land() {
    const auto result = send_command(make_command(kLandCommand), "land");
    if (!result.success) {
        return result;
    }
    return wait_for_mode(kLandMode, "land");
}

CommandResult Vehicle::wait_until_disarmed(std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto state = connection_.wait_for_state(kStatePollTimeout);
        if (state.has_value() && !state->armed) {
            return {true, "vehicle disarmed"};
        }
    }
    return {false, "timed out waiting for vehicle to disarm"};
}

CommandResult Vehicle::return_to_launch() {
    const auto result = send_command(make_command(kReturnToLaunchCommand), "return to launch");
    if (!result.success) {
        return result;
    }
    return wait_for_mode(kReturnToLaunchMode, "return to launch");
}

CommandResult Vehicle::upload_fence(const std::vector<safety::GlobalPoint> &boundary) {
    if (boundary.size() < 3 || boundary.size() > 255) {
        return {false, "fence boundary must contain between 3 and 255 points"};
    }

    std::vector<mavlink::FencePlanItem> items;
    items.reserve(boundary.size());
    for (std::size_t index = 0; index < boundary.size(); ++index) {
        const auto &point = boundary[index];
        if (!std::isfinite(point.latitude_deg) || !std::isfinite(point.longitude_deg) ||
            point.latitude_deg < -90.0 || point.latitude_deg > 90.0 || point.longitude_deg < -180.0 ||
            point.longitude_deg > 180.0) {
            return {false, "fence point coordinates are invalid"};
        }
        const mavlink::FencePoint fence_point{
            static_cast<float>(point.latitude_deg),
            static_cast<float>(point.longitude_deg),
        };
        // ArduPilot imports one polygon per command group and reads the group's
        // vertex count from param1 on every vertex item.
        items.push_back({fence_point, static_cast<std::uint16_t>(index), 5001, static_cast<float>(boundary.size())});
    }
    if (!connection_.upload_fence_plan(items)) {
        return {false, "fence plan upload failed"};
    }
    return {true, "fence uploaded"};
}

CommandResult Vehicle::verify_fence_uploaded(const std::vector<safety::GlobalPoint> &expected_boundary) {
    if (expected_boundary.size() < 3 || expected_boundary.size() > 255) {
        return {false, "expected fence boundary size is invalid"};
    }
    // A plan on the autopilot is not a fence: ArduPilot only enforces it when
    // FENCE_ENABLE is set. Read the parameter back as authoritative state.
    const auto fence_enable = connection_.read_param("FENCE_ENABLE", std::chrono::seconds(10));
    if (!fence_enable.has_value()) {
        return {false, "could not read FENCE_ENABLE"};
    }
    if (std::abs(*fence_enable - 1.0F) > 0.01F) {
        return {false, "fence is not enabled (FENCE_ENABLE != 1)"};
    }
    const auto downloaded = connection_.download_fence_plan(std::chrono::seconds(10));
    if (!downloaded.has_value() || downloaded->size() != expected_boundary.size()) {
        return {false, "fence plan readback failed"};
    }
    for (std::size_t index = 0; index < expected_boundary.size(); ++index) {
        const auto expected = expected_boundary[index];
        const auto &actual = downloaded->at(index).point;
        if (std::abs(actual.latitude_deg - expected.latitude_deg) > 0.00001F ||
            std::abs(actual.longitude_deg - expected.longitude_deg) > 0.00001F) {
            return {false, "fence readback does not match the uploaded boundary"};
        }
    }
    return {true, "fence upload verified"};
}

safety::FlightConditions Vehicle::get_flight_conditions(const telemetry::VehicleState &state,
                                                        std::chrono::steady_clock::time_point now) const {
    const bool vio_fresh = last_vio_update_ != std::chrono::steady_clock::time_point{} &&
                           now - last_vio_update_ <= watchdog_policy_.vio_timeout;
    return {
        state.connected, state.heartbeat_fresh,
        state.armed,     state.custom_mode,
        vio_healthy_,    vio_fresh,
        vio_confidence_, watchdog_policy_.min_vio_confidence,
    };
}

safety::WatchdogInput Vehicle::get_watchdog_input(const telemetry::VehicleState &state,
                                                  std::chrono::steady_clock::time_point now) const {
    const bool command_fresh = last_command_time_ != std::chrono::steady_clock::time_point{} &&
                               now - last_command_time_ <= watchdog_policy_.command_timeout;
    const auto conditions = get_flight_conditions(state, now);
    return {
        velocity_control_active_, conditions.connected,   conditions.heartbeat_fresh,
        conditions.armed,         conditions.custom_mode, command_fresh,
        conditions.vio_healthy,   conditions.vio_fresh,   conditions.vio_confidence,
    };
}

void Vehicle::start_watchdog_locked() {
    if (watchdog_thread_.joinable()) {
        return;
    }
    watchdog_thread_ = std::thread(&Vehicle::run_watchdog, this);
}

void Vehicle::run_watchdog() {
    std::unique_lock lock(velocity_mutex_);
    while (!shutting_down_) {
        const auto poll_interval = (std::max)(kMinimumWatchdogPoll, watchdog_policy_.poll_interval);
        velocity_condition_.wait_for(lock, poll_interval);
        if (shutting_down_) {
            return;
        }
        if (!velocity_control_active_) {
            continue;
        }
        const auto state = connection_.get_state();
        const auto input = get_watchdog_input(state, std::chrono::steady_clock::now());
        const auto decision = safety::evaluate_watchdog(watchdog_policy_, input);
        if (decision.stop) {
            send_zero_velocity_locked(decision.reason);
        }
    }
}

bool Vehicle::send_zero_velocity_locked(safety::WatchdogReason reason) {
    const bool sent = connection_.send_velocity({});
    velocity_control_active_ = false;
    last_velocity_stop_reason_ = reason;
    velocity_condition_.notify_all();
    return sent;
}

CommandResult Vehicle::wait_for_armed_state(bool expected, const char *name) {
    const auto deadline = std::chrono::steady_clock::now() + kStateTimeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto state = connection_.wait_for_state(kStatePollTimeout);
        if (state.has_value() && state->armed == expected) {
            return {true, std::string(name) + " verified"};
        }
    }
    return {false, std::string(name) + " acknowledgement received but state verification timed out"};
}

CommandResult Vehicle::wait_for_mode(std::uint32_t expected, const char *name) {
    const auto deadline = std::chrono::steady_clock::now() + kStateTimeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto state = connection_.wait_for_state(kStatePollTimeout);
        if (state.has_value() && state->custom_mode == expected) {
            return {true, std::string(name) + " verified"};
        }
    }
    return {false, std::string(name) + " acknowledgement received but mode verification timed out"};
}

CommandResult Vehicle::wait_for_location(const Location &location) {
    const auto deadline = std::chrono::steady_clock::now() + kNavigationStateTimeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto state = connection_.wait_for_state(kStatePollTimeout);
        if (!state.has_value() || !state->position_valid) {
            continue;
        }
        const auto latitude_error = std::abs(state->position.latitude_deg - location.latitude_deg);
        const auto longitude_error = std::abs(state->position.longitude_deg - location.longitude_deg);
        // Location altitude is above home (the REPOSITION frame is
        // GLOBAL_RELATIVE_ALT_INT), so verify against relative altitude, as
        // wait_for_altitude does for takeoff.
        const auto altitude_error = std::abs(state->position.relative_altitude_m - location.altitude_m);
        if (latitude_error <= kLocationToleranceDegrees && longitude_error <= kLocationToleranceDegrees &&
            altitude_error <= kAltitudeToleranceMeters) {
            return {true, "goto location verified"};
        }
    }
    return {false, "goto location acknowledgement received but position verification timed out"};
}

CommandResult Vehicle::wait_for_altitude(float minimum_altitude_m, const char *name) {
    const auto deadline = std::chrono::steady_clock::now() + kTakeoffStateTimeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto state = connection_.wait_for_state(kStatePollTimeout);
        if (state.has_value() && state->position_valid && state->position.relative_altitude_m >= minimum_altitude_m) {
            return {true, std::string(name) + " verified"};
        }
    }
    return {false, std::string(name) + " acknowledgement received but altitude verification timed out"};
}

CommandResult Vehicle::send_command(const mavlink::Command &command, const char *name) {
    if (!connection_.is_connected()) {
        return {false, "not connected"};
    }

    const auto acknowledgement = connection_.send_command(command, kCommandTimeout);
    if (!acknowledgement.has_value()) {
        return {false, std::string(name) + " timed out waiting for acknowledgement"};
    }
    if (acknowledgement->command != command.id) {
        return {false, std::string(name) + " received an acknowledgement for another command"};
    }
    if (acknowledgement->result != kAccepted) {
        return {false, std::string(name) + " rejected by ArduPilot"};
    }
    return {true, std::string(name) + " accepted"};
}

} // namespace nomad::vehicle
