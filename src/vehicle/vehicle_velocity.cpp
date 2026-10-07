// SPDX-License-Identifier: Apache-2.0
// Velocity control and its watchdog: the VIO gate, the safety-evaluated
// setpoint path, and the owned watchdog thread that stops the vehicle when the
// command stream, link, mode or VIO feed goes stale.
//
// Split from vehicle.cpp to keep each file under the source-size policy; all of
// these methods share velocity_mutex_.
#include "nomad/vehicle/vehicle.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>

namespace nomad::vehicle {
namespace {

constexpr auto kMinimumWatchdogPoll = std::chrono::milliseconds(1);

bool is_zero_velocity(const safety::VelocityCommand &command) {
    return command.vx == 0.0F && command.vy == 0.0F && command.vz == 0.0F && command.yaw_rate == 0.0F;
}

bool is_valid_vio_confidence(float confidence) {
    return std::isfinite(confidence) && confidence >= 0.0F && confidence <= 1.0F;
}

} // namespace

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
    const auto admission = require_operation(VehicleOperation::BodyVelocity);
    if (!admission.success) {
        return admission;
    }
    const auto conditions = get_flight_conditions(state, now);
    const auto decision = safety::evaluate_velocity(velocity_limits_, conditions, command);
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

CommandResult Vehicle::stop_velocity() {
    std::lock_guard lock(velocity_mutex_);
    if (!velocity_control_active_) {
        const auto admission = require_operation(VehicleOperation::BodyVelocity);
        if (!admission.success) {
            return admission;
        }
    }
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

safety::FlightConditions Vehicle::get_flight_conditions(const telemetry::VehicleState &state,
                                                        std::chrono::steady_clock::time_point now) const {
    const bool vio_fresh = last_vio_update_ != std::chrono::steady_clock::time_point{} &&
                           now - last_vio_update_ <= watchdog_policy_.vio_timeout;
    return {
        state.connected, state.heartbeat_fresh,
        state.armed,     state.custom_mode,
        vio_healthy_,    vio_fresh,
        vio_confidence_, watchdog_policy_.min_vio_confidence,
        telemetry::guided_mode_for(state.identity.aircraft_class).value_or(0),
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
        conditions.guided_mode,
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

} // namespace nomad::vehicle
