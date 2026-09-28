// SPDX-License-Identifier: Apache-2.0
#include "mavsdk_mavlink_connection.hpp"

#include <cmath>
#include <mutex>
#include <numbers>
#include <shared_mutex>

namespace nomad::mavlink {
namespace {

bool is_zero_setpoint(const VelocitySetpoint &setpoint) {
    return setpoint.vx == 0.0F && setpoint.vy == 0.0F && setpoint.vz == 0.0F && setpoint.yaw_rate == 0.0F;
}

bool has_finite_components(const VelocitySetpoint &setpoint) {
    return std::isfinite(setpoint.vx) && std::isfinite(setpoint.vy) && std::isfinite(setpoint.vz) &&
           std::isfinite(setpoint.yaw_rate);
}

} // namespace

mavsdk::Offboard::Result MavsdkMavlinkConnection::queue_velocity_setpoint(const VelocitySetpoint &setpoint) {
    // The SDK queues one frame; NOMAD owns refresh, freshness and safety zeroes.
    const float yaw_rate_deg_s = setpoint.yaw_rate * (180.0F / std::numbers::pi_v<float>);
    return offboard_->set_velocity_body_once({setpoint.vx, setpoint.vy, setpoint.vz, yaw_rate_deg_s});
}

bool MavsdkMavlinkConnection::send_velocity(const VelocitySetpoint &setpoint) {
    if (!admit_send()) {
        return false;
    }
    std::shared_lock lifetime_lock(plugin_lifetime_mutex_);
    if (!has_finite_components(setpoint) || !offboard_ || target_system_ == 0) {
        return false;
    }
    // A non-zero setpoint needs a live, latched peer. A zero setpoint is the
    // safety command the watchdog and shutdown paths rely on, so it is allowed
    // out on a link the core already believes is dead.
    const bool is_zero = is_zero_setpoint(setpoint);
    if (!is_zero && (!is_connected_unlocked() || !get_state().connected)) {
        return false;
    }
    if (queue_velocity_setpoint(setpoint) != mavsdk::Offboard::Result::Success) {
        return false;
    }
    std::lock_guard lock(observation_mutex_);
    velocity_active_ = !is_zero;
    return true;
}

bool MavsdkMavlinkConnection::is_velocity_active() const {
    std::lock_guard lock(observation_mutex_);
    return velocity_active_;
}

} // namespace nomad::mavlink
