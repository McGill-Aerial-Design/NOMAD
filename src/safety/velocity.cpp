// SPDX-License-Identifier: Apache-2.0
#include "nomad/safety/velocity.hpp"

#include <algorithm>
#include <cmath>

namespace nomad::safety {
namespace {

bool is_finite(float value) {
    return std::isfinite(value);
}

bool has_valid_limits(const VelocityLimits& limits) {
    return is_finite(limits.max_velocity_xy) && limits.max_velocity_xy >= 0.0F &&
           is_finite(limits.max_velocity_z) && limits.max_velocity_z >= 0.0F &&
           is_finite(limits.max_yaw_rate) && limits.max_yaw_rate >= 0.0F;
}

bool has_finite_command(const VelocityCommand& command) {
    return is_finite(command.vx) && is_finite(command.vy) && is_finite(command.vz) &&
           is_finite(command.yaw_rate);
}

float clamp(float value, float limit) {
    return std::clamp(value, -limit, limit);
}

}  // namespace

VelocityDecision evaluate_velocity(
    const VelocityLimits& limits,
    const FlightConditions& conditions,
    const VelocityCommand& command) {
    if (!has_valid_limits(limits)) {
        return {false, RejectReason::invalid_limits, "velocity limits are invalid", std::nullopt};
    }
    if (!conditions.connected || !conditions.heartbeat_fresh) {
        return {false, RejectReason::link, "flight-controller heartbeat is stale", std::nullopt};
    }
    if (!conditions.armed) {
        return {false, RejectReason::armed, "vehicle is not armed", std::nullopt};
    }
    if (conditions.custom_mode != kGuidedMode) {
        return {false, RejectReason::mode, "vehicle is not in GUIDED mode", std::nullopt};
    }
    if (!is_finite(conditions.min_vio_confidence) || conditions.min_vio_confidence < 0.0F ||
        conditions.min_vio_confidence > 1.0F || !conditions.vio_healthy || !conditions.vio_fresh ||
        !is_finite(conditions.vio_confidence) || conditions.vio_confidence < conditions.min_vio_confidence) {
        return {false, RejectReason::vio, "VIO is unhealthy, stale, or low confidence", std::nullopt};
    }
    if (!has_finite_command(command)) {
        return {false, RejectReason::nonfinite, "velocity command contains a non-finite value", std::nullopt};
    }

    const VelocityCommand setpoint{
        clamp(command.vx, limits.max_velocity_xy),
        -clamp(command.vy, limits.max_velocity_xy),
        -clamp(command.vz, limits.max_velocity_z),
        -clamp(command.yaw_rate, limits.max_yaw_rate),
    };
    return {true, RejectReason::none, "velocity command accepted", setpoint};
}

}  // namespace nomad::safety
