// SPDX-License-Identifier: Apache-2.0
#include "nomad/safety/velocity_config.hpp"

#include "nomad/util/parse.hpp"

#include <limits>

namespace nomad::safety {

namespace {

constexpr float kReviewedMaxVelocityXy = 2.0F;
constexpr float kReviewedMaxVelocityZ = 1.0F;
constexpr float kReviewedMaxYawRate = 1.0F;

}  // namespace

VelocityLimits::VelocityLimits()
    : max_velocity_xy(kReviewedMaxVelocityXy),
      max_velocity_z(kReviewedMaxVelocityZ),
      max_yaw_rate(kReviewedMaxYawRate) {}

VelocityLimits::VelocityLimits(float max_velocity_xy_value, float max_velocity_z_value, float max_yaw_rate_value)
    : max_velocity_xy(max_velocity_xy_value),
      max_velocity_z(max_velocity_z_value),
      max_yaw_rate(max_yaw_rate_value) {}

namespace {

// An unset or empty override keeps the reviewed default; anything malformed or
// negative is rejected so the caller can fail closed.
bool apply_limit(const char *text, float &value) {
    if (text == nullptr || text[0] == '\0') {
        return true;
    }
    const auto parsed = util::parse_non_negative_float(text);
    if (!parsed.has_value()) {
        return false;
    }
    value = *parsed;
    return true;
}

VelocityLimits invalid_limits() {
    const auto invalid = std::numeric_limits<float>::quiet_NaN();
    return VelocityLimits{invalid, invalid, invalid};
}

}  // namespace

VelocityLimits reviewed_velocity_limits() {
    return VelocityLimits{kReviewedMaxVelocityXy, kReviewedMaxVelocityZ, kReviewedMaxYawRate};
}

VelocityLimits load_velocity_limits(float max_velocity_xy, float max_velocity_z, float max_yaw_rate) {
    const VelocityLimits limits{max_velocity_xy, max_velocity_z, max_yaw_rate};
    if (!is_valid_velocity_limits(limits)) {
        return invalid_limits();
    }
    return limits;
}

VelocityLimits load_velocity_limits(const char *xy_env, const char *z_env, const char *yaw_rate_env) {
    auto limits = reviewed_velocity_limits();
    if (!apply_limit(xy_env, limits.max_velocity_xy) || !apply_limit(z_env, limits.max_velocity_z) ||
        !apply_limit(yaw_rate_env, limits.max_yaw_rate)) {
        // A malformed configured limit must never silently widen or disable
        // the envelope. The evaluator rejects this NaN-bearing policy.
        return invalid_limits();
    }
    return limits;
}

}  // namespace nomad::safety
