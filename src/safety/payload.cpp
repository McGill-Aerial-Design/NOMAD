// SPDX-License-Identifier: Apache-2.0
#include "nomad/safety/payload.hpp"

#include <algorithm>
#include <cmath>

namespace nomad::safety {

PayloadDecision validate_servo_command(int channel, int pwm_microseconds) {
    if (channel < kMinimumServoChannel || channel > kMaximumServoChannel) {
        return {false, "channel", "servo channel is outside the supported range"};
    }
    if (pwm_microseconds < kMinimumPwmMicroseconds || pwm_microseconds > kMaximumPwmMicroseconds) {
        return {false, "pwm", "servo PWM is outside the supported range"};
    }
    return {true, "none", "servo command accepted"};
}

std::optional<float> clamp_release_duration(float duration_seconds) {
    if (!std::isfinite(duration_seconds)) {
        return std::nullopt;
    }
    return std::clamp(duration_seconds, kMinimumReleaseSeconds, kMaximumReleaseSeconds);
}

ReleaseInterlock::ReleaseInterlock(float arm_window_seconds)
    : arm_window_seconds_(arm_window_seconds) {}

PayloadDecision ReleaseInterlock::arm(float now_seconds) {
    if (!std::isfinite(now_seconds) || !std::isfinite(arm_window_seconds_) || arm_window_seconds_ < 0.0F) {
        armed_at_seconds_.reset();
        return {false, "interlock", "payload interlock timing is invalid"};
    }
    armed_at_seconds_ = now_seconds;
    return {true, "none", "payload release armed"};
}

PayloadDecision ReleaseInterlock::evaluate_release(float now_seconds) {
    const auto armed_at = armed_at_seconds_;
    armed_at_seconds_.reset();
    if (!armed_at.has_value()) {
        return {false, "interlock", "payload release requires arming first"};
    }
    if (!std::isfinite(now_seconds) || now_seconds < *armed_at ||
        now_seconds - *armed_at > arm_window_seconds_) {
        return {false, "interlock", "payload release arm window expired"};
    }
    return {true, "none", "payload release accepted"};
}

bool ReleaseInterlock::is_armed() const {
    return armed_at_seconds_.has_value();
}

void ReleaseInterlock::reset() {
    armed_at_seconds_.reset();
}

}  // namespace nomad::safety
