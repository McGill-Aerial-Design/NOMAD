// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <optional>
#include <string>

namespace nomad::safety {

inline constexpr int kMinimumServoChannel = 1;
inline constexpr int kMaximumServoChannel = 16;
inline constexpr int kMinimumPwmMicroseconds = 500;
inline constexpr int kMaximumPwmMicroseconds = 2500;
inline constexpr float kMinimumReleaseSeconds = 0.05F;
inline constexpr float kMaximumReleaseSeconds = 5.0F;

struct PayloadDecision {
    bool allowed{false};
    std::string reason;
    std::string message;
};

PayloadDecision validate_servo_command(int channel, int pwm_microseconds);
std::optional<float> clamp_release_duration(float duration_seconds);

class ReleaseInterlock {
public:
    explicit ReleaseInterlock(float arm_window_seconds = 10.0F);

    PayloadDecision arm(float now_seconds);
    PayloadDecision evaluate_release(float now_seconds);
    bool is_armed() const;
    void reset();

private:
    float arm_window_seconds_;
    std::optional<float> armed_at_seconds_;
};

}  // namespace nomad::safety
