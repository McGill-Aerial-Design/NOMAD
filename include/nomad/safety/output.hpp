// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>

namespace nomad::safety {

inline constexpr int kMinimumServoChannel = 1;
inline constexpr int kMaximumServoChannel = 16;
inline constexpr int kMinimumPwmMicroseconds = 500;
inline constexpr int kMaximumPwmMicroseconds = 2500;

struct OutputDecision {
    bool allowed{false};
    std::string reason;
    std::string message;
};

OutputDecision validate_servo_command(int channel, int pwm_microseconds);

} // namespace nomad::safety
