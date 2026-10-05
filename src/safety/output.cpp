// SPDX-License-Identifier: Apache-2.0
#include "nomad/safety/output.hpp"

namespace nomad::safety {

OutputDecision validate_servo_command(int channel, int pwm_microseconds) {
    if (channel < kMinimumServoChannel || channel > kMaximumServoChannel) {
        return {false, "channel", "servo channel is outside the supported range"};
    }
    if (pwm_microseconds < kMinimumPwmMicroseconds || pwm_microseconds > kMaximumPwmMicroseconds) {
        return {false, "pwm", "servo PWM is outside the supported range"};
    }
    return {true, "none", "servo command accepted"};
}

} // namespace nomad::safety
