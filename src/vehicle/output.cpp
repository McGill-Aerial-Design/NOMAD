// SPDX-License-Identifier: Apache-2.0
#include "nomad/vehicle/vehicle.hpp"

#include "nomad/safety/payload.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace nomad::vehicle {
namespace {

constexpr std::uint16_t kSetServoCommand = 183;
constexpr std::uint16_t kSetRelayCommand = 181;
constexpr std::uint16_t kMotorTestCommand = 139;
constexpr std::uint16_t kMountConfigureCommand = 204;
constexpr std::uint16_t kUserCommand = 31010;

mavlink::Command make_command(std::uint16_t id, std::array<float, 7> parameters = {}) {
    return mavlink::Command{id, parameters};
}

CommandResult verified(const CommandResult &result, const char *message) {
    return result.success ? CommandResult{true, message} : result;
}

} // namespace

CommandResult Vehicle::set_servo(int channel, int pwm_microseconds) {
    const auto decision = safety::validate_servo_command(channel, pwm_microseconds);
    if (!decision.allowed) {
        return {false, decision.message};
    }
    const auto result =
        send_command(make_command(kSetServoCommand,
                                  {static_cast<float>(channel), static_cast<float>(pwm_microseconds), 0, 0, 0, 0, 0}),
                     "set servo");
    return verified(result, "servo command verified");
}

CommandResult Vehicle::set_relay(int relay_number, bool on) {
    if (relay_number < 0 || relay_number > 15) {
        return {false, "relay number must be between zero and fifteen"};
    }
    const auto result = send_command(make_command(kSetRelayCommand,
                                                  {static_cast<float>(relay_number), on ? 1.0F : 0.0F, 0, 0, 0, 0, 0}),
                                     "set relay");
    return verified(result, "relay command verified");
}

CommandResult Vehicle::motor_test(int motor_instance, int pwm_microseconds, float timeout_seconds) {
    if (motor_instance < 1) {
        return {false, "motor instance must be one or greater"};
    }
    if (pwm_microseconds != 0 && (pwm_microseconds < 500 || pwm_microseconds > 2500)) {
        return {false, "motor test PWM must be zero or between 500 and 2500 microseconds"};
    }
    if (!std::isfinite(timeout_seconds)) {
        return {false, "motor test timeout must be finite"};
    }
    const auto clamped_timeout = std::clamp(timeout_seconds, 0.05F, 3.0F);
    const auto result = send_command(
        make_command(kMotorTestCommand,
                     {static_cast<float>(motor_instance), 1.0F, static_cast<float>(pwm_microseconds), clamped_timeout,
                      1.0F, 0, 0}),
        "motor test");
    return verified(result, "motor test command verified");
}

CommandResult Vehicle::configure_gimbal(int mount_mode) {
    if (mount_mode < 0 || mount_mode > 4) {
        return {false, "mount mode must be between zero and four"};
    }
    const auto result = send_command(make_command(kMountConfigureCommand,
                                                  {static_cast<float>(mount_mode), 1.0F, 1.0F, 1.0F, 2.0F, 2.0F,
                                                   2.0F}),
                                     "configure gimbal");
    return verified(result, "gimbal configuration verified");
}

CommandResult Vehicle::send_user_command(const std::array<float, 7> &parameters) {
    for (const auto parameter : parameters) {
        if (!std::isfinite(parameter)) {
            return {false, "user command parameters must be finite"};
        }
    }
    const auto result = send_command(make_command(kUserCommand, parameters), "user command");
    return verified(result, "user command verified");
}

} // namespace nomad::vehicle
