// SPDX-License-Identifier: Apache-2.0
#include "nomad/vehicle/vehicle.hpp"

#include "command_ids.hpp"
#include "nomad/safety/output.hpp"

#include <algorithm>
#include <cmath>

namespace nomad::vehicle {
namespace {

CommandResult verified(const CommandResult &result, const char *message) {
    return result.success ? CommandResult{true, message, result.acknowledged} : result;
}

} // namespace

CommandResult Vehicle::set_servo(int channel, int pwm_microseconds) {
    const auto decision = safety::validate_servo_command(channel, pwm_microseconds);
    if (!decision.allowed) {
        return {false, decision.message};
    }
    const auto admission = require_operation(VehicleOperation::SetServo);
    if (!admission.success) {
        return admission;
    }
    const auto result =
        send_command(make_command(kSetServoCommand,
                                  {static_cast<float>(channel), static_cast<float>(pwm_microseconds), 0, 0, 0, 0, 0}),
                     "set servo");
    return verified(result, "servo command acknowledged");
}

CommandResult Vehicle::set_relay(int relay_number, bool on) {
    if (!relay_number_is_valid(relay_number)) {
        return {false, kRelayRangeMessage};
    }
    const auto admission = require_operation(VehicleOperation::SetRelay);
    if (!admission.success) {
        return admission;
    }
    const auto result = send_command(make_command(kSetRelayCommand,
                                                  {static_cast<float>(relay_number), on ? 1.0F : 0.0F, 0, 0, 0, 0, 0}),
                                     "set relay");
    return verified(result, "relay command acknowledged");
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
    const auto admission = require_operation(VehicleOperation::MotorTest);
    if (!admission.success) {
        return admission;
    }
    const auto clamped_timeout = std::clamp(timeout_seconds, 0.05F, 3.0F);
    const auto result = send_command(
        make_command(kMotorTestCommand,
                     {static_cast<float>(motor_instance), 1.0F, static_cast<float>(pwm_microseconds), clamped_timeout,
                      1.0F, 0, 0}),
        "motor test");
    return verified(result, "motor test command acknowledged");
}

CommandResult Vehicle::configure_gimbal(int mount_mode) {
    if (mount_mode < 0 || mount_mode > 4) {
        return {false, "mount mode must be between zero and four"};
    }
    const auto admission = require_operation(VehicleOperation::ConfigureGimbal);
    if (!admission.success) {
        return admission;
    }
    const auto result = send_command(make_command(kMountConfigureCommand,
                                                  {static_cast<float>(mount_mode), 1.0F, 1.0F, 1.0F, 2.0F, 2.0F,
                                                   2.0F}),
                                     "configure gimbal");
    return verified(result, "gimbal configuration verified");
}

CommandResult Vehicle::set_gimbal_target(double pitch_deg, double roll_deg) {
    if (!std::isfinite(pitch_deg) || pitch_deg < -90.0 || pitch_deg > 90.0) {
        return {false, "gimbal pitch must be finite and between -90 and 90 degrees"};
    }
    if (!std::isfinite(roll_deg) || roll_deg < -30.0 || roll_deg > 30.0) {
        return {false, "gimbal roll must be finite and between -30 and 30 degrees"};
    }
    const auto admission = require_operation(VehicleOperation::SetGimbalTarget);
    if (!admission.success) {
        return admission;
    }
    const auto result = send_command(
        make_command(kMountControlCommand, {static_cast<float>(pitch_deg), static_cast<float>(roll_deg), 0, 0, 0, 0,
                                            2.0F}),
        "set gimbal target");
    return verified(result, "gimbal target command acknowledged");
}

} // namespace nomad::vehicle
