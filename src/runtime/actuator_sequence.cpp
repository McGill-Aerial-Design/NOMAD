// SPDX-License-Identifier: Apache-2.0
#include "actuator_sequence.hpp"

namespace nomad::runtime::detail {
namespace {

ActuatorStageResult send_stage(const std::function<ActuatorStageResult(bool, int)> &send, bool safe, int pwm) {
    try {
        return send(safe, pwm);
    } catch (...) {
        return {{false, "actuator command raised an exception; do not retry blindly"}, "unknown"};
    }
}

ActuatorStageResult wait_and_send_safe(const ActuatorDecision &decision,
    const std::function<ActuatorStageResult(bool, int)> &send, const std::function<void(int)> &wait) {
    try {
        wait(decision.duration_ms);
    } catch (...) {
        return {{false, "actuator wait was interrupted; explicit safe recovery required"}, "unknown"};
    }
    return send_stage(send, true, decision.definition.safe_pwm);
}

} // namespace

ActuatorSequenceResult run_actuator_sequence(const ActuatorDecision &decision,
    const std::function<ActuatorStageResult(bool safe, int pwm)> &send,
    const std::function<void(int duration_ms)> &wait) {
    ActuatorSequenceResult result;
    result.initial = send_stage(send, decision.safe, decision.target_pwm);
    result.activation_success = !decision.safe && result.initial.command.success;
    const bool staged = result.activation_success && decision.duration_ms > 0;
    if (staged) {
        result.safe = wait_and_send_safe(decision, send, wait);
    }
    const auto &terminal = staged ? result.safe : result.initial;
    result.outcome = terminal.outcome;
    if (staged && (result.outcome == "rejected" || result.outcome == "failed-before-send")) {
        result.outcome = "failed";
    }
    result.success = terminal.command.success && result.outcome == "success";
    result.observed_command_success = terminal.command.success;
    result.message = staged ? "Activation software command succeeded; required safe command: " +
                              terminal.command.message : terminal.command.message;
    return result;
}

} // namespace nomad::runtime::detail
