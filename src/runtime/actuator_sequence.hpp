// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "actuator_state.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include <functional>

namespace nomad::runtime::detail {

struct ActuatorStageResult {
    vehicle::CommandResult command;
    std::string outcome{"not-attempted"};
};

struct ActuatorSequenceResult {
    ActuatorStageResult initial;
    ActuatorStageResult safe;
    bool activation_success{false};
    bool success{false};
    bool observed_command_success{false};
    std::string outcome;
    std::string message;
};

ActuatorSequenceResult run_actuator_sequence(const ActuatorDecision &decision,
    const std::function<ActuatorStageResult(bool safe, int pwm)> &send,
    const std::function<void(int duration_ms)> &wait);

} // namespace nomad::runtime::detail
