// SPDX-License-Identifier: Apache-2.0
#include "nomad/mission/executor.hpp"

#include "nomad/safety/velocity.hpp"

#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

namespace nomad::mission {

MissionExecutor::MissionExecutor(vehicle::Vehicle& vehicle) : vehicle_(vehicle) {}

MissionResult MissionExecutor::execute(const Mission& mission) {
    if (mission.empty()) {
        return {false, 0, "mission must contain at least one step"};
    }

    for (std::size_t index = 0; index < mission.size(); ++index) {
        const auto result = execute_step(mission[index]);
        if (!result.success) {
            return {false, index, "step " + std::to_string(index + 1) + ": " + result.message};
        }
    }
    return {true, mission.size(), "mission completed"};
}

vehicle::CommandResult MissionExecutor::execute_step(const Takeoff& step) {
    return vehicle_.takeoff(step.altitude_m);
}

vehicle::CommandResult MissionExecutor::execute_step(const Navigate& step) {
    return vehicle_.goto_location({step.latitude_deg, step.longitude_deg, step.altitude_m});
}

vehicle::CommandResult MissionExecutor::execute_step(const Wait& step) {
    if (!std::isfinite(step.seconds) || step.seconds < 0.0F) {
        return {false, "wait duration must be finite and non-negative"};
    }
    std::this_thread::sleep_for(std::chrono::duration<float>(step.seconds));
    return {true, "wait completed"};
}

vehicle::CommandResult MissionExecutor::execute_step(const Action& step) {
    if (step.name == "guided" || step.name == "set_guided") {
        return vehicle_.set_mode(safety::kGuidedMode);
    }
    if (step.name == "arm") {
        return vehicle_.arm();
    }
    if (step.name == "disarm") {
        return vehicle_.disarm();
    }
    if (step.name == "wait_disarmed") {
        return vehicle_.wait_until_disarmed(std::chrono::seconds(90));
    }
    if (step.name == "land") {
        return vehicle_.land();
    }
    if (step.name == "rtl" || step.name == "return_to_launch") {
        return vehicle_.return_to_launch();
    }
    return {false, "unknown mission action: " + step.name};
}

vehicle::CommandResult MissionExecutor::execute_step(const ReturnToLaunch&) {
    return vehicle_.return_to_launch();
}

vehicle::CommandResult MissionExecutor::execute_step(const Land&) {
    return vehicle_.land();
}

vehicle::CommandResult MissionExecutor::execute_step(const Step& step) {
    return std::visit([this](const auto& value) { return execute_step(value); }, step);
}

}  // namespace nomad::mission
