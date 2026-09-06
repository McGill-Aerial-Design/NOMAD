// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mission/mission.hpp"
#include "nomad/vehicle/vehicle.hpp"

#include <cstddef>
#include <string>

namespace nomad::mission {

struct MissionResult {
    bool success{false};
    std::size_t completed_steps{0};
    std::string message;
};

class MissionExecutor {
public:
    explicit MissionExecutor(vehicle::Vehicle& vehicle);

    MissionResult execute(const Mission& mission);

private:
    vehicle::CommandResult execute_step(const Takeoff& step);
    vehicle::CommandResult execute_step(const Navigate& step);
    vehicle::CommandResult execute_step(const Wait& step);
    vehicle::CommandResult execute_step(const Action& step);
    vehicle::CommandResult execute_step(const ReturnToLaunch& step);
    vehicle::CommandResult execute_step(const Land& step);
    vehicle::CommandResult execute_step(const Step& step);

    vehicle::Vehicle& vehicle_;
};

}  // namespace nomad::mission
