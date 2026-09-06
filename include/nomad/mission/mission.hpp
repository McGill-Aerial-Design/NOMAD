// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <variant>
#include <vector>

namespace nomad::mission {

struct Takeoff {
    float altitude_m{};
};

struct Navigate {
    double latitude_deg{};
    double longitude_deg{};
    float altitude_m{};
};

struct Wait {
    float seconds{};
};

struct Action {
    std::string name;
};

struct ReturnToLaunch {};
struct Land {};

using Step = std::variant<Takeoff, Navigate, Wait, Action, ReturnToLaunch, Land>;
using Mission = std::vector<Step>;

}  // namespace nomad::mission
