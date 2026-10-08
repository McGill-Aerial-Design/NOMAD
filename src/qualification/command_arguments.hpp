// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "../cli_commands.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// Inputs for the non-installed direct qualification driver.
struct DirectArguments {
    Arguments common;
    std::optional<float> altitude;
    std::optional<float> latitude;
    std::optional<float> longitude;
    std::optional<float> velocity_vx;
    std::optional<float> velocity_vy;
    std::optional<float> velocity_vz;
    std::optional<float> velocity_yaw_rate;
    std::optional<std::uint32_t> mode;
    std::optional<float> duration_seconds;
    std::vector<double> fixed_wing_route_values;
    std::vector<double> fixed_wing_recovery_values;
    std::vector<double> transition_to_vtol_values;
    std::vector<double> quadplane_landing_values;
};

// Defined in command_arguments.cpp.
std::optional<DirectArguments> parse_direct_arguments(int argc, char **argv);
