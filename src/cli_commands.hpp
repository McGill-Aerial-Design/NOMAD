// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// Parsed CLI arguments. Runtime requests remain typed; unsupported vehicle
// verbs are parsed so the client can report that protocol v1 does not expose them.
struct Arguments {
    std::string command;
    std::optional<float> altitude;
    std::optional<float> latitude;
    std::optional<float> longitude;
    std::optional<float> velocity_vx;
    std::optional<float> velocity_vy;
    std::optional<float> velocity_vz;
    std::optional<float> velocity_yaw_rate;
    std::optional<std::uint32_t> mode;
    std::optional<int> relay_number;
    std::optional<float> duration_seconds;
    std::optional<int> channel;
    std::optional<int> pwm_microseconds;
    std::optional<bool> relay_on;
    std::optional<int> motor_instance;
    std::optional<float> timeout_seconds;
    std::optional<int> mount_mode;
    std::vector<double> fixed_wing_route_values;
    std::vector<double> fixed_wing_recovery_values;
    std::vector<double> transition_to_vtol_values;
    std::vector<double> quadplane_landing_values;
};

// Defined in cli_arguments.cpp.
std::optional<Arguments> parse_arguments(int argc, char **argv);
