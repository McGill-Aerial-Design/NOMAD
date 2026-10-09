// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <optional>
#include <string>

// Typed inputs supported by the installed runtime client.
struct Arguments {
    std::string command;
    std::optional<int> relay_number;
    std::optional<int> channel;
    std::optional<int> pwm_microseconds;
    std::optional<bool> relay_on;
    std::optional<int> motor_instance;
    std::optional<float> timeout_seconds;
    std::optional<int> mount_mode;
};

// Defined in cli_arguments.cpp.
std::optional<Arguments> parse_arguments(int argc, char **argv);
