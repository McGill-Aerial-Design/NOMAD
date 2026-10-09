// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "command_arguments.hpp"

#include <cstdint>
#include <optional>
#include <string>

struct QualificationArguments {
    DirectArguments command;
    std::string endpoint{"udpin:0.0.0.0:14550"};
    std::uint8_t system_id{1};
};

std::optional<QualificationArguments> parse_qualification_arguments(int argc, char **argv);
