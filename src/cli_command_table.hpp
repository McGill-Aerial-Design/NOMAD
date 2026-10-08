// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <span>
#include <string_view>

struct CliCommand {
    std::string_view name;
    std::string_view arguments;
};

std::span<const CliCommand> cli_commands();

bool is_supported_command(std::string_view command);

void print_usage();
