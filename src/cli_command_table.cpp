// SPDX-License-Identifier: Apache-2.0
#include "cli_command_table.hpp"

#include <cstddef>
#include <iostream>

namespace {

constexpr CliCommand kCommands[] = {
    {"status", ""},
    {"admit", ""},
    {"revoke", ""},
    {"handback", ""},
    {"land", ""},
    {"servo", "<channel> <pwm_us>"},
    {"relay", "<number> <0|1>"},
    {"motor-test", "<instance> <pwm_us> <timeout_s>"},
    {"gimbal-config", "<mount_mode>"},
};

} // namespace

std::span<const CliCommand> cli_commands() {
    return kCommands;
}

bool is_supported_command(std::string_view command) {
    for (const auto &entry : cli_commands()) {
        if (entry.name == command) {
            return true;
        }
    }
    return false;
}

void print_usage() {
    const auto commands = cli_commands();
    std::cout << "Usage: nomad <";
    for (std::size_t index = 0; index < commands.size(); ++index) {
        std::cout << (index == 0 ? "" : "|") << commands[index].name;
    }
    std::cout << "> [value]\n";
    for (const auto &entry : commands) {
        if (!entry.arguments.empty()) {
            std::cout << entry.name << " requires: " << entry.arguments << '\n';
        }
    }
    std::cout << "Commands use NOMAD_RUNTIME_IPC_PORT (default 14611).\n";
}
