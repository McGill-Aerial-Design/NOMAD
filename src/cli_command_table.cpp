// SPDX-License-Identifier: Apache-2.0
#include "cli_command_table.hpp"

#include <cstddef>
#include <iostream>

namespace {

// Order matches the historical usage line, so the client-facing verb list stays
// recognizable to anything that scrapes it.
constexpr CliCommand kCommands[] = {
    {"connect", false, ""},
    {"status", false, ""},
    {"admit", true, ""},
    {"revoke", true, ""},
    {"handback", true, ""},
    {"arm", true, ""},
    {"disarm", true, ""},
    {"mode", true, "<custom_mode>"},
    {"takeoff", true, "<altitude_m>"},
    {"vtol-takeoff", true, "<altitude_m>"},
    {"transition-to-fixed-wing", true, ""},
    {"fixed-wing-route", true,
     "<lat1> <lon1> <relative_altitude1_m> <lat2> <lon2> <relative_altitude2_m>"},
    {"fixed-wing-recovery", true, "<latitude> <longitude> <relative_altitude_m>"},
    {"transition-to-vtol", true, "<recovery_latitude> <recovery_longitude> <relative_altitude_m>"},
    {"quadplane-vtol-land", true, "<landing_latitude> <landing_longitude>"},
    {"goto", true, "<latitude> <longitude> <altitude_m>"},
    {"land", true, ""},
    {"rtl", true, ""},
    {"servo", true, "<channel> <pwm_us>"},
    {"relay", true, "<number> <0|1>"},
    {"motor-test", true, "<instance> <pwm_us> <timeout_s>"},
    {"gimbal-config", true, "<mount_mode>"},
    {"mission-demo", true, ""},
    {"velocity", true, "--vx <m_s> [--vy --vz --yaw-rate] --duration <seconds>"},
    {"velocity-demo", true, ""},
    {"fence-demo", true, ""},
    {"payload-demo", true, "<relay_number> <duration_s>"},
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

bool is_actuation_command(std::string_view command) {
    for (const auto &entry : cli_commands()) {
        if (entry.name == command) {
            return entry.actuation;
        }
    }
    // parse_arguments rejects an unknown verb before this is consulted; an
    // unknown verb is never treated as actuation.
    return false;
}

void print_usage() {
    const auto commands = cli_commands();
    std::cout << "Usage: nomad [--runtime|--direct] <";
    for (std::size_t index = 0; index < commands.size(); ++index) {
        std::cout << (index == 0 ? "" : "|") << commands[index].name;
    }
    std::cout << "> [value] [--endpoint udpin:host:port]\n";
    for (const auto &entry : commands) {
        if (!entry.arguments.empty()) {
            std::cout << entry.name << " requires: " << entry.arguments << '\n';
        }
    }
    std::cout << "Actuation commands require the NOMAD_API_KEY environment variable.\n";
    std::cout << "--runtime sends the supported typed requests to NOMAD_RUNTIME_IPC_PORT (default 14611).\n";
    std::cout << "--direct opens a one-shot MAVSDK connection; bare verbs retain this legacy behavior.\n";
}
