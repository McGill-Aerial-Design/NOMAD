// SPDX-License-Identifier: Apache-2.0
#include "command_table.hpp"

namespace {

constexpr DirectCommand kCommands[] = {
    {"connect", false},
    {"status", false},
    {"arm", true},
    {"disarm", true},
    {"mode", true},
    {"takeoff", true},
    {"vtol-takeoff", true},
    {"transition-to-fixed-wing", true},
    {"fixed-wing-route", true},
    {"fixed-wing-recovery", true},
    {"transition-to-vtol", true},
    {"quadplane-vtol-land", true},
    {"goto", true},
    {"land", true},
    {"rtl", true},
    {"servo", true},
    {"relay", true},
    {"motor-test", true},
    {"gimbal-config", true},
    {"mission-demo", true},
    {"velocity", true},
    {"velocity-demo", true},
    {"fence-demo", true},
    {"payload-demo", true},
};

} // namespace

std::span<const DirectCommand> direct_commands() {
    return kCommands;
}

bool is_supported_direct_command(std::string_view command) {
    for (const auto &entry : direct_commands()) {
        if (entry.name == command) {
            return true;
        }
    }
    return false;
}

bool is_direct_actuation_command(std::string_view command) {
    for (const auto &entry : direct_commands()) {
        if (entry.name == command) {
            return entry.actuation;
        }
    }
    // parse_direct_arguments rejects an unknown verb before this is consulted; an
    // unknown verb is never treated as actuation.
    return false;
}
