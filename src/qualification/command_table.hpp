// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <span>
#include <string_view>

// Qualification verbs and their actuation gate.
struct DirectCommand {
    std::string_view name;
    // True when the direct qualification tool requires NOMAD_API_KEY.
    bool actuation;
};

std::span<const DirectCommand> direct_commands();

bool is_supported_direct_command(std::string_view command);

bool is_direct_actuation_command(std::string_view command);
