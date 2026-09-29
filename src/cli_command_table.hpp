// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <span>
#include <string_view>

// One recognized CLI verb and the positional values it takes (empty when it
// takes none). Direct actuation is used by the non-installed qualification tool.
//
// This table is the single source of truth for the accepted verb surface, the
// usage text and the API-key boundary. Adding a verb used to mean editing three
// places; now it is one row, and the remaining question — is it dispatched? —
// is reported at runtime instead of failing silently.
struct CliCommand {
    std::string_view name;
    // True when the direct qualification tool requires NOMAD_API_KEY.
    bool actuation;
    std::string_view arguments;
};

std::span<const CliCommand> cli_commands();

bool is_supported_command(std::string_view command);

bool is_actuation_command(std::string_view command);

void print_usage();
