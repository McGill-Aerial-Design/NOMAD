// SPDX-License-Identifier: Apache-2.0
#include "arguments.hpp"

#include <charconv>
#include <string_view>
#include <vector>

namespace {

std::optional<std::uint8_t> parse_system_id(std::string_view value) {
    unsigned int parsed{};
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size() || parsed == 0 || parsed > 255) {
        return std::nullopt;
    }
    return static_cast<std::uint8_t>(parsed);
}

} // namespace

std::optional<QualificationArguments> parse_qualification_arguments(int argc, char **argv) {
    if (argc < 1) {
        return std::nullopt;
    }

    QualificationArguments result{};
    std::vector<char *> command_argv{argv[0]};
    for (int index = 1; index < argc; ++index) {
        const std::string_view token(argv[index]);
        if (token != "--endpoint" && token != "--system-id") {
            command_argv.push_back(argv[index]);
            continue;
        }
        if (index + 1 >= argc) {
            return std::nullopt;
        }

        const std::string_view value(argv[++index]);
        if (token == "--endpoint") {
            result.endpoint = value;
            continue;
        }
        const auto system_id = parse_system_id(value);
        if (!system_id.has_value()) {
            return std::nullopt;
        }
        result.system_id = *system_id;
    }

    const auto command = parse_direct_arguments(static_cast<int>(command_argv.size()), command_argv.data());
    if (!command.has_value()) {
        return std::nullopt;
    }
    result.command = *command;
    return result;
}
