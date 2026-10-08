// SPDX-License-Identifier: Apache-2.0
// Parse one typed command before runtime IPC begins. Reject malformed or extra
// values so the caller can print usage without opening a runtime connection.
#include "cli_command_table.hpp"
#include "cli_commands.hpp"
#include "nomad/util/parse.hpp"

#include <charconv>
#include <cstdint>
#include <optional>
#include <string_view>

namespace {

using nomad::util::parse_float;

std::optional<std::uint32_t> parse_mode(std::string_view value) {
    std::uint32_t mode{};
    const auto result = std::from_chars(value.data(), value.data() + value.size(), mode);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size()) {
        return std::nullopt;
    }
    return mode;
}

std::optional<int> parse_output_int(std::string_view value, std::uint32_t maximum) {
    const auto parsed = parse_mode(value);
    if (!parsed.has_value() || *parsed > maximum) {
        return std::nullopt;
    }
    return static_cast<int>(*parsed);
}

bool consume_servo(Arguments &arguments, std::string_view value) {
    const auto parsed = parse_output_int(value, 65535);
    if (!parsed.has_value()) {
        return false;
    }
    if (!arguments.channel.has_value()) {
        arguments.channel = *parsed;
        return true;
    }
    if (!arguments.pwm_microseconds.has_value()) {
        arguments.pwm_microseconds = *parsed;
        return true;
    }
    return false;
}

bool consume_relay(Arguments &arguments, std::string_view value) {
    if (!arguments.relay_number.has_value()) {
        const auto parsed = parse_output_int(value, 15);
        if (!parsed.has_value()) {
            return false;
        }
        arguments.relay_number = *parsed;
        return true;
    }
    if (!arguments.relay_on.has_value()) {
        const auto parsed = parse_output_int(value, 1);
        if (!parsed.has_value()) {
            return false;
        }
        arguments.relay_on = *parsed == 1;
        return true;
    }
    return false;
}

bool consume_motor_test(Arguments &arguments, std::string_view value) {
    if (!arguments.motor_instance.has_value()) {
        const auto parsed = parse_output_int(value, 65535);
        if (!parsed.has_value()) {
            return false;
        }
        arguments.motor_instance = *parsed;
        return true;
    }
    if (!arguments.pwm_microseconds.has_value()) {
        const auto parsed = parse_output_int(value, 65535);
        if (!parsed.has_value()) {
            return false;
        }
        arguments.pwm_microseconds = *parsed;
        return true;
    }
    if (!arguments.timeout_seconds.has_value()) {
        arguments.timeout_seconds = parse_float(value);
        return arguments.timeout_seconds.has_value();
    }
    return false;
}

bool consume_gimbal_config(Arguments &arguments, std::string_view value) {
    if (arguments.mount_mode.has_value()) {
        return false;
    }
    const auto parsed = parse_output_int(value, 4);
    if (!parsed.has_value()) {
        return false;
    }
    arguments.mount_mode = *parsed;
    return true;
}

bool consume_token(Arguments &arguments, int argc, char **argv, int &index) {
    const std::string_view token(argv[index]);
    if (arguments.command == "servo") {
        return consume_servo(arguments, token);
    }
    if (arguments.command == "relay") {
        return consume_relay(arguments, token);
    }
    if (arguments.command == "motor-test") {
        return consume_motor_test(arguments, token);
    }
    if (arguments.command == "gimbal-config") {
        return consume_gimbal_config(arguments, token);
    }
    return false;
}

bool has_required_arguments(const Arguments &arguments) {
    const std::string_view command = arguments.command;
    if (command == "servo") {
        return arguments.channel.has_value() && arguments.pwm_microseconds.has_value();
    }
    if (command == "relay") {
        return arguments.relay_number.has_value() && arguments.relay_on.has_value();
    }
    if (command == "motor-test") {
        return arguments.motor_instance.has_value() && arguments.pwm_microseconds.has_value() &&
               arguments.timeout_seconds.has_value();
    }
    if (command == "gimbal-config") {
        return arguments.mount_mode.has_value();
    }
    return true;
}

} // namespace

std::optional<Arguments> parse_arguments(int argc, char **argv) {
    if (argc < 2 || !is_supported_command(argv[1])) {
        return std::nullopt;
    }

    Arguments arguments{argv[1]};
    for (int index = 2; index < argc; ++index) {
        if (!consume_token(arguments, argc, argv, index)) {
            return std::nullopt;
        }
    }
    if (!has_required_arguments(arguments)) {
        return std::nullopt;
    }
    return arguments;
}
