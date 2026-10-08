// SPDX-License-Identifier: Apache-2.0
// Parse direct qualification inputs before opening the vehicle connection.
#include "command_table.hpp"
#include "command_arguments.hpp"
#include "../cli_command_table.hpp"
#include "nomad/util/parse.hpp"

#include <charconv>
#include <cstdint>
#include <optional>
#include <string_view>

namespace {

using nomad::util::parse_float;
using nomad::util::parse_double;

std::optional<std::uint32_t> parse_mode(std::string_view value) {
    std::uint32_t mode{};
    const auto result = std::from_chars(value.data(), value.data() + value.size(), mode);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size()) {
        return std::nullopt;
    }
    return mode;
}

// Fills the first unset slot; a second positional is rejected.
bool assign_first(std::optional<float> &slot, std::string_view value) {
    if (slot.has_value()) {
        return false;
    }
    slot = parse_float(value);
    return slot.has_value();
}

bool consume_takeoff(DirectArguments &arguments, std::string_view value) {
    return assign_first(arguments.altitude, value);
}

bool consume_goto(DirectArguments &arguments, std::string_view value) {
    std::optional<float> *slot = !arguments.latitude    ? &arguments.latitude
                                 : !arguments.longitude ? &arguments.longitude
                                                        : &arguments.altitude;
    return assign_first(*slot, value);
}

bool consume_fixed_wing_route(DirectArguments &arguments, std::string_view value) {
    if (arguments.fixed_wing_route_values.size() >= 6) {
        return false;
    }
    const auto parsed = parse_double(value);
    if (!parsed.has_value()) {
        return false;
    }
    arguments.fixed_wing_route_values.push_back(*parsed);
    return true;
}

bool consume_fixed_wing_recovery(DirectArguments &arguments, std::string_view value) {
    if (arguments.fixed_wing_recovery_values.size() >= 3) {
        return false;
    }
    const auto parsed = parse_double(value);
    if (!parsed.has_value()) {
        return false;
    }
    arguments.fixed_wing_recovery_values.push_back(*parsed);
    return true;
}

bool consume_transition_to_vtol(DirectArguments &arguments, std::string_view value) {
    if (arguments.transition_to_vtol_values.size() >= 3) {
        return false;
    }
    const auto parsed = parse_double(value);
    if (!parsed.has_value()) {
        return false;
    }
    arguments.transition_to_vtol_values.push_back(*parsed);
    return true;
}

bool consume_quadplane_landing(DirectArguments &arguments, std::string_view value) {
    if (arguments.quadplane_landing_values.size() >= 2) {
        return false;
    }
    const auto parsed = parse_double(value);
    if (!parsed.has_value()) {
        return false;
    }
    arguments.quadplane_landing_values.push_back(*parsed);
    return true;
}

bool consume_mode(DirectArguments &arguments, std::string_view value) {
    if (arguments.mode.has_value()) {
        return false;
    }
    arguments.mode = parse_mode(value);
    return arguments.mode.has_value();
}

bool consume_payload_demo(DirectArguments &arguments, std::string_view value) {
    if (!arguments.common.relay_number.has_value()) {
        const auto parsed = parse_mode(value);
        if (!parsed.has_value() || *parsed > 15) {
            return false;
        }
        arguments.common.relay_number = static_cast<int>(*parsed);
        return true;
    }
    if (!arguments.duration_seconds.has_value()) {
        arguments.duration_seconds = parse_float(value);
        return arguments.duration_seconds.has_value();
    }
    return false;
}

// velocity takes flags rather than positionals; each flag consumes the next
// token, and a repeated flag keeps the last value as it always has.
bool consume_velocity(DirectArguments &arguments, std::string_view flag, int argc, char **argv, int &index) {
    if (index + 1 >= argc) {
        return false;
    }
    std::optional<float> *slot = nullptr;
    if (flag == "--vx") {
        slot = &arguments.velocity_vx;
    } else if (flag == "--vy") {
        slot = &arguments.velocity_vy;
    } else if (flag == "--vz") {
        slot = &arguments.velocity_vz;
    } else if (flag == "--yaw-rate") {
        slot = &arguments.velocity_yaw_rate;
    } else if (flag == "--duration") {
        slot = &arguments.duration_seconds;
    }
    if (slot == nullptr) {
        return false;
    }
    index += 1;
    *slot = parse_float(argv[index]);
    return slot->has_value();
}

bool consume_verb_value(DirectArguments &arguments, std::string_view token, int argc, char **argv, int &index) {
    const std::string_view command = arguments.common.command;
    if (command == "velocity") {
        return consume_velocity(arguments, token, argc, argv, index);
    }
    if (command == "takeoff" || command == "vtol-takeoff") {
        return consume_takeoff(arguments, token);
    }
    if (command == "goto") {
        return consume_goto(arguments, token);
    }
    if (command == "fixed-wing-route") {
        return consume_fixed_wing_route(arguments, token);
    }
    if (command == "fixed-wing-recovery") {
        return consume_fixed_wing_recovery(arguments, token);
    }
    if (command == "transition-to-vtol") {
        return consume_transition_to_vtol(arguments, token);
    }
    if (command == "quadplane-vtol-land") {
        return consume_quadplane_landing(arguments, token);
    }
    if (command == "mode") {
        return consume_mode(arguments, token);
    }
    if (command == "payload-demo") {
        return consume_payload_demo(arguments, token);
    }
    return false;
}

bool consume_token(DirectArguments &arguments, int argc, char **argv, int &index) {
    const std::string_view token(argv[index]);
    return consume_verb_value(arguments, token, argc, argv, index);
}

// Verbs that need every positional they declare refuse a partial invocation
// with usage rather than acting on a default.
bool has_required_arguments(const DirectArguments &arguments) {
    const std::string_view command = arguments.common.command;
    if (command == "payload-demo") {
        return arguments.common.relay_number.has_value() && arguments.duration_seconds.has_value();
    }
    if (command == "mode") {
        return arguments.mode.has_value();
    }
    if (command == "takeoff" || command == "vtol-takeoff") {
        return arguments.altitude.has_value();
    }
    if (command == "velocity") {
        return arguments.velocity_vx.has_value() && arguments.duration_seconds.has_value();
    }
    if (command == "goto") {
        return arguments.latitude.has_value() && arguments.longitude.has_value() && arguments.altitude.has_value();
    }
    if (command == "fixed-wing-route") {
        return arguments.fixed_wing_route_values.size() == 6;
    }
    if (command == "fixed-wing-recovery") {
        return arguments.fixed_wing_recovery_values.size() == 3;
    }
    if (command == "transition-to-vtol") {
        return arguments.transition_to_vtol_values.size() == 3;
    }
    if (command == "quadplane-vtol-land") {
        return arguments.quadplane_landing_values.size() == 2;
    }
    return true;
}

} // namespace

std::optional<DirectArguments> parse_direct_arguments(int argc, char **argv) {
    if (argc < 2 || !is_supported_direct_command(argv[1])) {
        return std::nullopt;
    }

    DirectArguments arguments{};
    if (is_supported_command(argv[1])) {
        const auto common = parse_arguments(argc, argv);
        if (!common.has_value()) {
            return std::nullopt;
        }
        arguments.common = *common;
        return arguments;
    }
    arguments.common.command = argv[1];
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
