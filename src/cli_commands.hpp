// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mavlink/connection.hpp"
#include "nomad/vehicle/vehicle.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// Default MAVLink endpoint when the operator does not pass --endpoint.
inline constexpr std::string_view kDefaultEndpoint = "udpin:0.0.0.0:14550";

// Parsed one-shot CLI arguments. No field records the transport: MAVSDK is the
// only implementation, so there is nothing left to select.
struct Arguments {
    std::string command;
    std::string endpoint{kDefaultEndpoint};
    std::uint8_t system_id{1};
    bool endpoint_explicit{false};
    bool system_id_explicit{false};
    std::optional<float> altitude;
    std::optional<float> latitude;
    std::optional<float> longitude;
    std::optional<float> velocity_vx;
    std::optional<float> velocity_vy;
    std::optional<float> velocity_vz;
    std::optional<float> velocity_yaw_rate;
    std::optional<std::uint32_t> mode;
    std::optional<int> relay_number;
    std::optional<float> duration_seconds;
    std::optional<int> channel;
    std::optional<int> pwm_microseconds;
    std::optional<bool> relay_on;
    std::optional<int> motor_instance;
    std::optional<float> timeout_seconds;
    std::optional<int> mount_mode;
    std::vector<double> fixed_wing_route_values;
    std::vector<double> fixed_wing_recovery_values;
    std::vector<double> transition_to_vtol_values;
    std::vector<double> quadplane_landing_values;
};

// Defined in cli_arguments.cpp.
std::optional<Arguments> parse_arguments(int argc, char **argv);

// Defined in cli_commands.cpp. The accepted verb surface, its usage text and
// the API-key boundary live in cli_command_table.hpp.
int print_result(const nomad::vehicle::CommandResult &result);

int run_status(nomad::mavlink::MavlinkConnection &connection);

// Defined in cli_velocity.cpp.
int run_velocity(nomad::vehicle::Vehicle &vehicle, float vx, float vy, float vz, float yaw_rate,
                 float duration_seconds);

int run_velocity_demo(nomad::vehicle::Vehicle &vehicle);

// Defined in cli_demos.cpp.
int run_mission_demo(nomad::vehicle::Vehicle &vehicle);

int run_fence_demo(nomad::vehicle::Vehicle &vehicle);

int run_payload_demo(nomad::vehicle::Vehicle &vehicle, int relay_number, float duration_seconds);
