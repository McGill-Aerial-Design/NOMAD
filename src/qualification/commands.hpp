// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mavlink/connection.hpp"
#include "nomad/vehicle/vehicle.hpp"

int print_result(const nomad::vehicle::CommandResult &result);
int run_status(nomad::mavlink::MavlinkConnection &connection);
int run_velocity(nomad::vehicle::Vehicle &vehicle, float vx, float vy, float vz, float yaw_rate,
                 float duration_seconds);
int run_velocity_demo(nomad::vehicle::Vehicle &vehicle);
int run_mission_demo(nomad::vehicle::Vehicle &vehicle);
int run_fence_demo(nomad::vehicle::Vehicle &vehicle);
int run_payload_demo(nomad::vehicle::Vehicle &vehicle, int relay_number, float duration_seconds);
