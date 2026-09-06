// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mavlink/udp_connection.hpp"
#include "nomad/vehicle/vehicle.hpp"

int run_status(nomad::mavlink::UdpMavlinkConnection &connection);

int run_velocity(nomad::vehicle::Vehicle &vehicle, float vx, float vy, float vz, float yaw_rate,
                 float duration_seconds);

int run_velocity_demo(nomad::vehicle::Vehicle &vehicle);
