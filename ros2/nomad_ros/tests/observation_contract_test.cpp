// SPDX-License-Identifier: Apache-2.0
#include "nomad/mavlink/connection.hpp"
#include "nomad/mavlink/mavlink_observation.hpp"
#include "nomad/vehicle/vehicle.hpp"

#include <type_traits>
#include <utility>

namespace {

template <typename T>
concept CanSendVelocity = requires(T &connection, const nomad::mavlink::VelocitySetpoint &setpoint) {
    connection.send_velocity(setpoint);
};

template <typename T>
concept CanSendCommand = requires(T &connection, const nomad::mavlink::Command &command) {
    connection.send_command(command, std::chrono::milliseconds(100));
};

static_assert(!CanSendVelocity<nomad::mavlink::MavlinkObservation>);
static_assert(!CanSendCommand<nomad::mavlink::MavlinkObservation>);
static_assert(!std::is_constructible_v<nomad::vehicle::Vehicle, nomad::mavlink::MavlinkObservation &>);

} // namespace
