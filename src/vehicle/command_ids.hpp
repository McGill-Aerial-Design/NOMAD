// SPDX-License-Identifier: Apache-2.0
#pragma once

// Every MAV_CMD identifier and shared command builder the vehicle layer puts on
// the wire, in one place.
//
// Nothing at a call site says whether the autopilot defines an id, so a wrong
// one survives review until someone exercises the verb against real firmware:
// motor test once sent 139, which is not a MAV_CMD entry at all, while
// MAV_CMD_DO_MOTOR_TEST is 209. tests/test_command_ids.py resolves every id
// declared here against the pinned dialect definition instead, and fails when a
// source declares an id missing from its table.
//
// The MAV_CMD name stays in a trailing comment so a reader can check an id
// without opening the dialect.

#include "nomad/mavlink/connection.hpp"

#include <array>
#include <cstdint>

namespace nomad::vehicle {

constexpr std::uint16_t kArmDisarmCommand = 400;      // MAV_CMD_COMPONENT_ARM_DISARM
constexpr std::uint16_t kSetModeCommand = 176;        // MAV_CMD_DO_SET_MODE
constexpr std::uint32_t kQuadplaneQlandMode = 20;     // Plane.Modes.QLAND
constexpr std::uint16_t kTakeoffCommand = 22;         // MAV_CMD_NAV_TAKEOFF
// ArduPlane's QuadPlane GUIDED handler dispatches this same command id through
// QuadPlane::do_user_takeoff; it is a distinct NOMAD semantic operation.
constexpr std::uint16_t kQuadplaneGuidedTakeoffCommand = 22; // MAV_CMD_NAV_TAKEOFF
constexpr std::uint16_t kQuadplaneTransitionCommand = 3000; // MAV_CMD_DO_VTOL_TRANSITION
constexpr std::uint16_t kDoRepositionCommand = 192; // MAV_CMD_DO_REPOSITION
constexpr std::uint16_t kLandCommand = 21;            // MAV_CMD_NAV_LAND
constexpr std::uint16_t kReturnToLaunchCommand = 20;  // MAV_CMD_NAV_RETURN_TO_LAUNCH
constexpr std::uint16_t kSetServoCommand = 183;       // MAV_CMD_DO_SET_SERVO
constexpr std::uint16_t kSetRelayCommand = 181;       // MAV_CMD_DO_SET_RELAY
constexpr std::uint16_t kMotorTestCommand = 209;      // MAV_CMD_DO_MOTOR_TEST
constexpr std::uint16_t kMountConfigureCommand = 204; // MAV_CMD_DO_MOUNT_CONFIGURE
constexpr std::uint16_t kMountControlCommand = 205;   // MAV_CMD_DO_MOUNT_CONTROL
constexpr std::uint16_t kFenceVertexCommand = 5001;   // MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION

constexpr std::uint8_t kAcceptedResult = 0; // MAV_RESULT_ACCEPTED

inline mavlink::Command make_command(std::uint16_t id, std::array<float, 7> parameters = {}) {
    return mavlink::Command{id, parameters};
}

// One relay range policy for every relay path: set_relay and the payload
// release both validate against these bounds with this message.
constexpr int kMinimumRelayNumber = 0;
constexpr int kMaximumRelayNumber = 15;
constexpr const char *kRelayRangeMessage = "relay number must be between zero and fifteen";

inline bool relay_number_is_valid(int relay_number) {
    return relay_number >= kMinimumRelayNumber && relay_number <= kMaximumRelayNumber;
}

} // namespace nomad::vehicle
