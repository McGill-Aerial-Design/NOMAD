# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Regression tests for command surfaces removed from the ROS adapter."""

from __future__ import annotations

import time

import mavlink_wire as wire

from ros_integration_support import _publish_cmd_vel


def test_ros_command_services_are_not_advertised(ros_session) -> None:
    """Protocol-v1-unavailable flight operations have no ROS service endpoint."""
    node = ros_session["control_node"]
    time.sleep(0.5)
    services = {name for name, _ in node.get_service_names_and_types()}

    for operation in ("arm", "disarm", "land", "rtl"):
        assert f"/nomad/{operation}" not in services


def test_cmd_vel_cannot_emit_a_setpoint_or_flight_command(ros_session) -> None:
    """Publishing the old velocity topic reaches no direct MAVLink mutation."""
    state = ros_session["state"]
    if not ros_session["connected"]:
        assert state.node_connected, "node never connected to the MAVLink telemetry responder"

    before_setpoints = len(state.setpoints)
    before_commands = len(state.command_ids)
    _publish_cmd_vel(ros_session, 1.0, vx=1.0)
    time.sleep(0.2)

    assert len(state.setpoints) == before_setpoints, "removed cmd_vel topic emitted a velocity setpoint"
    after_commands = state.command_ids[before_commands:]
    flight_commands = {
        wire.ARM_DISARM_COMMAND,
        wire.LAND_COMMAND,
        wire.RTL_COMMAND,
        176,  # MAV_CMD_DO_SET_MODE
    }
    assert not flight_commands.intersection(after_commands), (
        f"removed ROS control path emitted flight commands: {after_commands}"
    )


def test_telemetry_observer_emits_no_mavlink_messages(ros_session) -> None:
    """Telemetry observation emits no heartbeats, requests, or aircraft commands."""
    state = ros_session["state"]
    time.sleep(1.5)
    assert not state.outbound_message_ids, (
        "receive-only ROS observer transmitted MAVLink messages: "
        f"message_ids={state.outbound_message_ids}, command_ids={state.command_ids}"
    )
