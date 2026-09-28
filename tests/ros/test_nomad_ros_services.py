# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Command-service integration tests for ``nomad_ros``."""

from __future__ import annotations

import time

import pytest
import mavlink_wire as wire

from ros_integration_support import (
    _start_ros_session,
    _stop_ros_session,
    _call_trigger_service,
    _GUIDED_CUSTOM_MODE,
    _CUSTOM_MODE_LAND,
    _CUSTOM_MODE_RTL,
)


def test_integrated_mode_observes_but_inhibits_arm() -> None:
    session = _start_ros_session(integrated_flight_mode=True)
    try:
        state, _, _, _, _, control = session
        before = state.command_ids.count(wire.ARM_DISARM_COMMAND)
        success, message = _call_trigger_service(control[0], "arm")
        assert not success, "integrated ROS adapter must refuse arm"
        assert "inhibited" in message
        assert state.command_ids.count(wire.ARM_DISARM_COMMAND) == before, "refused arm must not reach the aircraft"
    finally:
        _stop_ros_session(session)


def _reset_vehicle(state, armed: bool, custom_mode: int) -> None:
    """Put the responder's vehicle state back to a known starting point."""
    state.armed = armed
    state.custom_mode = custom_mode


def _require_connection(session) -> None:
    if not session["connected"]:
        pytest.fail("node never connected to the MAVLink responder")


def test_arm_service_arms_vehicle(ros_session) -> None:
    """/nomad/arm verifies the authoritative armed bit."""
    _require_connection(ros_session)
    state = ros_session["state"]
    _reset_vehicle(state, armed=False, custom_mode=_GUIDED_CUSTOM_MODE)
    time.sleep(0.2)

    success, message = _call_trigger_service(ros_session["control_node"], "arm")
    assert success, f"arm service failed: {message}"
    assert state.armed, "responder never saw the armed state after the arm service"


def test_disarm_service_disarms_vehicle(ros_session) -> None:
    """/nomad/disarm verifies the cleared armed bit."""
    _require_connection(ros_session)
    state = ros_session["state"]
    _reset_vehicle(state, armed=True, custom_mode=_GUIDED_CUSTOM_MODE)
    time.sleep(0.2)

    success, message = _call_trigger_service(ros_session["control_node"], "disarm")
    assert success, f"disarm service failed: {message}"
    assert not state.armed, "responder still reports armed after the disarm service"
    _reset_vehicle(state, armed=True, custom_mode=_GUIDED_CUSTOM_MODE)


def test_land_service_sets_land_mode(ros_session) -> None:
    """/nomad/land verifies the LAND mode."""
    _require_connection(ros_session)
    state = ros_session["state"]
    _reset_vehicle(state, armed=True, custom_mode=_GUIDED_CUSTOM_MODE)
    time.sleep(0.2)

    success, message = _call_trigger_service(ros_session["control_node"], "land")
    assert success, f"land service failed: {message}"
    assert state.custom_mode == _CUSTOM_MODE_LAND
    _reset_vehicle(state, armed=True, custom_mode=_GUIDED_CUSTOM_MODE)


def test_rtl_service_sets_rtl_mode(ros_session) -> None:
    """/nomad/rtl verifies the RTL mode."""
    _require_connection(ros_session)
    state = ros_session["state"]
    _reset_vehicle(state, armed=True, custom_mode=_GUIDED_CUSTOM_MODE)
    time.sleep(0.2)

    success, message = _call_trigger_service(ros_session["control_node"], "rtl")
    assert success, f"rtl service failed: {message}"
    assert state.custom_mode == _CUSTOM_MODE_RTL
    _reset_vehicle(state, armed=True, custom_mode=_GUIDED_CUSTOM_MODE)


def test_rejected_command_returns_service_failure(ros_session) -> None:
    """An ACKed-as-failed command surfaces as a failed service response."""
    _require_connection(ros_session)
    state = ros_session["state"]
    _reset_vehicle(state, armed=False, custom_mode=_GUIDED_CUSTOM_MODE)
    state.reject_next = True
    time.sleep(0.2)

    success, message = _call_trigger_service(ros_session["control_node"], "arm")
    assert not success, "arm succeeded despite an explicit MAV_RESULT_FAILED ACK"
    assert "rejected" in message.lower(), f"unexpected failure message: {message}"
    assert not state.armed, "vehicle armed despite the rejected command"
