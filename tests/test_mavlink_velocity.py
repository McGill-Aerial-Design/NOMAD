# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for the pure logic of edge_core.ros_http_bridge.mavlink_velocity.

Covers velocity clamping, the finite-value guard, env parsing, and the FLU->FRD
frame conversion (without opening a real MAVLink link).
"""

from __future__ import annotations

import math

import pytest

from edge_core.ros_http_bridge import mavlink_velocity as mv
from edge_core.ros_http_bridge.mavlink_velocity import (
    MavlinkVelocityController,
    _read_positive_float,
)
from edge_core.safety import clamp


def test_clamp_within_and_outside_bounds():
    assert clamp(0.5, -1.0, 1.0) == 0.5
    assert clamp(5.0, -1.0, 1.0) == 1.0
    assert clamp(-5.0, -1.0, 1.0) == -1.0


def test_clamp_rejects_nonfinite():
    for bad in (math.inf, -math.inf, math.nan):
        with pytest.raises(ValueError):
            clamp(bad, -1.0, 1.0)


def test_read_positive_float_valid(monkeypatch):
    monkeypatch.setenv("NOMAD_TEST_FLOAT", "2.5")
    assert _read_positive_float("NOMAD_TEST_FLOAT", 1.0) == 2.5


def test_read_positive_float_falls_back(monkeypatch):
    monkeypatch.setenv("NOMAD_TEST_FLOAT", "-3")
    assert _read_positive_float("NOMAD_TEST_FLOAT", 1.0) == 1.0
    monkeypatch.setenv("NOMAD_TEST_FLOAT", "garbage")
    assert _read_positive_float("NOMAD_TEST_FLOAT", 0.7) == 0.7


def test_submit_frame_conversion_negates_y_z_yaw(monkeypatch):
    """ROS FLU cmd_vel -> MAVLink FRD: vy, vz and yaw_rate are negated."""
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552", require_armed=False, require_guided=False)

    sent: list[tuple[float, float, float, float]] = []

    # Bypass all gates: pretend the link, heartbeat and VIO are healthy.
    now = 1000.0
    monkeypatch.setattr(mv.time, "monotonic", lambda: now)
    ctrl._conn = object()  # non-None so the link gate passes
    ctrl._last_heartbeat = now
    ctrl._vio_healthy = True
    ctrl._vio_confidence = 1.0
    ctrl._vio_last_update = now

    def fake_send(vx, vy, vz, yaw_rate):
        sent.append((vx, vy, vz, yaw_rate))
        return True

    monkeypatch.setattr(ctrl, "_send_velocity_frd", fake_send)

    # Values stay within the clamp limits so only sign conversion is observed.
    assert ctrl.submit(1.0, 1.5, 0.5, 0.4) is True
    assert sent == [(1.0, -1.5, -0.5, -0.4)]
    assert ctrl.sent_count == 1


def test_submit_clamps_before_sending(monkeypatch):
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552", require_armed=False, require_guided=False)
    sent: list[tuple[float, float, float, float]] = []
    now = 2000.0
    monkeypatch.setattr(mv.time, "monotonic", lambda: now)
    ctrl._conn = object()
    ctrl._last_heartbeat = now
    ctrl._vio_healthy = True
    ctrl._vio_confidence = 1.0
    ctrl._vio_last_update = now
    monkeypatch.setattr(ctrl, "_send_velocity_frd", lambda *a: sent.append(a) or True)

    ctrl.submit(99.0, 99.0, 99.0, 99.0)
    vx, vy, vz, yaw = sent[0]
    assert vx == MavlinkVelocityController.MAX_VELOCITY_XY
    assert vy == -MavlinkVelocityController.MAX_VELOCITY_XY
    assert vz == -MavlinkVelocityController.MAX_VELOCITY_Z
    assert yaw == -MavlinkVelocityController.MAX_YAW_RATE


def test_submit_rejected_without_heartbeat():
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552")
    # No connection / heartbeat -> command refused.
    assert ctrl.submit(1.0, 0.0, 0.0, 0.0) is False
    assert ctrl.rejected_count == 1


def test_stop_sends_zero_velocity(monkeypatch):
    """SR-LNK-03: stop() commands zero velocity before tearing the link down."""
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552")
    sent: list[tuple[float, float, float, float]] = []
    monkeypatch.setattr(ctrl, "_send_velocity_frd", lambda *a: sent.append(a) or True)

    # No threads were started; stop() must still emit a single zero setpoint.
    ctrl.stop()
    assert sent == [(0.0, 0.0, 0.0, 0.0)]
