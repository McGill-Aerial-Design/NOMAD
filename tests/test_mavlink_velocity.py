# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for the pure logic of edge_core.ros_http_bridge.mavlink_velocity.

Covers velocity clamping, the finite-value guard, env parsing, and the FLU->FRD
frame conversion (without opening a real MAVLink link).
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from edge_core.ros_http_bridge import mavlink_velocity as mv
from edge_core.ros_http_bridge.mavlink_velocity import (
    MavlinkVelocityController,
    _read_positive_float,
)
from edge_core.safety import clamp


class _FakeMav:
    def __init__(self):
        self.sent: list[tuple] = []

    def set_position_target_local_ned_send(self, *args):
        self.sent.append(args)


class _FakeConn:
    """Minimal stand-in for a pymavlink connection."""

    def __init__(self):
        self.mav = _FakeMav()
        self.target_system = 1
        self.target_component = 1
        self.closed = False

    def close(self):
        self.closed = True


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


# -- endpoint normalization -------------------------------------------------


def test_bare_host_port_gets_udp_scheme():
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552")
    assert ctrl.endpoint == "udp:127.0.0.1:14552"


@pytest.mark.parametrize("endpoint", ["udp:127.0.0.1:14552", "tcp:10.0.0.1:5760", "udpin:0.0.0.0:14550"])
def test_endpoint_with_scheme_is_left_untouched(endpoint):
    ctrl = MavlinkVelocityController(endpoint=endpoint)
    assert ctrl.endpoint == endpoint


# -- VIO bookkeeping --------------------------------------------------------


def test_note_vio_records_state(monkeypatch):
    monkeypatch.setattr(mv.time, "monotonic", lambda: 123.0)
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552")
    ctrl.note_vio(0.9, healthy=True)
    assert ctrl._vio_confidence == 0.9
    assert ctrl._vio_healthy is True
    assert ctrl._vio_last_update == 123.0


# -- MAVLink TX -------------------------------------------------------------


def test_send_velocity_frd_transmits():
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552")
    conn = _FakeConn()
    ctrl._conn = conn
    assert ctrl._send_velocity_frd(1.0, -2.0, 0.5, 0.1) is True
    assert len(conn.mav.sent) == 1
    # vx/vy/vz/yaw_rate land in the SET_POSITION_TARGET_LOCAL_NED slots.
    args = conn.mav.sent[0]
    assert args[8:11] == (1.0, -2.0, 0.5)
    assert args[-1] == 0.1


def test_send_velocity_frd_without_connection_returns_false():
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552")
    ctrl._conn = None
    assert ctrl._send_velocity_frd(0.0, 0.0, 0.0, 0.0) is False


def test_send_velocity_frd_swallows_link_error():
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552")
    conn = _FakeConn()

    def boom(*args):
        raise RuntimeError("link down")

    conn.mav.set_position_target_local_ned_send = boom
    ctrl._conn = conn
    assert ctrl._send_velocity_frd(1.0, 0.0, 0.0, 0.0) is False


# -- connection lifecycle ---------------------------------------------------


def test_connect_opens_link(monkeypatch):
    conn = _FakeConn()
    monkeypatch.setattr(mv, "MAVLINK_AVAILABLE", True)
    monkeypatch.setattr(mv, "mavutil", SimpleNamespace(mavlink_connection=lambda *a, **k: conn))
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552")
    assert ctrl._connect() is True
    assert ctrl._conn is conn


def test_connect_handles_error(monkeypatch):
    def boom(*a, **k):
        raise OSError("no route to host")

    monkeypatch.setattr(mv, "MAVLINK_AVAILABLE", True)
    monkeypatch.setattr(mv, "mavutil", SimpleNamespace(mavlink_connection=boom))
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552")
    assert ctrl._connect() is False
    assert ctrl._conn is None


def test_connect_returns_false_when_pymavlink_absent(monkeypatch):
    monkeypatch.setattr(mv, "MAVLINK_AVAILABLE", False)
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552")
    assert ctrl._connect() is False


def test_start_disabled_without_pymavlink(monkeypatch):
    monkeypatch.setattr(mv, "MAVLINK_AVAILABLE", False)
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552")
    assert ctrl.start() is False


# -- submit edge cases ------------------------------------------------------


def _open_gates(ctrl, monkeypatch, now):
    monkeypatch.setattr(mv.time, "monotonic", lambda: now)
    ctrl._conn = object()
    ctrl._last_heartbeat = now
    ctrl._vio_healthy = True
    ctrl._vio_confidence = 1.0
    ctrl._vio_last_update = now


def test_submit_counts_rejection_when_send_fails(monkeypatch):
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552", require_armed=False, require_guided=False)
    _open_gates(ctrl, monkeypatch, now=3000.0)
    monkeypatch.setattr(ctrl, "_send_velocity_frd", lambda *a: False)
    assert ctrl.submit(1.0, 0.0, 0.0, 0.0) is False
    assert ctrl.rejected_count == 1
    assert ctrl.sent_count == 0


def test_submit_drops_allowed_decision_without_setpoint(monkeypatch):
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552", require_armed=False, require_guided=False)
    _open_gates(ctrl, monkeypatch, now=4000.0)
    # Defensive guard: an "allowed" decision that somehow carries no setpoint
    # must be dropped rather than transmitted.
    monkeypatch.setattr(
        mv,
        "evaluate",
        lambda *a, **k: SimpleNamespace(allowed=True, setpoint=None, reason=None, message=None),
    )
    assert ctrl.submit(1.0, 0.0, 0.0, 0.0) is False


# -- background threads (driven through one deterministic iteration) --------


def test_watchdog_zeros_velocity_on_stale_command(monkeypatch):
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552")
    stops: list[bool] = []

    def fake_stop():
        stops.append(True)
        ctrl._stop_event.set()  # end the loop after this single iteration
        return True

    monkeypatch.setattr(ctrl, "_send_stop", fake_stop)
    ctrl._active = True
    ctrl._last_command_time = 0.0  # far in the past -> command timeout exceeded
    ctrl._watchdog_loop()
    assert stops == [True]
    assert ctrl._active is False


def test_rx_loop_tracks_armed_and_mode(monkeypatch):
    ctrl = MavlinkVelocityController(endpoint="127.0.0.1:14552")

    class _Heartbeat:
        # MAV_TYPE for a vehicle (non-GCS) so it drives the armed/mode gate.
        type = mv.mavutil.mavlink.MAV_TYPE_QUADROTOR
        base_mode = 128  # armed bit set

        def get_srcSystem(self):
            return 1

    class _Conn:
        target_system = 1
        target_component = 1

        def __init__(self, controller):
            self._controller = controller
            self._served = False

        def recv_match(self, **kwargs):
            if self._served:
                self._controller._stop_event.set()
                return None
            self._served = True
            return _Heartbeat()

    ctrl._conn = _Conn(ctrl)
    monkeypatch.setattr(mv.mavutil, "mode_string_v10", lambda msg: "GUIDED")
    ctrl._rx_loop()
    assert ctrl._armed is True
    assert ctrl._flight_mode == "GUIDED"
    assert ctrl._last_heartbeat > 0
