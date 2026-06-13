# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for edge_core.services.mavlink.commands.MavlinkCommands.

The geofence gating on the position-target paths is covered in
``test_mavlink_fence``; this pins the rest of the command surface — the
``_send_guarded`` link guard and the individual command builders (arm, payload,
relay, velocity, statustext, mode, takeoff, land, home request, global/local
position targets) — with a fake pymavlink link and the command-ack sender stubbed.
"""

from __future__ import annotations

import pytest
from pymavlink import mavutil

from edge_core.services.mavlink.commands import MavlinkCommands


class FakeMav:
    def __init__(self):
        self.calls: list[tuple] = []

    def set_position_target_local_ned_send(self, *args):
        self.calls.append(("local", args))

    def set_position_target_global_int_send(self, *args):
        self.calls.append(("global", args))

    def statustext_send(self, *args):
        self.calls.append(("statustext", args))


class FakeConn:
    target_system = 1
    target_component = 1

    def __init__(self):
        self.mav = FakeMav()


def _commands(monkeypatch, *, with_conn=True, ack_result=True):
    """A MavlinkCommands with no NOMAD fence, a fake link, and a recording ack sender."""
    monkeypatch.delenv("NOMAD_FENCE_POLYGON", raising=False)
    monkeypatch.delenv("NOMAD_FENCE_MARGIN_M", raising=False)
    c = MavlinkCommands()
    if with_conn:
        c._conn = FakeConn()
    sent: list[tuple] = []

    def fake_ack(command_id, *params, timeout_s=0.75):
        sent.append((command_id, params, timeout_s))
        return ack_result

    c._send_command_long_and_wait_ack = fake_ack  # type: ignore[method-assign]
    return c, sent


# --------------------------------------------------------------------------- #
# the base mixin contract + _send_guarded
# --------------------------------------------------------------------------- #


def test_base_command_sender_is_abstract():
    with pytest.raises(NotImplementedError):
        MavlinkCommands()._send_command_long_and_wait_ack(0)


def test_send_guarded_without_connection():
    c = MavlinkCommands()  # _conn defaults to None
    assert c._send_guarded("x", lambda: (_ for _ in ()).throw(AssertionError("must not send"))) is False


def test_send_guarded_passes_through_results():
    c = MavlinkCommands()
    c._conn = FakeConn()
    assert c._send_guarded("none-result", lambda: None) is True  # None -> success
    assert c._send_guarded("truthy", lambda: True) is True
    assert c._send_guarded("falsy", lambda: False) is False


def test_send_guarded_swallows_exceptions():
    c = MavlinkCommands()
    c._conn = FakeConn()

    def boom():
        raise RuntimeError("tx failed")

    assert c._send_guarded("boom", boom) is False


# --------------------------------------------------------------------------- #
# command-long builders
# --------------------------------------------------------------------------- #


def test_arm_disarm(monkeypatch):
    c, sent = _commands(monkeypatch)
    assert c.arm_disarm(True) is True
    assert sent[-1][:2] == (mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, (1,))
    c.arm_disarm(False)
    assert sent[-1][:2] == (mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, (0,))


def test_trigger_payload_defaults_channel(monkeypatch):
    c, sent = _commands(monkeypatch)
    assert c.trigger_payload(1500) is True
    assert sent[-1][:2] == (mavutil.mavlink.MAV_CMD_DO_SET_SERVO, (9, 1500))
    c.trigger_payload(1900, servo_channel=12)
    assert sent[-1][:2] == (mavutil.mavlink.MAV_CMD_DO_SET_SERVO, (12, 1900))


def test_set_relay(monkeypatch):
    c, sent = _commands(monkeypatch)
    assert c.set_relay(2, True) is True
    assert sent[-1][:2] == (mavutil.mavlink.MAV_CMD_DO_SET_RELAY, (2, 1))
    c.set_relay(2, False)
    assert sent[-1][:2] == (mavutil.mavlink.MAV_CMD_DO_SET_RELAY, (2, 0))


def test_set_mode_and_land(monkeypatch):
    c, sent = _commands(monkeypatch)
    assert c.set_mode(4) is True
    assert sent[-1][:2] == (mavutil.mavlink.MAV_CMD_DO_SET_MODE, (1, 4))
    # land() is set_mode(9) (ArduCopter LAND).
    assert c.land() is True
    assert sent[-1][:2] == (mavutil.mavlink.MAV_CMD_DO_SET_MODE, (1, 9))


def test_takeoff_passes_altitude_and_timeout(monkeypatch):
    c, sent = _commands(monkeypatch)
    assert c.takeoff(12.5) is True
    command_id, params, timeout_s = sent[-1]
    assert command_id == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    assert params[-1] == 12.5  # altitude is the 7th param
    assert timeout_s == 2.0


def test_request_home_position(monkeypatch):
    c, sent = _commands(monkeypatch)
    assert c.request_home_position() is True
    command_id, params, timeout_s = sent[-1]
    assert command_id == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
    assert params[0] == mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION
    assert timeout_s == 1.0


def test_command_long_returns_false_on_negative_ack(monkeypatch):
    c, _ = _commands(monkeypatch, ack_result=False)
    assert c.arm_disarm(True) is False


def test_command_long_without_connection(monkeypatch):
    c, sent = _commands(monkeypatch, with_conn=False)
    assert c.arm_disarm(True) is False
    assert sent == []  # guarded out before the ack sender


# --------------------------------------------------------------------------- #
# direct-link builders (velocity / statustext)
# --------------------------------------------------------------------------- #


def test_send_velocity_default_frame(monkeypatch):
    c, _ = _commands(monkeypatch)
    assert c.send_velocity_command(1.0, -2.0, 0.5, 0.1) is True
    kind, args = c._conn.mav.calls[-1]
    assert kind == "local"
    assert args[3] == mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED  # frame
    assert args[8:11] == (1.0, -2.0, 0.5)  # vx, vy, vz


def test_send_velocity_custom_frame(monkeypatch):
    c, _ = _commands(monkeypatch)
    c.send_velocity_command(0.0, 0.0, 0.0, coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED)
    assert c._conn.mav.calls[-1][1][3] == mavutil.mavlink.MAV_FRAME_LOCAL_NED


def test_stop_velocity_sends_zeros(monkeypatch):
    c, _ = _commands(monkeypatch)
    assert c.stop_velocity() is True
    args = c._conn.mav.calls[-1][1]
    assert args[8:11] == (0.0, 0.0, 0.0)


def test_send_statustext_truncates_and_defaults_severity(monkeypatch):
    c, _ = _commands(monkeypatch)
    long_text = "N" * 80
    assert c.send_statustext(long_text) is True
    kind, args = c._conn.mav.calls[-1]
    assert kind == "statustext"
    assert args[0] == mavutil.mavlink.MAV_SEVERITY_INFO
    assert args[1] == long_text[:50].encode("utf-8")


def test_send_statustext_custom_severity(monkeypatch):
    c, _ = _commands(monkeypatch)
    c.send_statustext("warn", severity=mavutil.mavlink.MAV_SEVERITY_WARNING)
    assert c._conn.mav.calls[-1][1][0] == mavutil.mavlink.MAV_SEVERITY_WARNING


# --------------------------------------------------------------------------- #
# position targets (link guard + yaw mask; fence covered in test_mavlink_fence)
# --------------------------------------------------------------------------- #


def test_global_position_target_without_connection(monkeypatch):
    c, _ = _commands(monkeypatch, with_conn=False)
    assert c.send_global_position_target(45.5, -73.6, 50.0) is False


def test_global_position_target_scales_and_sets_yaw_mask(monkeypatch):
    c, _ = _commands(monkeypatch)
    assert c.send_global_position_target(45.5, -73.6, 50.0, yaw=1.2) is True
    kind, args = c._conn.mav.calls[-1]
    assert kind == "global"
    # type_mask with the yaw bit (10) cleared.
    assert args[4] == (0b0000_1111_1111_1000 & ~(1 << 10))
    assert args[5] == round(45.5 * 1e7)  # lat scaled to 1e7
    assert args[6] == round(-73.6 * 1e7)


def test_global_position_target_default_yaw_mask(monkeypatch):
    c, _ = _commands(monkeypatch)
    c.send_global_position_target(45.5, -73.6, 50.0)
    assert c._conn.mav.calls[-1][1][4] == 0b0000_1111_1111_1000  # yaw bit not cleared


def test_local_position_target_without_connection(monkeypatch):
    c, _ = _commands(monkeypatch, with_conn=False)
    assert c.send_position_target(10.0, 5.0, -3.0, 0.0) is False


def test_local_position_target_sends_when_no_fence(monkeypatch):
    c, _ = _commands(monkeypatch)
    assert c.send_position_target(10.0, 5.0, -3.0, 0.5) is True
    kind, args = c._conn.mav.calls[-1]
    assert kind == "local"
    assert args[5:8] == (10.0, 5.0, -3.0)  # x, y, z
