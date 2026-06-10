# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Geofence enforcement tests for the MAVLink command adapter (SR-FEN-02).

Proves that ``MavlinkCommands.send_global_position_target`` /
``send_position_target`` refuse to transmit a position target outside the
configured ``NOMAD_FENCE_POLYGON``, and that every failure input (malformed
config, missing home position, non-finite target) fails closed: rejected,
nothing sent.
"""

from __future__ import annotations

import math

from edge_core.services.mavlink.commands import MavlinkCommands

# A roughly 222 m (N-S) x 156 m (E-W) box centred on (45.5, -73.6).
FENCE = "45.501,-73.601;45.501,-73.599;45.499,-73.599;45.499,-73.601"
CENTER_LAT, CENTER_LON = 45.5, -73.6


class _RecordingMav:
    def __init__(self):
        self.calls: list[tuple[str, tuple]] = []

    def set_position_target_global_int_send(self, *args):
        self.calls.append(("global", args))

    def set_position_target_local_ned_send(self, *args):
        self.calls.append(("local", args))


class _FakeConn:
    target_system = 1
    target_component = 1

    def __init__(self):
        self.mav = _RecordingMav()


class _FakeState:
    def __init__(self, home_lat=None, home_lon=None):
        self.home_lat = home_lat
        self.home_lon = home_lon


class _FakeStateManager:
    def __init__(self, state):
        self._state = state

    def get_state(self):
        return self._state


def _commands(monkeypatch, polygon=None, margin=None, home=None) -> MavlinkCommands:
    monkeypatch.delenv("NOMAD_FENCE_POLYGON", raising=False)
    monkeypatch.delenv("NOMAD_FENCE_MARGIN_M", raising=False)
    if polygon is not None:
        monkeypatch.setenv("NOMAD_FENCE_POLYGON", polygon)
    if margin is not None:
        monkeypatch.setenv("NOMAD_FENCE_MARGIN_M", margin)
    cmds = MavlinkCommands()
    cmds._conn = _FakeConn()
    if home is not None:
        cmds.state_manager = _FakeStateManager(_FakeState(*home))
    return cmds


def _sent(cmds: MavlinkCommands) -> list[tuple[str, tuple]]:
    return cmds._conn.mav.calls


def test_no_fence_configured_passes_targets_through(monkeypatch):
    cmds = _commands(monkeypatch, home=(CENTER_LAT, CENTER_LON))
    assert cmds.send_global_position_target(89.0, 179.0, 50.0) is True
    assert cmds.send_position_target(10_000.0, 0.0, -10.0, 0.0) is True
    assert [kind for kind, _ in _sent(cmds)] == ["global", "local"]


def test_global_target_inside_fence_is_sent(monkeypatch):
    cmds = _commands(monkeypatch, polygon=FENCE)
    assert cmds.send_global_position_target(CENTER_LAT, CENTER_LON, 50.0) is True
    assert len(_sent(cmds)) == 1


def test_global_target_outside_fence_is_rejected(monkeypatch):
    cmds = _commands(monkeypatch, polygon=FENCE)
    assert cmds.send_global_position_target(45.502, CENTER_LON, 50.0) is False
    assert _sent(cmds) == []


def test_global_target_inside_keep_in_margin_is_rejected(monkeypatch):
    # (45.5008, -73.6) is ~22 m from the north edge: fine with the default
    # 2 m margin, rejected with a 50 m margin.
    assert _commands(monkeypatch, polygon=FENCE).send_global_position_target(45.5008, CENTER_LON, 50.0) is True
    cmds = _commands(monkeypatch, polygon=FENCE, margin="50.0")
    assert cmds.send_global_position_target(45.5008, CENTER_LON, 50.0) is False
    assert _sent(cmds) == []


def test_global_target_nonfinite_is_rejected(monkeypatch):
    cmds = _commands(monkeypatch, polygon=FENCE)
    assert cmds.send_global_position_target(math.nan, CENTER_LON, 50.0) is False
    assert cmds.send_global_position_target(CENTER_LAT, math.inf, 50.0) is False
    assert _sent(cmds) == []


def test_local_target_inside_fence_is_sent(monkeypatch):
    cmds = _commands(monkeypatch, polygon=FENCE, home=(CENTER_LAT, CENTER_LON))
    assert cmds.send_position_target(20.0, 10.0, -10.0, 0.0) is True
    assert len(_sent(cmds)) == 1


def test_local_target_outside_fence_is_rejected(monkeypatch):
    cmds = _commands(monkeypatch, polygon=FENCE, home=(CENTER_LAT, CENTER_LON))
    assert cmds.send_position_target(500.0, 0.0, -10.0, 0.0) is False
    assert _sent(cmds) == []


def test_local_target_without_home_is_rejected(monkeypatch):
    # Fence configured but home unknown: containment cannot be verified.
    no_state = _commands(monkeypatch, polygon=FENCE)
    assert no_state.send_position_target(0.0, 0.0, -10.0, 0.0) is False
    no_home = _commands(monkeypatch, polygon=FENCE, home=(None, None))
    assert no_home.send_position_target(0.0, 0.0, -10.0, 0.0) is False
    assert _sent(no_state) == [] and _sent(no_home) == []


def test_malformed_polygon_fails_closed(monkeypatch):
    for bad in ("garbage", "45.5,-73.6;45.6,-73.6", "45.5,-73.6;nan,-73.6;45.6,-73.5"):
        cmds = _commands(monkeypatch, polygon=bad, home=(CENTER_LAT, CENTER_LON))
        assert cmds.send_global_position_target(CENTER_LAT, CENTER_LON, 50.0) is False
        assert cmds.send_position_target(0.0, 0.0, -10.0, 0.0) is False
        assert _sent(cmds) == []


def test_malformed_margin_fails_closed(monkeypatch):
    for bad in ("garbage", "-1.0", "inf"):
        cmds = _commands(monkeypatch, polygon=FENCE, margin=bad)
        assert cmds.send_global_position_target(CENTER_LAT, CENTER_LON, 50.0) is False
        assert _sent(cmds) == []


def test_fence_does_not_gate_velocity_commands(monkeypatch):
    # The fence applies to position targets only; the velocity path keeps its
    # own envelope (edge_core.safety.envelope) and the FC fence as backstop.
    cmds = _commands(monkeypatch, polygon=FENCE)
    sent = []
    cmds._conn.mav.set_position_target_local_ned_send = lambda *a: sent.append(a)
    assert cmds.send_velocity_command(1.0, 0.0, 0.0) is True
    assert len(sent) == 1
