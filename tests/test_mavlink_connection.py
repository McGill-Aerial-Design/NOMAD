# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for edge_core.services.mavlink.connection.MavlinkConnection.

The receiver loop talks to a live flight controller over pymavlink, so it is
exercised here with ``mavutil.mavlink_connection`` patched to return a scripted
fake link. ``_run`` is driven directly (no thread) and exits when the scripted
messages are exhausted: the fake's ``recv_match`` trips the stop event on its
trailing ``None``, which also walks the disconnect-status path.
"""

from __future__ import annotations

import math
import threading
from types import SimpleNamespace

import pytest

from edge_core.services.mavlink import connection as conn_mod
from edge_core.services.mavlink.connection import MavlinkConnection


class FakeMsg:
    """A pymavlink-style message: ``get_type()`` + attribute fields."""

    def __init__(self, msg_type: str, **fields):
        self._type = msg_type
        for key, value in fields.items():
            setattr(self, key, value)

    def get_type(self) -> str:
        return self._type


class FakeStateManager:
    def __init__(self, connected: bool = False):
        self.force_calls: list[dict] = []
        self.update_calls: list[dict] = []
        self._state = SimpleNamespace(
            connected=connected,
            cpu_temp_c=55.0,
            gpu_load_pct=12.0,
            disk_free_gb=120.0,
        )

    def force_state_update(self, **kw):
        self.force_calls.append(kw)
        if "connected" in kw:
            self._state.connected = kw["connected"]

    def update_state(self, **kw):
        self.update_calls.append(kw)

    def get_state(self):
        return self._state


class FakeConn:
    target_system = 1
    target_component = 1

    def __init__(self, messages=None, on_exhaust=None):
        self._messages = list(messages or [])
        self._on_exhaust = on_exhaust
        self.closed = False
        self.sent_commands: list[tuple] = []
        self.mav = SimpleNamespace(command_long_send=self._record_cmd)

    def recv_match(self, type=None, blocking=False, timeout=None):  # noqa: A002 - pymavlink kwarg name
        if self._messages:
            return self._messages.pop(0)
        if self._on_exhaust is not None:
            self._on_exhaust()
        return None

    def _record_cmd(self, *args):
        self.sent_commands.append(args)

    def close(self):
        self.closed = True


def _make(monkeypatch, conn: FakeConn, **env) -> MavlinkConnection:
    """A MavlinkConnection wired to ``conn`` and a recording state manager."""
    for key in ("NOMAD_MAVLINK_ENDPOINT", "NOMAD_MAVLINK_DISCONNECT_TIMEOUT_S"):
        monkeypatch.delenv(key, raising=False)
    for key, value in env.items():
        monkeypatch.setenv(key, value)
    monkeypatch.setattr(conn_mod.mavutil, "mavlink_connection", lambda *a, **k: conn)
    mc = MavlinkConnection(FakeStateManager())
    return mc


# --------------------------------------------------------------------------- #
# endpoint normalization (ctor)
# --------------------------------------------------------------------------- #


def test_bare_host_port_becomes_outbound_udp(monkeypatch):
    monkeypatch.delenv("NOMAD_MAVLINK_ENDPOINT", raising=False)
    assert MavlinkConnection(FakeStateManager(), "127.0.0.1:14550").endpoint == "udp:127.0.0.1:14550"


def test_explicit_schemes_pass_through(monkeypatch):
    monkeypatch.delenv("NOMAD_MAVLINK_ENDPOINT", raising=False)
    for ep in ("tcp:1.2.3.4:5760", "udpin:0.0.0.0:14550", "serial:/dev/ttyACM0", "/dev/ttyUSB0"):
        assert MavlinkConnection(FakeStateManager(), ep).endpoint == ep


def test_endpoint_defaults_from_env(monkeypatch):
    monkeypatch.setenv("NOMAD_MAVLINK_ENDPOINT", "udpout:10.0.0.1:14551")
    assert MavlinkConnection(FakeStateManager()).endpoint == "udpout:10.0.0.1:14551"


# --------------------------------------------------------------------------- #
# servo output cache
# --------------------------------------------------------------------------- #


def test_get_servo_output_pwm_none_before_any_message(monkeypatch):
    mc = _make(monkeypatch, FakeConn())
    assert mc.get_servo_output_pwm(1) is None


# --------------------------------------------------------------------------- #
# _connect
# --------------------------------------------------------------------------- #


def test_connect_swallows_exception(monkeypatch):
    monkeypatch.delenv("NOMAD_MAVLINK_ENDPOINT", raising=False)

    def boom(*a, **k):
        raise OSError("no link")

    monkeypatch.setattr(conn_mod.mavutil, "mavlink_connection", boom)
    mc = MavlinkConnection(FakeStateManager())
    mc._connect()
    assert mc._conn is None


def test_run_reconnects_when_first_connect_yields_none(monkeypatch):
    monkeypatch.delenv("NOMAD_MAVLINK_ENDPOINT", raising=False)
    good = FakeConn([FakeMsg("SYS_STATUS", voltage_battery=12000)])
    good._on_exhaust = None
    results = [None, good]
    monkeypatch.setattr(conn_mod.mavutil, "mavlink_connection", lambda *a, **k: results.pop(0))
    monkeypatch.setattr(conn_mod.time, "sleep", lambda *_: None)
    mc = MavlinkConnection(FakeStateManager())
    good._on_exhaust = mc._stop_event.set  # stop once the good link drains
    mc._run()
    assert {"battery_voltage": 12.0} in mc.state_manager.update_calls


# --------------------------------------------------------------------------- #
# _run message dispatch
# --------------------------------------------------------------------------- #


def test_run_dispatches_every_message_type(monkeypatch):
    monkeypatch.setattr(conn_mod.mavutil, "mode_string_v10", lambda m: "GUIDED")
    time_sync = SimpleNamespace(calls=[], update_gps_time=lambda u, b: time_sync.calls.append((u, b)))

    messages = [
        FakeMsg("HEARTBEAT", base_mode=128, custom_mode=0),
        FakeMsg("SYS_STATUS", voltage_battery=12600),
        FakeMsg("GLOBAL_POSITION_INT", lat=455000000, lon=-736000000, alt=100000, relative_alt=50000),
        FakeMsg("HOME_POSITION", latitude=455000000, longitude=-736000000, altitude=100000),
        FakeMsg("ATTITUDE", roll=0.0, pitch=0.0, yaw=math.pi / 2),
        FakeMsg("SYSTEM_TIME", time_unix_usec=1_700_000_000_000_000, time_boot_ms=1234),
        FakeMsg("COMMAND_ACK", command=400, result=0),
        FakeMsg("SERVO_OUTPUT_RAW", **{f"servo{c}_raw": (1500 if c == 1 else 0) for c in range(1, 17)}),
    ]
    conn = FakeConn(messages)
    mc = _make(monkeypatch, conn)
    mc.set_time_sync_service(time_sync)
    conn._on_exhaust = mc._stop_event.set
    mc._run()

    sm = mc.state_manager
    assert {"flight_mode": "GUIDED", "connected": True, "armed": True} in sm.force_calls
    assert {"battery_voltage": 12.6} in sm.update_calls
    assert {
        "gps_fix": True,
        "gps_lat": pytest.approx(45.5),
        "gps_lon": pytest.approx(-73.6),
        "gps_alt": pytest.approx(100.0),
        "alt_agl_m": pytest.approx(50.0),
    } in sm.update_calls
    assert {
        "home_lat": pytest.approx(45.5),
        "home_lon": pytest.approx(-73.6),
        "home_alt": pytest.approx(100.0),
    } in sm.update_calls
    heading_calls = [c for c in sm.update_calls if "heading_deg" in c]
    assert heading_calls and heading_calls[0]["heading_deg"] == pytest.approx(90.0)
    assert time_sync.calls == [(1_700_000_000_000_000, 1234)]
    assert mc.get_servo_output_pwm(1) == 1500
    assert mc.get_servo_output_pwm(2) is None


def test_run_global_position_without_fix_reports_none(monkeypatch):
    conn = FakeConn([FakeMsg("GLOBAL_POSITION_INT", lat=0, lon=0, alt=0, relative_alt=0)])
    mc = _make(monkeypatch, conn)
    conn._on_exhaust = mc._stop_event.set
    mc._run()
    gps = [c for c in mc.state_manager.update_calls if "gps_fix" in c][0]
    assert gps == {"gps_fix": False, "gps_lat": None, "gps_lon": None, "gps_alt": None, "alt_agl_m": None}


def test_run_system_time_ignored_without_sync_service(monkeypatch):
    conn = FakeConn([FakeMsg("SYSTEM_TIME", time_unix_usec=1, time_boot_ms=2)])
    mc = _make(monkeypatch, conn)
    conn._on_exhaust = mc._stop_event.set
    mc._run()  # must not raise even though no time-sync service is attached
    assert mc._time_sync_service is None


# --------------------------------------------------------------------------- #
# disconnect watchdog
# --------------------------------------------------------------------------- #


def test_update_connection_status_marks_lost_after_timeout(monkeypatch):
    mc = _make(monkeypatch, FakeConn(), NOMAD_MAVLINK_DISCONNECT_TIMEOUT_S="3.0")
    mc.state_manager._state.connected = True
    mc._last_heartbeat = 100.0
    mc._update_connection_status(now=200.0)
    assert {"connected": False, "flight_mode": "LOST"} in mc.state_manager.force_calls


def test_update_connection_status_noop_when_recent_or_already_lost(monkeypatch):
    mc = _make(monkeypatch, FakeConn())
    # No heartbeat ever -> nothing to time out.
    mc._update_connection_status(now=1000.0)
    # Fresh heartbeat -> still connected.
    mc._last_heartbeat = 999.9
    mc.state_manager._state.connected = True
    mc._update_connection_status(now=1000.0)
    assert mc.state_manager.force_calls == []


# --------------------------------------------------------------------------- #
# _resolve_mode
# --------------------------------------------------------------------------- #


def test_resolve_mode_uses_pymavlink(monkeypatch):
    monkeypatch.setattr(conn_mod.mavutil, "mode_string_v10", lambda m: "AUTO")
    assert MavlinkConnection._resolve_mode(FakeMsg("HEARTBEAT")) == "AUTO"


def test_resolve_mode_falls_back_to_unknown(monkeypatch):
    monkeypatch.setattr(conn_mod.mavutil, "mode_string_v10", lambda m: "")
    assert MavlinkConnection._resolve_mode(FakeMsg("HEARTBEAT")) == "UNKNOWN"

    def boom(m):
        raise ValueError("bad mode")

    monkeypatch.setattr(conn_mod.mavutil, "mode_string_v10", boom)
    assert MavlinkConnection._resolve_mode(FakeMsg("HEARTBEAT")) == "UNKNOWN"


# --------------------------------------------------------------------------- #
# command-ack tracking
# --------------------------------------------------------------------------- #


def test_record_command_ack_keeps_last_25(monkeypatch):
    mc = _make(monkeypatch, FakeConn())
    for i in range(30):
        mc._record_command_ack(FakeMsg("COMMAND_ACK", command=i, result=0))
    assert len(mc._command_acks) == 25
    assert mc._command_acks[0]["command"] == 5  # first five dropped


def test_wait_command_ack_accepts_and_rejects(monkeypatch):
    mc = _make(monkeypatch, FakeConn())
    mc._record_command_ack(FakeMsg("COMMAND_ACK", command=400, result=0))  # ACCEPTED
    assert mc._wait_command_ack(400, since=0.0, timeout_s=0.05) is True

    mc._record_command_ack(FakeMsg("COMMAND_ACK", command=401, result=4))  # FAILED
    assert mc._wait_command_ack(401, since=0.0, timeout_s=0.05) is False


def test_wait_command_ack_times_out_when_no_match(monkeypatch):
    mc = _make(monkeypatch, FakeConn())
    # An ack exists but predates `since`, so it is skipped and the wait expires.
    mc._record_command_ack(FakeMsg("COMMAND_ACK", command=400, result=0))
    assert mc._wait_command_ack(400, since=1e18, timeout_s=0.05) is False


def test_send_command_long_returns_false_without_connection(monkeypatch):
    mc = _make(monkeypatch, FakeConn())
    mc._conn = None
    assert mc._send_command_long_and_wait_ack(400, 1.0) is False


def test_send_command_long_sends_and_confirms(monkeypatch):
    conn = FakeConn()
    mc = _make(monkeypatch, conn)
    mc._conn = conn
    # The fake FC acks the command synchronously as it is sent.
    conn.mav.command_long_send = lambda *a: mc._record_command_ack(FakeMsg("COMMAND_ACK", command=a[2], result=0))
    assert mc._send_command_long_and_wait_ack(400, 1.0, 2.0, timeout_s=0.2) is True


# --------------------------------------------------------------------------- #
# lifecycle + health broadcast
# --------------------------------------------------------------------------- #


def test_start_runs_loop_and_stop_closes_connection(monkeypatch):
    conn = FakeConn()
    mc = _make(monkeypatch, conn)
    conn._on_exhaust = mc._stop_event.set
    mc.start()
    mc._thread.join(timeout=2.0)
    mc.stop()
    assert conn.closed is True


def test_health_broadcast_emits_statustext(monkeypatch):
    mc = _make(monkeypatch, FakeConn())
    sent: list[str] = []
    got = threading.Event()
    mc.send_statustext = lambda s: (sent.append(s), got.set())  # type: ignore[attr-defined]
    mc.start_health_broadcast(interval=0.01)
    assert mc.start_health_broadcast(interval=0.01) is None  # idempotent second start
    assert got.wait(2.0)
    mc.stop_health_broadcast()
    assert sent and sent[0].startswith("NOMAD: CPU")


def test_start_is_idempotent_when_thread_already_alive(monkeypatch):
    mc = _make(monkeypatch, FakeConn())
    sentinel = SimpleNamespace(is_alive=lambda: True)
    mc._thread = sentinel
    mc.start()  # guard returns early; no new thread is spawned
    assert mc._thread is sentinel


def test_stop_swallows_close_error(monkeypatch):
    conn = FakeConn()

    def boom():
        raise OSError("close failed")

    conn.close = boom
    mc = _make(monkeypatch, conn)
    mc._conn = conn
    mc.stop()  # must not propagate the close() error


def test_run_swallows_recv_exception(monkeypatch):
    conn = FakeConn()
    mc = _make(monkeypatch, conn)
    calls = {"n": 0}

    def recv(**kwargs):
        calls["n"] += 1
        if calls["n"] == 1:
            raise OSError("recv blew up")
        mc._stop_event.set()
        return None

    conn.recv_match = recv
    mc._run()  # first recv raises -> treated as no message, loop continues then stops
    assert calls["n"] == 2


def test_health_broadcast_loop_survives_state_errors(monkeypatch):
    mc = _make(monkeypatch, FakeConn())
    mc._health_stop_event = threading.Event()
    calls = {"n": 0}

    def flaky_get_state():
        calls["n"] += 1
        mc._health_stop_event.set()  # one iteration, then stop
        raise RuntimeError("state unavailable")

    mc.state_manager.get_state = flaky_get_state
    mc._health_interval = 0.0
    mc._broadcast_health_loop()  # exception is caught; loop exits cleanly
    assert calls["n"] == 1
