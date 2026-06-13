# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""TimeSyncService: NTP/GPS sync-status logic with subprocess, socket, and the
system clock all mocked. No network, no real `timedatectl`, no real threads
(the monitor loop is driven synchronously)."""

from __future__ import annotations

import builtins
import socket
import subprocess
import time
from datetime import datetime, timezone
from io import StringIO

import pytest

from edge_core.services.time_manager import (
    TimeSyncModule,
    TimeSyncService,
    TimeSyncSource,
    TimeSyncStatus,
    get_time_sync_service,
    init_time_sync_service,
)


def _svc() -> TimeSyncService:
    return TimeSyncService()


# ---- TimeSyncStatus.to_dict -------------------------------------------------


def test_status_to_dict_with_last_sync():
    now = datetime.now(timezone.utc)
    st = TimeSyncStatus(
        synced=True,
        source=TimeSyncSource.NTP,
        offset_seconds=0.5,
        last_sync=now,
        gps_time_available=True,
        ntp_reachable=True,
    )
    d = st.to_dict()
    assert d["source"] == "NTP"
    assert d["last_sync"] == now.isoformat()
    assert d["synced"] is True
    assert d["offset_seconds"] == 0.5


def test_status_to_dict_without_last_sync():
    st = TimeSyncStatus(False, TimeSyncSource.NONE, 0.0, None, False, False)
    assert st.to_dict()["last_sync"] is None


# ---- _detect_jetson ---------------------------------------------------------


def test_detect_jetson_true(monkeypatch):
    monkeypatch.setattr(builtins, "open", lambda *a, **k: StringIO("NVIDIA Tegra Release"))
    assert TimeSyncService._detect_jetson() is True


def test_detect_jetson_false(monkeypatch):
    def _missing(*a, **k):
        raise FileNotFoundError

    monkeypatch.setattr(builtins, "open", _missing)
    assert TimeSyncService._detect_jetson() is False


# ---- properties -------------------------------------------------------------


def test_initial_properties():
    svc = _svc()
    assert svc.is_synced is False
    assert svc.sync_source is TimeSyncSource.NONE
    assert isinstance(svc.status, TimeSyncStatus)


# ---- check_ntp_status / _check_ntp_reachable --------------------------------


def test_check_ntp_status_timedatectl_synced(monkeypatch):
    svc = _svc()
    svc._is_linux = True
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *a, **k: subprocess.CompletedProcess(a, 0, "System clock synchronized: yes", ""),
    )
    assert svc.check_ntp_status() is True


def test_check_ntp_status_falls_back_to_reachable(monkeypatch):
    svc = _svc()
    svc._is_linux = True
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *a, **k: subprocess.CompletedProcess(a, 0, "System clock synchronized: no", ""),
    )
    monkeypatch.setattr(svc, "_check_ntp_reachable", lambda: True)
    assert svc.check_ntp_status() is True


def test_check_ntp_status_timedatectl_missing(monkeypatch):
    svc = _svc()
    svc._is_linux = True

    def _boom(*a, **k):
        raise FileNotFoundError("no timedatectl")

    monkeypatch.setattr(subprocess, "run", _boom)
    monkeypatch.setattr(svc, "_check_ntp_reachable", lambda: False)
    assert svc.check_ntp_status() is False


def test_check_ntp_status_non_linux(monkeypatch):
    svc = _svc()
    svc._is_linux = False
    monkeypatch.setattr(svc, "_check_ntp_reachable", lambda: True)
    assert svc.check_ntp_status() is True


class _FakeSock:
    def __init__(self, recv_len=48, raise_on=None):
        self._recv_len = recv_len
        self._raise_on = raise_on

    def settimeout(self, _t):
        pass

    def sendto(self, *_a):
        if self._raise_on == "send":
            raise OSError("network down")

    def recvfrom(self, _n):
        if self._raise_on == "recv":
            raise TimeoutError()
        return (b"\x00" * self._recv_len, None)

    def close(self):
        pass


def test_check_ntp_reachable_true(monkeypatch):
    monkeypatch.setattr(socket, "socket", lambda *a, **k: _FakeSock(48))
    assert _svc()._check_ntp_reachable() is True


def test_check_ntp_reachable_short_response(monkeypatch):
    monkeypatch.setattr(socket, "socket", lambda *a, **k: _FakeSock(10))
    assert _svc()._check_ntp_reachable() is False


def test_check_ntp_reachable_error(monkeypatch):
    monkeypatch.setattr(socket, "socket", lambda *a, **k: _FakeSock(raise_on="recv"))
    assert _svc()._check_ntp_reachable() is False


# ---- update_gps_time / _check_time_offset -----------------------------------


def test_update_gps_time_marks_available_with_zero_offset(monkeypatch):
    svc = _svc()
    monkeypatch.setattr(time, "time", lambda: 1000.0)
    svc.update_gps_time(unix_time_us=1_000_000_000, boot_time_ms=5000)  # gps == 1000.0 s
    assert svc.status.gps_time_available is True
    assert svc.status.offset_seconds == pytest.approx(0.0, abs=1e-6)


def test_update_gps_time_zero_is_ignored():
    svc = _svc()
    svc.update_gps_time(0, 0)
    assert svc.status.gps_time_available is False


def test_update_gps_time_large_offset(monkeypatch):
    svc = _svc()
    monkeypatch.setattr(time, "time", lambda: 2000.0)
    svc.update_gps_time(1_000_000_000, 0)  # gps 1000 s vs system 2000 s
    assert svc.status.offset_seconds == pytest.approx(1000.0)


def test_check_time_offset_noop_without_gps():
    svc = _svc()
    svc._gps_time_us = 0
    svc._check_time_offset()  # early return, status untouched
    assert svc.status.offset_seconds == 0.0


# ---- force_sync_from_gps ----------------------------------------------------


def test_force_sync_without_gps_fails():
    assert _svc().force_sync_from_gps() is False


def test_force_sync_non_linux_fails():
    svc = _svc()
    svc._is_linux = False
    svc._gps_time_us = 1_000_000_000
    assert svc.force_sync_from_gps() is False


def test_force_sync_timedatectl_success(monkeypatch):
    svc = _svc()
    svc._is_linux = True
    svc._gps_time_us = 1_000_000_000
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: subprocess.CompletedProcess(a, 0, "", ""))
    assert svc.force_sync_from_gps() is True
    assert svc.is_synced is True
    assert svc.sync_source is TimeSyncSource.GPS


def test_force_sync_date_fallback(monkeypatch):
    svc = _svc()
    svc._is_linux = True
    svc._gps_time_us = 1_000_000_000
    cmds: list[str] = []

    def _run(cmd, *a, **k):
        cmds.append(cmd[0])
        rc = 1 if cmd[0] == "timedatectl" else 0
        return subprocess.CompletedProcess(cmd, rc, "", "err")

    monkeypatch.setattr(subprocess, "run", _run)
    assert svc.force_sync_from_gps() is True
    assert "date" in cmds


def test_force_sync_both_fail(monkeypatch):
    svc = _svc()
    svc._is_linux = True
    svc._gps_time_us = 1_000_000_000
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: subprocess.CompletedProcess(a, 1, "", "err"))
    assert svc.force_sync_from_gps() is False


@pytest.mark.parametrize(
    "exc",
    [
        subprocess.TimeoutExpired(cmd="timedatectl", timeout=10),
        PermissionError(),
        RuntimeError("unexpected"),
    ],
)
def test_force_sync_handles_exceptions(monkeypatch, exc):
    svc = _svc()
    svc._is_linux = True
    svc._gps_time_us = 1_000_000_000

    def _boom(*a, **k):
        raise exc

    monkeypatch.setattr(subprocess, "run", _boom)
    assert svc.force_sync_from_gps() is False


# ---- _update_sync_status / _check_sync_status -------------------------------


def test_update_sync_status_notifies_state_and_callback():
    events: list = []

    class _FakeSM:
        def update_state(self, **kw):
            events.append(kw)

    svc = TimeSyncService(state_manager=_FakeSM(), on_sync_change=lambda s: events.append(("cb", s.source)))
    svc._update_sync_status(TimeSyncSource.NTP)
    assert svc.is_synced is True
    assert {"time_synced": True} in events
    assert ("cb", TimeSyncSource.NTP) in events


def test_check_sync_status_ntp(monkeypatch):
    events: list = []
    svc = TimeSyncService(on_sync_change=lambda s: events.append(s.source))
    monkeypatch.setattr(svc, "check_ntp_status", lambda: True)
    svc._check_sync_status()
    assert svc.is_synced is True
    assert svc.sync_source is TimeSyncSource.NTP
    assert events[-1] is TimeSyncSource.NTP  # transition logged + callback fired


def test_check_sync_status_updates_state_manager(monkeypatch):
    updates: list = []

    class _SM:
        def update_state(self, **kw):
            updates.append(kw)

    svc = TimeSyncService(state_manager=_SM())
    monkeypatch.setattr(svc, "check_ntp_status", lambda: True)
    svc._check_sync_status()
    assert {"time_synced": True} in updates


def test_check_sync_status_gps_fallback(monkeypatch):
    svc = _svc()
    monkeypatch.setattr(svc, "check_ntp_status", lambda: False)
    monkeypatch.setattr(svc, "_check_ntp_reachable", lambda: False)
    svc._status = TimeSyncStatus(False, TimeSyncSource.NONE, 0.1, None, True, False)
    svc._check_sync_status()
    assert svc.is_synced is True
    assert svc.sync_source is TimeSyncSource.GPS


def test_check_sync_status_none(monkeypatch):
    svc = _svc()
    monkeypatch.setattr(svc, "check_ntp_status", lambda: False)
    monkeypatch.setattr(svc, "_check_ntp_reachable", lambda: False)
    svc._check_sync_status()
    assert svc.is_synced is False
    assert svc.sync_source is TimeSyncSource.NONE


def test_check_sync_status_logs_loss(monkeypatch):
    events: list = []
    svc = TimeSyncService(on_sync_change=lambda s: events.append(s.synced))
    svc._status = TimeSyncStatus(True, TimeSyncSource.NTP, 0.0, datetime.now(timezone.utc), False, True)
    monkeypatch.setattr(svc, "check_ntp_status", lambda: False)
    monkeypatch.setattr(svc, "_check_ntp_reachable", lambda: False)
    svc._check_sync_status()
    assert svc.is_synced is False
    assert events[-1] is False  # loss transition fired the callback


# ---- get_corrected_timestamp ------------------------------------------------


def test_get_corrected_timestamp_without_gps_returns_now():
    ts = _svc().get_corrected_timestamp()
    assert ts.tzinfo is not None


def test_get_corrected_timestamp_interpolates(monkeypatch):
    svc = _svc()
    svc._status = TimeSyncStatus(True, TimeSyncSource.GPS, 0.5, None, True, False)
    svc._gps_time_us = 1_000_000_000  # 1000.0 s
    svc._last_gps_time_update = 900.0
    monkeypatch.setattr(time, "time", lambda: 905.0)  # 5 s elapsed since GPS update
    ts = svc.get_corrected_timestamp()
    assert ts == datetime.fromtimestamp(1005.0, tz=timezone.utc)


# ---- start / stop / monitor loop -------------------------------------------


def test_start_is_idempotent_and_stop_clears_running(monkeypatch):
    svc = _svc()
    monkeypatch.setattr(svc, "_check_sync_status", lambda: None)
    monkeypatch.setattr(svc, "_monitor_loop", lambda: None)  # thread exits instantly
    svc.start()
    assert svc._running is True
    svc.start()  # early return on the second call
    svc.stop()
    assert svc._running is False


def test_monitor_loop_one_iteration(monkeypatch):
    svc = _svc()
    svc._gps_time_us = 5  # > 0 so the GPS-offset branch runs
    calls = {"sync": 0, "offset": 0}
    monkeypatch.setattr(svc, "_check_sync_status", lambda: calls.__setitem__("sync", calls["sync"] + 1))
    monkeypatch.setattr(svc, "_check_time_offset", lambda: calls.__setitem__("offset", calls["offset"] + 1))

    def _stop_after_first(_secs):
        svc._running = False

    monkeypatch.setattr(time, "sleep", _stop_after_first)
    svc._running = True
    svc._monitor_loop()
    assert calls == {"sync": 1, "offset": 1}


def test_monitor_loop_handles_exception(monkeypatch):
    svc = _svc()

    def _boom():
        raise RuntimeError("transient")

    sleeps: list[float] = []

    def _record_sleep(secs):
        sleeps.append(secs)
        svc._running = False

    monkeypatch.setattr(svc, "_check_sync_status", _boom)
    monkeypatch.setattr(time, "sleep", _record_sleep)
    svc._running = True
    svc._monitor_loop()
    assert sleeps == [5.0]  # error path backs off for 5 s


# ---- module + global accessors ---------------------------------------------


def test_global_service_singleton_and_init():
    from edge_core.services.state import StateManager

    s1 = get_time_sync_service()
    assert get_time_sync_service() is s1
    s3 = init_time_sync_service(StateManager.instance())
    assert s3 is not s1
    assert get_time_sync_service() is s3


def test_time_sync_module_lifecycle(monkeypatch):
    from edge_core.services.state import StateManager

    class _Ctx:
        def __init__(self):
            self.services: dict = {}

        def require_service(self, _name):
            return StateManager.instance()

        def register_service(self, name, svc):
            self.services[name] = svc

    ctx = _Ctx()
    mod = TimeSyncModule()
    mod.configure(ctx)
    assert "time_sync" in ctx.services
    monkeypatch.setattr(ctx.services["time_sync"], "_check_sync_status", lambda: None)
    monkeypatch.setattr(ctx.services["time_sync"], "_monitor_loop", lambda: None)
    mod.start()
    mod.stop()
