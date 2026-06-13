# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for infra.tailscale.tailscale_manager.

``tailscale status``/``up`` go through the module-level ``_run`` wrapper, which
is patched to a command-dispatching stub. The status JSON parser is pure and is
pinned directly; the monitor loop is driven one iteration at a time.
"""

from __future__ import annotations

import json
from types import SimpleNamespace

from infra.tailscale import tailscale_manager as tsm
from infra.tailscale.tailscale_manager import (
    TailscaleInfo,
    TailscaleManager,
    TailscaleStatus,
    parse_status_json,
)


def make_run(rules, default=(1, "")):
    def run(cmd, timeout=10.0):
        joined = " ".join(cmd)
        for needle, out in rules:
            if needle in joined:
                return out
        return default

    return run


# --------------------------------------------------------------------------- #
# parse_status_json
# --------------------------------------------------------------------------- #


def test_parse_status_running_with_ipv4():
    data = {
        "BackendState": "Running",
        "Self": {"TailscaleIPs": ["fd7a::1", "100.64.0.5"], "HostName": "nomad-drone"},
        "Peer": {"a": {}, "b": {}},
    }
    info = parse_status_json(data)
    assert info.status is TailscaleStatus.CONNECTED
    assert info.ip_address == "100.64.0.5"  # IPv6 skipped
    assert info.hostname == "nomad-drone"
    assert info.peer_count == 2


def test_parse_status_state_map():
    assert parse_status_json({"BackendState": "Starting"}).status is TailscaleStatus.CONNECTING
    assert parse_status_json({"BackendState": "NeedsLogin"}).status is TailscaleStatus.NEEDS_AUTH
    assert parse_status_json({"BackendState": "NeedsMachineAuth"}).status is TailscaleStatus.NEEDS_AUTH
    assert parse_status_json({"BackendState": "Stopped"}).status is TailscaleStatus.DISCONNECTED
    assert parse_status_json({"BackendState": "Whatever"}).status is TailscaleStatus.ERROR


def test_parse_status_defaults_when_self_missing():
    info = parse_status_json({"BackendState": "Running"})
    assert info.ip_address is None
    assert info.hostname == "unknown"
    assert info.peer_count == 0


# --------------------------------------------------------------------------- #
# _run
# --------------------------------------------------------------------------- #


def test_run_ok(monkeypatch):
    monkeypatch.setattr(tsm.subprocess, "run", lambda *a, **k: SimpleNamespace(returncode=0, stdout="out"))
    assert tsm._run(["tailscale", "status"]) == (0, "out")


def test_run_not_installed(monkeypatch):
    monkeypatch.setattr(tsm.subprocess, "run", lambda *a, **k: (_ for _ in ()).throw(FileNotFoundError()))
    assert tsm._run(["tailscale"]) == (127, "")


def test_run_generic_error(monkeypatch):
    monkeypatch.setattr(tsm.subprocess, "run", lambda *a, **k: (_ for _ in ()).throw(OSError("boom")))
    assert tsm._run(["tailscale"]) == (1, "")


# --------------------------------------------------------------------------- #
# _check_status
# --------------------------------------------------------------------------- #


def test_check_status_not_installed(monkeypatch):
    m = TailscaleManager()
    monkeypatch.setattr(tsm, "_run", make_run([("status", (127, ""))]))
    m._check_status()
    assert m.info.status is TailscaleStatus.NOT_INSTALLED


def test_check_status_command_error(monkeypatch):
    m = TailscaleManager()
    monkeypatch.setattr(tsm, "_run", make_run([("status", (1, ""))]))
    m._check_status()
    assert m.info.status is TailscaleStatus.ERROR


def test_check_status_parses_json(monkeypatch):
    payload = json.dumps({"BackendState": "Running", "Self": {"TailscaleIPs": ["100.64.0.9"]}})
    m = TailscaleManager()
    monkeypatch.setattr(tsm, "_run", make_run([("status", (0, payload))]))
    m._check_status()
    assert m.info.status is TailscaleStatus.CONNECTED
    assert m.info.ip_address == "100.64.0.9"


def test_check_status_bad_json(monkeypatch):
    m = TailscaleManager()
    monkeypatch.setattr(tsm, "_run", make_run([("status", (0, "not json"))]))
    m._check_status()
    assert m.info.status is TailscaleStatus.ERROR


# --------------------------------------------------------------------------- #
# _reconnect
# --------------------------------------------------------------------------- #


def test_reconnect_runs_tailscale_up(monkeypatch):
    seen = []
    monkeypatch.setattr(tsm, "_run", lambda cmd, timeout=10.0: (seen.append(cmd), (0, ""))[1])
    TailscaleManager()._reconnect()
    assert seen == [["tailscale", "up"]]


def test_reconnect_logs_on_failure(monkeypatch):
    monkeypatch.setattr(tsm, "_run", lambda cmd, timeout=10.0: (1, ""))
    TailscaleManager()._reconnect()  # exit != 0 -> warning logged, no raise


# --------------------------------------------------------------------------- #
# _monitor_loop
# --------------------------------------------------------------------------- #


def _drive_loop_once(monkeypatch, m):
    def fake_wait(timeout=None):
        m._stop_event.set()
        return True

    monkeypatch.setattr(m._stop_event, "wait", fake_wait)


def test_monitor_loop_reconnects_when_down(monkeypatch):
    m = TailscaleManager(auto_reconnect=True)
    monkeypatch.setattr(
        m, "_check_status", lambda: setattr(m, "_info", TailscaleInfo(status=TailscaleStatus.DISCONNECTED))
    )
    recon = []
    monkeypatch.setattr(m, "_reconnect", lambda: recon.append(1))
    _drive_loop_once(monkeypatch, m)
    m._monitor_loop()
    assert recon == [1]


def test_monitor_loop_no_reconnect_when_connected(monkeypatch):
    m = TailscaleManager(auto_reconnect=True)
    monkeypatch.setattr(
        m, "_check_status", lambda: setattr(m, "_info", TailscaleInfo(status=TailscaleStatus.CONNECTED))
    )
    monkeypatch.setattr(m, "_reconnect", lambda: (_ for _ in ()).throw(AssertionError("should not reconnect")))
    _drive_loop_once(monkeypatch, m)
    m._monitor_loop()
    assert m.info.status is TailscaleStatus.CONNECTED


def test_monitor_loop_respects_auto_reconnect_off(monkeypatch):
    m = TailscaleManager(auto_reconnect=False)
    monkeypatch.setattr(
        m, "_check_status", lambda: setattr(m, "_info", TailscaleInfo(status=TailscaleStatus.DISCONNECTED))
    )
    monkeypatch.setattr(m, "_reconnect", lambda: (_ for _ in ()).throw(AssertionError("auto_reconnect off")))
    _drive_loop_once(monkeypatch, m)
    m._monitor_loop()


def test_monitor_loop_swallows_errors(monkeypatch):
    m = TailscaleManager()
    calls = {"n": 0}

    def boom():
        calls["n"] += 1
        raise RuntimeError("probe blew up")

    monkeypatch.setattr(m, "_check_status", boom)
    _drive_loop_once(monkeypatch, m)
    m._monitor_loop()  # error caught
    assert calls["n"] == 1


# --------------------------------------------------------------------------- #
# lifecycle + ctor
# --------------------------------------------------------------------------- #


def test_ctor_defaults():
    m = TailscaleManager()
    assert m._check_interval == 10.0
    assert m._auto_reconnect is True
    assert m.info.status is TailscaleStatus.DISCONNECTED


def test_start_runs_loop_then_stop(monkeypatch):
    m = TailscaleManager(check_interval=0.0)
    calls = {"n": 0}

    def fake_check():
        calls["n"] += 1
        m._stop_event.set()

    monkeypatch.setattr(m, "_check_status", fake_check)
    monkeypatch.setattr(m, "_reconnect", lambda: None)
    m.start()
    m._thread.join(timeout=2.0)
    m.stop()
    assert calls["n"] >= 1


def test_start_idempotent_when_alive():
    m = TailscaleManager()
    sentinel = SimpleNamespace(is_alive=lambda: True)
    m._thread = sentinel
    m.start()
    assert m._thread is sentinel


def test_stop_joins_live_thread():
    m = TailscaleManager()
    joined = []
    m._thread = SimpleNamespace(is_alive=lambda: True, join=lambda timeout: joined.append(timeout))
    m.stop()
    assert joined == [2.0] and m._stop_event.is_set()


def test_stop_without_thread_is_noop():
    TailscaleManager().stop()
