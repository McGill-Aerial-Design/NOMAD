# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Route coverage for ``edge_core.api_routes.terminal``.

Covers the whitelisted command runner, the gated arbitrary-exec endpoint, the
command listing, and the log tail. The actual process launch
(``_run_subprocess_group``) and ``journalctl`` call are mocked, so the tests
drive the routing/validation/response-shaping logic and the admin-key gate
without spawning anything.
"""

from __future__ import annotations

import subprocess

from fastapi.testclient import TestClient

import edge_core.api_routes.terminal as terminal
from edge_core.api import create_app
from edge_core.services.state import StateManager

_KEY = "admin-key"
_HDR = {"X-API-Key": _KEY}


def _build_app(monkeypatch, *, api_key: str | None = _KEY, enable_exec: bool = False):
    if api_key is None:
        monkeypatch.delenv("NOMAD_API_KEY", raising=False)
    else:
        monkeypatch.setenv("NOMAD_API_KEY", api_key)
    monkeypatch.setenv("NOMAD_ALLOW_INSECURE_REMOTE", "true")
    if enable_exec:
        monkeypatch.setenv("NOMAD_ENABLE_TERMINAL_EXEC", "true")
    else:
        monkeypatch.delenv("NOMAD_ENABLE_TERMINAL_EXEC", raising=False)
    return create_app(StateManager.instance())


def _fake_completed(stdout="", stderr="", rc=0):
    def _run(*a, **k):
        return subprocess.CompletedProcess(a, rc, stdout, stderr)

    return _run


# ==================== /api/terminal/run ====================


def test_run_requires_admin_key(monkeypatch):
    app = _build_app(monkeypatch, api_key=None)  # no key -> handler gate raises 403
    with TestClient(app) as client:
        resp = client.post("/api/terminal/run", json={"command_name": "status_nomad"})
    assert resp.status_code == 403


def test_run_rejects_unknown_command(monkeypatch):
    app = _build_app(monkeypatch)
    with TestClient(app) as client:
        resp = client.post("/api/terminal/run", json={"command_name": "rm_rf_slash"}, headers=_HDR)
    assert resp.status_code == 400
    assert "not allowed" in resp.json()["detail"]


def test_run_simple_command_success(monkeypatch):
    app = _build_app(monkeypatch)
    monkeypatch.setattr(terminal, "_run_subprocess_group", _fake_completed(stdout="active\n", rc=0))
    with TestClient(app) as client:
        body = client.post("/api/terminal/run", json={"command_name": "status_nomad"}, headers=_HDR).json()
    assert body["success"] is True
    assert body["stdout"] == "active\n"
    assert body["command_executed"].startswith("systemctl is-active")


def test_run_shell_command_branch(monkeypatch):
    app = _build_app(monkeypatch)
    # status_novnc contains '|' and '>' so it takes the shell=True path.
    monkeypatch.setattr(terminal, "_run_subprocess_group", _fake_completed(stdout="inactive\n", rc=1))
    with TestClient(app) as client:
        body = client.post("/api/terminal/run", json={"command_name": "status_novnc"}, headers=_HDR).json()
    assert body["success"] is False
    assert body["return_code"] == 1


def test_run_timeout(monkeypatch):
    app = _build_app(monkeypatch)

    def _boom(*a, **k):
        raise subprocess.TimeoutExpired(cmd="x", timeout=10)

    monkeypatch.setattr(terminal, "_run_subprocess_group", _boom)
    with TestClient(app) as client:
        body = client.post("/api/terminal/run", json={"command_name": "status_nomad"}, headers=_HDR).json()
    assert body["success"] is False
    assert "timed out" in body["stderr"]


def test_run_generic_exception(monkeypatch):
    app = _build_app(monkeypatch)

    def _boom(*a, **k):
        raise RuntimeError("kaboom")

    monkeypatch.setattr(terminal, "_run_subprocess_group", _boom)
    with TestClient(app) as client:
        body = client.post("/api/terminal/run", json={"command_name": "status_nomad"}, headers=_HDR).json()
    assert body["success"] is False
    assert body["stderr"] == "kaboom"


# ==================== /api/terminal/exec ====================


def test_exec_disabled_returns_403(monkeypatch):
    app = _build_app(monkeypatch, enable_exec=False)
    with TestClient(app) as client:
        resp = client.post("/api/terminal/exec", json={"command": "ls"}, headers=_HDR)
    assert resp.status_code == 403


def test_exec_empty_command_returns_400(monkeypatch):
    app = _build_app(monkeypatch, enable_exec=True)
    with TestClient(app) as client:
        resp = client.post("/api/terminal/exec", json={"command": "   "}, headers=_HDR)
    assert resp.status_code == 400


def test_exec_success_extracts_cwd(monkeypatch):
    app = _build_app(monkeypatch, enable_exec=True)
    stdout = "hello world\n__NOMAD_CWD__/home/pilot\n"
    monkeypatch.setattr(terminal, "_run_subprocess_group", _fake_completed(stdout=stdout, rc=0))
    with TestClient(app) as client:
        body = client.post("/api/terminal/exec", json={"command": "echo hello world"}, headers=_HDR).json()
    assert body["success"] is True
    assert body["cwd"] == "/home/pilot"
    assert "hello world" in body["stdout"]
    assert "__NOMAD_CWD__" not in body["stdout"]


def test_exec_timeout(monkeypatch):
    app = _build_app(monkeypatch, enable_exec=True)

    def _boom(*a, **k):
        raise subprocess.TimeoutExpired(cmd="x", timeout=30)

    monkeypatch.setattr(terminal, "_run_subprocess_group", _boom)
    with TestClient(app) as client:
        body = client.post("/api/terminal/exec", json={"command": "sleep 99"}, headers=_HDR).json()
    assert body["success"] is False
    assert "timed out" in body["stderr"]


# ==================== /api/terminal/commands + /logs ====================


def test_list_commands(monkeypatch):
    app = _build_app(monkeypatch)
    with TestClient(app) as client:
        body = client.get("/api/terminal/commands", headers=_HDR).json()
    assert "status_nomad" in body["commands"]


def test_logs_success(monkeypatch):
    app = _build_app(monkeypatch)
    monkeypatch.setattr(subprocess, "run", _fake_completed(stdout="line1\nline2\n", rc=0))
    with TestClient(app) as client:
        body = client.get("/api/terminal/logs?service=nomad-edge-core&lines=10", headers=_HDR).json()
    assert body["service"] == "nomad-edge-core"
    assert "line1" in body["logs"]
    assert body["lines"] == 10


def test_logs_error_returns_error_field(monkeypatch):
    app = _build_app(monkeypatch)

    def _boom(*a, **k):
        raise RuntimeError("no journalctl")

    monkeypatch.setattr(subprocess, "run", _boom)
    with TestClient(app) as client:
        body = client.get("/api/terminal/logs", headers=_HDR).json()
    assert body["error"] == "no journalctl"
