# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Route coverage for ``edge_core.api_routes.isaac`` (IsaacModule).

Wires just the isaac management module and drives its status probe (cache-fresh,
cache-stale with the container up/down, and a docker probe failure) plus the
container/bridge start/stop systemd routes. Every ``docker``/``systemctl`` call
is mocked, dispatched on argv, so nothing touches the host.
"""

from __future__ import annotations

import subprocess
import time

from fastapi.testclient import TestClient

from edge_core.api import create_app
from edge_core.core import wire_modules
from edge_core.services.state import StateManager


def _build_app(monkeypatch):
    monkeypatch.delenv("NOMAD_API_KEY", raising=False)
    monkeypatch.setenv("NOMAD_SIM_MODE", "true")
    monkeypatch.setenv("NOMAD_ALLOW_INSECURE_REMOTE", "true")
    app = create_app(StateManager.instance())
    wire_modules(app, allow_list=["isaac_mgmt"])
    return app


def _install_docker(monkeypatch, *, container_status="Up 2 hours", proc_running=True):
    def _run(cmd, *a, **k):
        argv = list(cmd) if isinstance(cmd, (list, tuple)) else [cmd]
        if len(argv) > 1 and argv[1] == "ps":  # docker ps --filter ...
            return subprocess.CompletedProcess(cmd, 0, container_status, "")
        if "pgrep" in argv:  # docker exec <c> pgrep -f <pat>
            return subprocess.CompletedProcess(cmd, 0 if proc_running else 1, "", "")
        return subprocess.CompletedProcess(cmd, 0, "", "")  # systemctl actions

    monkeypatch.setattr(subprocess, "run", _run)


# ==================== GET /api/isaac/status ====================


def test_status_stale_container_running(monkeypatch):
    app = _build_app(monkeypatch)  # default cache timestamp 0.0 -> stale
    _install_docker(monkeypatch, container_status="Up 3 hours", proc_running=True)
    with TestClient(app) as client:
        body = client.get("/api/isaac/status").json()
    assert body["container_running"] is True
    assert body["nvblox_running"] is True
    assert body["bridge_running"] is True


def test_status_stale_container_stopped(monkeypatch):
    app = _build_app(monkeypatch)
    _install_docker(monkeypatch, container_status="", proc_running=True)
    with TestClient(app) as client:
        body = client.get("/api/isaac/status").json()
    assert body["container_running"] is False
    assert body["nvblox_running"] is False  # probes skipped when container down
    assert body["bridge_running"] is False


def test_status_docker_probe_failure(monkeypatch):
    app = _build_app(monkeypatch)

    def _boom(*a, **k):
        raise RuntimeError("docker unreachable")

    monkeypatch.setattr(subprocess, "run", _boom)
    with TestClient(app) as client:
        body = client.get("/api/isaac/status").json()
    assert body["container_running"] is False


def test_status_uses_fresh_cache(monkeypatch):
    app = _build_app(monkeypatch)
    app.state.isaac_runtime_cache = {"timestamp": time.time(), "container_running": True}
    _install_docker(monkeypatch, proc_running=False)
    with TestClient(app) as client:
        body = client.get("/api/isaac/status").json()
    assert body["container_running"] is True  # taken from the fresh cache
    assert body["nvblox_running"] is False  # pgrep returns non-zero


# ==================== start/stop container + bridge ====================


def test_start_container_success(monkeypatch):
    app = _build_app(monkeypatch)
    _install_docker(monkeypatch)
    with TestClient(app) as client:
        body = client.post("/api/isaac/start").json()
    assert body["success"] is True


def test_start_container_failure_returns_500(monkeypatch):
    app = _build_app(monkeypatch)

    def _boom(*a, **k):
        raise subprocess.CalledProcessError(1, "systemctl")

    monkeypatch.setattr(subprocess, "run", _boom)
    with TestClient(app) as client:
        resp = client.post("/api/isaac/start")
    assert resp.status_code == 500


def test_stop_container_success(monkeypatch):
    app = _build_app(monkeypatch)
    _install_docker(monkeypatch)
    with TestClient(app) as client:
        body = client.post("/api/isaac/stop").json()
    assert body["success"] is True


def test_bridge_start_success(monkeypatch):
    app = _build_app(monkeypatch)
    _install_docker(monkeypatch)
    with TestClient(app) as client:
        body = client.post("/api/isaac/bridge/start").json()
    assert body["success"] is True


def test_bridge_stop_success(monkeypatch):
    app = _build_app(monkeypatch)
    _install_docker(monkeypatch)
    with TestClient(app) as client:
        body = client.post("/api/isaac/bridge/stop").json()
    assert body["success"] is True


def test_bridge_start_failure_returns_500(monkeypatch):
    app = _build_app(monkeypatch)

    def _boom(*a, **k):
        raise RuntimeError("systemd down")

    monkeypatch.setattr(subprocess, "run", _boom)
    with TestClient(app) as client:
        resp = client.post("/api/isaac/bridge/start")
    assert resp.status_code == 500
