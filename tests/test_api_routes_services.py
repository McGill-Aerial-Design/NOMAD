# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Route coverage for ``edge_core.api_routes.services``.

The ``/api/services/status`` endpoint fans out to ~10 ``systemctl``/``pgrep``/
``docker`` subprocesses and a ``/dev/ttyACM0`` probe. Those are all mocked here
so the test drives the service-state aggregation, the flight-controller-present gate,
the TTL cache, and the VIO/Isaac branches without touching the host.
"""

from __future__ import annotations

import os
import subprocess

from fastapi.testclient import TestClient

from edge_core.api import create_app
from edge_core.services.state import StateManager


def _build_app(monkeypatch):
    monkeypatch.delenv("NOMAD_API_KEY", raising=False)
    monkeypatch.setenv("NOMAD_ALLOW_INSECURE_REMOTE", "true")
    return create_app(StateManager.instance())


def _install_fakes(monkeypatch, *, fc_present=True, active=True, container="Up 2 hours", calls=None):
    """Patch subprocess.run and os.path.exists to deterministic responses."""

    def _run(cmd, *a, **k):
        if calls is not None:
            calls.append(cmd)
        prog = (cmd[0] if isinstance(cmd, (list, tuple)) else cmd) or ""
        if prog == "systemctl":
            return subprocess.CompletedProcess(cmd, 0 if active else 3, "active" if active else "inactive", "")
        if prog == "pgrep":
            return subprocess.CompletedProcess(cmd, 0 if active else 1, "4321" if active else "", "")
        if prog == "docker":
            return subprocess.CompletedProcess(cmd, 0, container, "")
        return subprocess.CompletedProcess(cmd, 0, "", "")

    monkeypatch.setattr(subprocess, "run", _run)

    real_exists = os.path.exists

    def _exists(path):
        if path == "/dev/ttyACM0":
            return fc_present
        return real_exists(path)

    monkeypatch.setattr(os.path, "exists", _exists)


def test_all_services_active_with_flight_controller(monkeypatch):
    app = _build_app(monkeypatch)
    _install_fakes(monkeypatch, fc_present=True, active=True, container="Up 5 minutes")
    with TestClient(app) as client:
        body = client.get("/api/services/status").json()
    assert body["mavlink_router"]["running"] is True
    assert body["mavlink_router"]["flight_controller_present"] is True
    assert body["mediamtx"]["running"] is True
    assert body["novnc"]["running"] is True
    assert body["edge_core"] == {"status": "active", "running": True}
    assert body["isaac_ros"]["running"] is True
    assert body["isaac_ros_container"]["status"] == "running"
    assert body["vio"]["status"] == "not_initialized"


def test_no_flight_controller_and_inactive_services(monkeypatch):
    app = _build_app(monkeypatch)
    _install_fakes(monkeypatch, fc_present=False, active=False, container="")
    with TestClient(app) as client:
        body = client.get("/api/services/status").json()
    assert body["mavlink_router"]["status"] == "no_flight_controller"
    assert body["mavlink_router"]["running"] is False
    assert body["mediamtx"]["running"] is False
    assert body["novnc"]["running"] is False
    assert body["isaac_ros"]["status"] == "not_initialized"
    assert body["isaac_ros_container"]["status"] == "not_running"


def test_vio_state_present(monkeypatch):
    app = _build_app(monkeypatch)
    app.state.external_vio_state = {"source": "zed-mini", "confidence": 73}
    _install_fakes(monkeypatch, container="Up 1 hour")
    with TestClient(app) as client:
        body = client.get("/api/services/status").json()
    assert body["vio"]["status"] == "active"
    assert body["vio"]["source"] == "zed-mini"
    assert body["vio"]["confidence"] == 73


def test_ttl_cache_skips_second_subprocess_fanout(monkeypatch):
    app = _build_app(monkeypatch)
    calls: list = []
    _install_fakes(monkeypatch, calls=calls)
    with TestClient(app) as client:
        client.get("/api/services/status")
        first = len(calls)
        client.get("/api/services/status")  # within 2s TTL -> cache hit
        second = len(calls)
    assert first > 0
    assert second == first  # no new subprocesses on the cached call


def test_probe_failures_are_swallowed(monkeypatch):
    app = _build_app(monkeypatch)

    def _boom(*a, **k):
        raise RuntimeError("probe down")

    monkeypatch.setattr(subprocess, "run", _boom)
    monkeypatch.setattr(os.path, "exists", lambda p: False)
    with TestClient(app) as client:
        resp = client.get("/api/services/status")
    # Every probe is wrapped; a hard failure degrades to a safe default, not 500.
    assert resp.status_code == 200
    body = resp.json()
    assert body["mavlink_router"]["running"] is False
    assert body["isaac_ros"]["running"] is False
