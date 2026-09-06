# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Route coverage for ``edge_core.api_routes.calibration`` (CalibrationModule).

Wires just the calibration module onto a live app and drives the ZED
calibration routes. The vehicle actuation routes (servo PWM, camera tilt,
spray relay, shooter arm/trigger) were removed during the C++ cutover —
the C++ core CLI and ``NomadCoreClient`` own those commands now — so this
module must NOT serve any ``/api/servo/*`` or ``/api/spray/*`` path.
"""

from __future__ import annotations

import subprocess

from fastapi.testclient import TestClient

from edge_core.api import create_app
from edge_core.core import wire_modules
from edge_core.services.state import StateManager

_KEY = "cal-key"
_HDR = {"X-API-Key": _KEY}


def _build_app(monkeypatch):
    monkeypatch.setenv("NOMAD_API_KEY", _KEY)
    monkeypatch.setenv("NOMAD_SIM_MODE", "true")
    monkeypatch.setenv("NOMAD_ALLOW_INSECURE_REMOTE", "true")
    app = create_app(StateManager.instance())
    wire_modules(app, allow_list=["calibration"])
    return app


def test_calibration_module_serves_no_vehicle_command_paths(monkeypatch):
    app = _build_app(monkeypatch)
    served = set(app.openapi()["paths"])
    leftover = [path for path in served if path.startswith("/api/servo") or path.startswith("/api/spray")]
    assert not leftover, f"vehicle command paths survived the cutover: {leftover}"


def test_imu_reset_success(monkeypatch):
    app = _build_app(monkeypatch)
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: subprocess.CompletedProcess(a, 0, "", ""))
    with TestClient(app) as client:
        body = client.post("/api/calibration/imu/reset_biases", headers=_HDR).json()
    assert body["success"] is True


def test_imu_reset_failure_returns_502(monkeypatch):
    app = _build_app(monkeypatch)

    def _boom(*a, **k):
        raise RuntimeError("no ZED")

    monkeypatch.setattr(subprocess, "run", _boom)
    with TestClient(app) as client:
        resp = client.post("/api/calibration/imu/reset_biases", headers=_HDR)
    assert resp.status_code == 502


def test_sensor_viewer_launch_success(monkeypatch):
    app = _build_app(monkeypatch)
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **k: object())
    with TestClient(app) as client:
        body = client.post("/api/calibration/zed/sensor-viewer/start", headers=_HDR).json()
    assert body["success"] is True


def test_sensor_viewer_launch_failure_returns_502(monkeypatch):
    app = _build_app(monkeypatch)

    def _boom(*a, **k):
        raise RuntimeError("no display")

    monkeypatch.setattr(subprocess, "Popen", _boom)
    with TestClient(app) as client:
        resp = client.post("/api/calibration/zed/sensor-viewer/start", headers=_HDR)
    assert resp.status_code == 502
