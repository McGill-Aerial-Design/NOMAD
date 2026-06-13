# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Route coverage for ``edge_core.api_routes.calibration`` (CalibrationModule).

Wires just the calibration module onto a live app, attaches a fake servo
controller to ``app.state``, and drives the servo/spray/tilt + ZED calibration
routes. The ``/api/servo/*`` and ``/api/spray/*`` paths are SC command paths, so
the app is built with an API key and requests carry ``X-API-Key``. PWM
rejection is asserted against the real ``edge_core.safety`` envelope.
"""

from __future__ import annotations

import subprocess

from fastapi.testclient import TestClient

from edge_core.api import create_app
from edge_core.core import wire_modules
from edge_core.services.state import StateManager

_KEY = "cal-key"
_HDR = {"X-API-Key": _KEY}


class _FakeServo:
    def __init__(self, *, pwm_ok=True, tilt_ok=True, tilt_angle=90.0, relay_ok=True):
        self.pwm_ok = pwm_ok
        self.tilt_ok = tilt_ok
        self.tilt_angle = tilt_angle
        self.relay_ok = relay_ok
        self.configured_channel = None
        self.configured_relay = None

    def set_channel_pwm(self, channel, pwm):
        return self.pwm_ok

    def set_camera_tilt(self, angle):
        return self.tilt_ok

    def get_camera_tilt(self):
        return self.tilt_angle

    def configure_camera_tilt_mavlink(self, channel):
        self.configured_channel = channel

    def configure_water_pump_relay(self, relay):
        self.configured_relay = relay
        return self.relay_ok


def _build_app(monkeypatch, servo=None):
    monkeypatch.setenv("NOMAD_API_KEY", _KEY)
    monkeypatch.setenv("NOMAD_SIM_MODE", "true")
    monkeypatch.setenv("NOMAD_ALLOW_INSECURE_REMOTE", "true")
    app = create_app(StateManager.instance())
    wire_modules(app, allow_list=["calibration"])
    app.state.servo_controller = servo
    return app


# ==================== /api/servo/channel/{channel}/pwm ====================


def test_pwm_rejected_by_safety_envelope(monkeypatch):
    app = _build_app(monkeypatch, servo=_FakeServo())
    with TestClient(app) as client:
        resp = client.post("/api/servo/channel/99/pwm?pwm=1500", headers=_HDR)
    assert resp.status_code == 400  # channel 99 is outside 1-16 (SR-PAY-01)


def test_pwm_no_controller_returns_503(monkeypatch):
    app = _build_app(monkeypatch, servo=None)
    with TestClient(app) as client:
        resp = client.post("/api/servo/channel/14/pwm?pwm=1500", headers=_HDR)
    assert resp.status_code == 503


def test_pwm_success(monkeypatch):
    app = _build_app(monkeypatch, servo=_FakeServo(pwm_ok=True))
    with TestClient(app) as client:
        resp = client.post("/api/servo/channel/14/pwm?pwm=1500", headers=_HDR)
    assert resp.status_code == 200
    assert resp.json() == {"success": True}


def test_pwm_mavlink_rejected_returns_502(monkeypatch):
    app = _build_app(monkeypatch, servo=_FakeServo(pwm_ok=False))
    with TestClient(app) as client:
        resp = client.post("/api/servo/channel/14/pwm?pwm=1500", headers=_HDR)
    assert resp.status_code == 502


# ==================== /api/servo/camera/tilt ====================


def test_camera_tilt_set_success(monkeypatch):
    app = _build_app(monkeypatch, servo=_FakeServo(tilt_ok=True))
    with TestClient(app) as client:
        body = client.post("/api/servo/camera/tilt?angle=45", headers=_HDR).json()
    assert body == {"success": True, "angle": 45.0}


def test_camera_tilt_set_no_controller(monkeypatch):
    app = _build_app(monkeypatch, servo=None)
    with TestClient(app) as client:
        resp = client.post("/api/servo/camera/tilt?angle=45", headers=_HDR)
    assert resp.status_code == 503


def test_camera_tilt_set_rejected(monkeypatch):
    app = _build_app(monkeypatch, servo=_FakeServo(tilt_ok=False))
    with TestClient(app) as client:
        resp = client.post("/api/servo/camera/tilt?angle=45", headers=_HDR)
    assert resp.status_code == 502


def test_camera_tilt_get_success(monkeypatch):
    app = _build_app(monkeypatch, servo=_FakeServo(tilt_angle=120.0))
    with TestClient(app) as client:
        body = client.get("/api/servo/camera/tilt", headers=_HDR).json()
    assert body == {"angle": 120.0}


def test_camera_tilt_get_unconfigured_returns_503(monkeypatch):
    app = _build_app(monkeypatch, servo=_FakeServo(tilt_angle=None))
    with TestClient(app) as client:
        resp = client.get("/api/servo/camera/tilt", headers=_HDR)
    assert resp.status_code == 503


def test_camera_tilt_get_no_controller(monkeypatch):
    app = _build_app(monkeypatch, servo=None)
    with TestClient(app) as client:
        resp = client.get("/api/servo/camera/tilt", headers=_HDR)
    assert resp.status_code == 503


# ==================== /api/servo/camera/config + /api/spray/calibration ====================


def test_camera_config_success(monkeypatch):
    servo = _FakeServo()
    app = _build_app(monkeypatch, servo=servo)
    with TestClient(app) as client:
        body = client.post("/api/servo/camera/config", json={"channel": 12}, headers=_HDR).json()
    assert body == {"success": True, "channel": 12}
    assert servo.configured_channel == 12


def test_camera_config_no_controller(monkeypatch):
    app = _build_app(monkeypatch, servo=None)
    with TestClient(app) as client:
        resp = client.post("/api/servo/camera/config", json={"channel": 12}, headers=_HDR)
    assert resp.status_code == 503


def test_spray_calibration_success(monkeypatch):
    servo = _FakeServo(relay_ok=True)
    app = _build_app(monkeypatch, servo=servo)
    with TestClient(app) as client:
        body = client.post("/api/spray/calibration", json={"water_pump_relay_number": 5}, headers=_HDR).json()
    assert body == {"success": True, "relay_number": 5}
    assert servo.configured_relay == 5


def test_spray_calibration_invalid_relay_returns_400(monkeypatch):
    app = _build_app(monkeypatch, servo=_FakeServo(relay_ok=False))
    with TestClient(app) as client:
        resp = client.post("/api/spray/calibration", json={"water_pump_relay_number": 5}, headers=_HDR)
    assert resp.status_code == 400


def test_spray_calibration_rejects_extra_fields(monkeypatch):
    app = _build_app(monkeypatch, servo=_FakeServo())
    with TestClient(app) as client:
        resp = client.post(
            "/api/spray/calibration",
            json={"water_pump_relay_number": 5, "gain": 1.2},
            headers=_HDR,
        )
    assert resp.status_code == 422  # extra="forbid"


def test_spray_calibration_no_controller(monkeypatch):
    app = _build_app(monkeypatch, servo=None)
    with TestClient(app) as client:
        resp = client.post("/api/spray/calibration", json={"water_pump_relay_number": 5}, headers=_HDR)
    assert resp.status_code == 503


# ==================== /api/calibration/* (ZED tools) ====================


def test_imu_reset_success(monkeypatch):
    app = _build_app(monkeypatch, servo=_FakeServo())
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: subprocess.CompletedProcess(a, 0, "", ""))
    with TestClient(app) as client:
        body = client.post("/api/calibration/imu/reset_biases", headers=_HDR).json()
    assert body["success"] is True


def test_imu_reset_failure_returns_502(monkeypatch):
    app = _build_app(monkeypatch, servo=_FakeServo())

    def _boom(*a, **k):
        raise RuntimeError("no ZED")

    monkeypatch.setattr(subprocess, "run", _boom)
    with TestClient(app) as client:
        resp = client.post("/api/calibration/imu/reset_biases", headers=_HDR)
    assert resp.status_code == 502


def test_sensor_viewer_launch_success(monkeypatch):
    app = _build_app(monkeypatch, servo=_FakeServo())
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **k: object())
    with TestClient(app) as client:
        body = client.post("/api/calibration/zed/sensor-viewer/start", headers=_HDR).json()
    assert body["success"] is True


def test_sensor_viewer_launch_failure_returns_502(monkeypatch):
    app = _build_app(monkeypatch, servo=_FakeServo())

    def _boom(*a, **k):
        raise RuntimeError("no display")

    monkeypatch.setattr(subprocess, "Popen", _boom)
    with TestClient(app) as client:
        resp = client.post("/api/calibration/zed/sensor-viewer/start", headers=_HDR)
    assert resp.status_code == 502
