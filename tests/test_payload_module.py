# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for edge_core.services.payload_module.PayloadModule.

The module wires the global servo controller into the SDK and exposes the
``/api/servo/shooter/*`` command routes. The routes are mounted on a bare
FastAPI app (no app-level command auth) and driven with TestClient; the
controller is a real ``ServoController`` with a fake MAVLink service so the
arm->trigger interlock behaves as in production.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from edge_core.modules.payload import servo as servo_mod
from edge_core.modules.payload.servo import ServoController
from edge_core.services.payload_module import PayloadModule


class _FakeMav:
    def set_relay(self, relay: int, enabled: bool) -> bool:
        return True

    def trigger_payload(self, pwm: int, channel: int) -> bool:
        return True


@pytest.fixture(autouse=True)
def _reset_global_controller(monkeypatch):
    servo_mod._controller = None
    # The pump-on window sleeps for its duration; keep tests fast.
    monkeypatch.setattr(servo_mod.time, "sleep", lambda *_: None)
    yield
    servo_mod._controller = None


def _install_controller(relay: int = 2) -> ServoController:
    c = ServoController(mavlink_service=_FakeMav())
    c.initialize()
    c.configure_water_pump_relay(relay)
    servo_mod._controller = c
    return c


def _client() -> TestClient:
    app = FastAPI()
    PayloadModule().register_routes(app)
    return TestClient(app)


# --------------------------------------------------------------------------- #
# configure / stop
# --------------------------------------------------------------------------- #


def test_configure_wires_controller_into_context():
    registered: dict = {}
    app = SimpleNamespace(state=SimpleNamespace())
    ctx = SimpleNamespace(
        require_service=lambda name: _FakeMav(),
        get_config=lambda key, default=None: "14",
        register_service=lambda name, svc: registered.__setitem__(name, svc),
        app=app,
    )
    PayloadModule().configure(ctx)
    assert "servo_controller" in registered
    assert app.state.servo_controller is registered["servo_controller"]
    # Tilt channel 14 was configured from get_config.
    assert registered["servo_controller"].get_camera_tilt() == 90.0


def test_stop_shuts_down_controller():
    c = _install_controller()
    PayloadModule().stop()
    assert c.is_available() is False


def test_stop_is_safe_without_controller():
    servo_mod._controller = None
    PayloadModule().stop()  # no controller -> no error


# --------------------------------------------------------------------------- #
# /api/servo/shooter/arm
# --------------------------------------------------------------------------- #


def test_arm_returns_window():
    _install_controller()
    resp = _client().post("/api/servo/shooter/arm")
    assert resp.status_code == 200
    body = resp.json()
    assert body["success"] is True
    assert body["armed_for_s"] > 0


def test_arm_503_without_controller():
    servo_mod._controller = None
    assert _client().post("/api/servo/shooter/arm").status_code == 503


# --------------------------------------------------------------------------- #
# /api/servo/shooter/trigger
# --------------------------------------------------------------------------- #


def test_trigger_503_without_controller():
    servo_mod._controller = None
    assert _client().post("/api/servo/shooter/trigger").status_code == 503


def test_trigger_400_on_invalid_relay():
    _install_controller()
    resp = _client().post("/api/servo/shooter/trigger", params={"relay_number": 99})
    assert resp.status_code == 400


def test_trigger_409_without_prior_arm():
    _install_controller()
    resp = _client().post("/api/servo/shooter/trigger", params={"duration_ms": 200})
    assert resp.status_code == 409


def test_trigger_succeeds_after_arm():
    _install_controller()
    client = _client()
    assert client.post("/api/servo/shooter/arm").status_code == 200
    resp = client.post("/api/servo/shooter/trigger", params={"duration_ms": 200})
    assert resp.status_code == 200
    assert resp.json() == {"success": True}


def test_trigger_accepts_valid_relay_override_then_needs_arm():
    _install_controller()
    client = _client()
    client.post("/api/servo/shooter/arm")
    # relay_number=3 is valid (0-15) -> reconfigures, then fires with the arm.
    resp = client.post("/api/servo/shooter/trigger", params={"relay_number": 3, "duration_ms": 100})
    assert resp.status_code == 200
