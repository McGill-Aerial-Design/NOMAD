# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""I/O-surface tests for the payload servo adapter (non-safety paths).

The SR-PAY-01/02/03 fault-injection guarantees live in ``test_payload_servo``;
this file covers the remaining MAVLink-I/O surface of ``MavlinkServo`` /
``ServoController`` and the module-global controller helpers (camera tilt,
status, configuration validation, lifecycle) so the adapter is fully exercised.
The safety decisions themselves stay in the 100%-covered ``edge_core.safety``.
"""

from __future__ import annotations

import pytest

from edge_core.modules.payload import servo as servo_mod
from edge_core.modules.payload.servo import (
    MavlinkServo,
    ServoController,
    get_servo_controller,
    init_servo_controller,
    shutdown_servo_controller,
)


class _FakeMav:
    def __init__(self, ok: bool = True):
        self.ok = ok
        self.calls: list[tuple] = []

    def trigger_payload(self, pwm: int, channel: int) -> bool:
        self.calls.append(("trigger_payload", pwm, channel))
        return self.ok


@pytest.fixture(autouse=True)
def _reset_global_controller():
    """The controller is a module global; reset it around each test."""
    servo_mod._controller = None
    yield
    servo_mod._controller = None


# --------------------------------------------------------------------------- #
# MavlinkServo
# --------------------------------------------------------------------------- #


def test_servo_initialize_rejects_out_of_range_channel():
    assert MavlinkServo("s", None, channel=0).initialize() is False
    assert MavlinkServo("s", None, channel=99).initialize() is False
    assert MavlinkServo("s", None, channel=14).initialize() is True


def test_servo_set_angle_clamps_and_transmits():
    mav = _FakeMav()
    s = MavlinkServo("tilt", mav, channel=14)
    assert s.set_angle(90.0) is True
    assert mav.calls == [("trigger_payload", 1500, 14)]  # midpoint -> 1500us
    # Above max clamps to max angle (180 -> 2500us).
    s.set_angle(999.0)
    assert mav.calls[-1] == ("trigger_payload", 2500, 14)
    assert s.get_state().angle == 180.0


def test_servo_set_pwm_rejects_invalid_then_returns_mav_result():
    # Out-of-range PWM is blocked by the SC validator before any mav call.
    bad = MavlinkServo("s", _FakeMav(), channel=14)
    assert bad.set_pwm(100) is False
    assert bad._mav.calls == []
    # No mav service -> False.
    assert MavlinkServo("s", None, channel=14).set_pwm(1500) is False
    # Valid -> mav result is returned (success and failure).
    assert MavlinkServo("s", _FakeMav(ok=True), channel=14).set_pwm(1500) is True
    assert MavlinkServo("s", _FakeMav(ok=False), channel=14).set_pwm(1500) is False


def test_angle_to_pulse_us_endpoints():
    s = MavlinkServo("s", None, channel=14)
    assert s._angle_to_pulse_us(0.0) == 500
    assert s._angle_to_pulse_us(180.0) == 2500


# --------------------------------------------------------------------------- #
# ServoController: camera tilt + config
# --------------------------------------------------------------------------- #


def _ctrl(ok: bool = True) -> tuple[ServoController, _FakeMav]:
    mav = _FakeMav(ok=ok)
    c = ServoController(mavlink_service=mav)
    c.initialize()
    return c, mav


def test_configure_camera_tilt_validates_channel():
    c, _ = _ctrl()
    assert c.configure_camera_tilt_mavlink(0) is False
    assert c.configure_camera_tilt_mavlink(99) is False
    assert c.configure_camera_tilt_mavlink(14) is True


def test_configure_camera_tilt_preserves_last_angle():
    c, _ = _ctrl()
    c.configure_camera_tilt_mavlink(14)
    c.set_camera_tilt(45.0)
    # Reconfiguring on a new channel keeps the last commanded angle.
    c.configure_camera_tilt_mavlink(13)
    assert c.get_camera_tilt() == 45.0


def test_set_and_get_camera_tilt_when_unconfigured():
    c, _ = _ctrl()
    assert c.set_camera_tilt(30.0) is False  # not configured
    assert c.get_camera_tilt() is None


def test_set_camera_tilt_transmits_when_configured():
    c, mav = _ctrl()
    c.configure_camera_tilt_mavlink(14)
    assert c.set_camera_tilt(0.0) is True
    assert ("trigger_payload", 500, 14) in mav.calls


def test_configure_water_pump_relay_validates():
    c, _ = _ctrl()
    assert c.configure_water_pump_relay(-1) is False
    assert c.configure_water_pump_relay(16) is False
    assert c.configure_water_pump_relay(2) is True


def test_set_channel_pwm_without_mav_service():
    c = ServoController(mavlink_service=None)
    c.initialize()
    assert c.set_channel_pwm(8, 1500) is False  # valid range but no link


def test_get_status_reports_servos_and_relay():
    c, _ = _ctrl()
    c.configure_water_pump_relay(3)
    c.configure_camera_tilt_mavlink(14)
    status = c.get_status()
    assert status["initialized"] is True
    assert status["mode"] == "cube_orange_mavlink"
    assert status["servo_count"] == 1
    assert status["relays"]["water_shooter"]["relay_number"] == 3
    assert status["servos"]["camera_tilt"]["channel"] == 14


def test_lifecycle_flags():
    c, _ = _ctrl()
    assert c.is_available() is True
    c.shutdown()
    assert c.is_available() is False
    new_mav = _FakeMav()
    c.set_mavlink_service(new_mav)
    assert c._mavlink_service is new_mav


# --------------------------------------------------------------------------- #
# module-global controller helpers
# --------------------------------------------------------------------------- #


def test_init_creates_controller_and_configures_tilt():
    mav = _FakeMav()
    assert init_servo_controller(mavlink_service=mav, camera_tilt_channel=14) is True
    c = get_servo_controller()
    assert c is not None
    assert c.get_camera_tilt() == 90.0  # tilt configured at neutral


def test_init_twice_rebinds_mavlink_service():
    first = _FakeMav()
    init_servo_controller(mavlink_service=first)
    second = _FakeMav()
    init_servo_controller(mavlink_service=second)  # existing controller -> rebind
    assert get_servo_controller()._mavlink_service is second


def test_init_without_tilt_channel_leaves_tilt_unconfigured():
    init_servo_controller(mavlink_service=_FakeMav(), camera_tilt_channel=None)
    assert get_servo_controller().get_camera_tilt() is None


def test_shutdown_clears_global():
    init_servo_controller(mavlink_service=_FakeMav())
    shutdown_servo_controller()
    assert get_servo_controller() is None


def test_get_controller_none_before_init():
    assert get_servo_controller() is None


def test_shutdown_when_never_initialized_is_noop(monkeypatch):
    shutdown_servo_controller()  # _controller is None -> no error
    assert get_servo_controller() is None


# --------------------------------------------------------------------------- #
# defensive branches
# --------------------------------------------------------------------------- #


def test_configure_camera_tilt_returns_false_when_servo_init_fails(monkeypatch):
    c, _ = _ctrl()
    # Channel passes the range check, but the servo's own initialize() fails.
    monkeypatch.setattr(servo_mod.MavlinkServo, "initialize", lambda self: False)
    assert c.configure_camera_tilt_mavlink(14) is False


def test_trigger_water_shooter_without_mav_after_arm():
    c = ServoController(mavlink_service=None)
    c.initialize()
    c.configure_water_pump_relay(2)
    c.arm_release()  # interlock opens, but there is no link to fire
    assert c.trigger_water_shooter(200) is False


def test_relay_off_best_effort_exception_is_swallowed(monkeypatch):
    class _RelayRaiseMav:
        def set_relay(self, relay: int, enabled: bool) -> bool:
            if enabled:
                return False  # the on-command fails
            raise RuntimeError("off also failed")  # best-effort off then raises

    monkeypatch.setattr(servo_mod.time, "sleep", lambda *_: None)
    c = ServoController(mavlink_service=_RelayRaiseMav())
    c.initialize()
    c.configure_water_pump_relay(2)
    c.arm_release()
    # Returns False; the exception during best-effort de-energize is swallowed.
    assert c.trigger_water_shooter(200) is False
