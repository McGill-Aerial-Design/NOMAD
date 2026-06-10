# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Fault-injection tests for the payload servo/relay adapter (SR-PAY-01/02/03).

Proves the H-06 guarantees on ``ServoController``: range validation before any
MAVLink transmission, the arm->release interlock on the pump path, and — the
critical one — the pump relay is de-energized on **every** exit path, including
an exception mid-pulse. The pump must never be left energized.
"""

from __future__ import annotations

import math
import time

import pytest

from edge_core.modules.payload.servo import ServoController


class _FakeMav:
    def __init__(self, relay_ok: bool = True):
        self.calls: list[tuple] = []
        self.relay_ok = relay_ok

    def set_relay(self, relay: int, enabled: bool) -> bool:
        self.calls.append(("set_relay", relay, enabled))
        return self.relay_ok if enabled else True

    def trigger_payload(self, pwm: int, channel: int) -> bool:
        self.calls.append(("trigger_payload", pwm, channel))
        return True


def _controller(relay_ok: bool = True) -> tuple[ServoController, _FakeMav]:
    ctrl = ServoController()
    mav = _FakeMav(relay_ok=relay_ok)
    ctrl._mavlink_service = mav
    ctrl.initialize()
    ctrl.configure_water_pump_relay(2)
    return ctrl, mav


# --------------------------------------------------------------------------- #
# SR-PAY-03: interlock on the release path
# --------------------------------------------------------------------------- #
def test_release_without_arm_sends_nothing(monkeypatch):
    ctrl, mav = _controller()
    assert ctrl.trigger_water_shooter(200) is False
    assert mav.calls == []


def test_armed_release_fires_then_deenergizes(monkeypatch):
    ctrl, mav = _controller()
    monkeypatch.setattr(time, "sleep", lambda _s: None)
    ctrl.arm_release()
    assert ctrl.trigger_water_shooter(200) is True
    assert mav.calls == [("set_relay", 2, True), ("set_relay", 2, False)]


def test_arm_is_consumed_by_each_attempt(monkeypatch):
    ctrl, mav = _controller()
    monkeypatch.setattr(time, "sleep", lambda _s: None)
    ctrl.arm_release()
    assert ctrl.trigger_water_shooter(200) is True
    assert ctrl.trigger_water_shooter(200) is False
    # Only the first (armed) attempt reached the relay.
    assert mav.calls == [("set_relay", 2, True), ("set_relay", 2, False)]


# --------------------------------------------------------------------------- #
# SR-PAY-02: the pump can never stay energized
# --------------------------------------------------------------------------- #
def test_pump_deenergized_when_sleep_raises(monkeypatch):
    def _boom(_s: float) -> None:
        raise RuntimeError("injected fault mid-pulse")

    ctrl, mav = _controller()
    monkeypatch.setattr(time, "sleep", _boom)
    ctrl.arm_release()
    with pytest.raises(RuntimeError):
        ctrl.trigger_water_shooter(200)
    # The finally block still turned the relay off.
    assert mav.calls == [("set_relay", 2, True), ("set_relay", 2, False)]


def test_relay_on_failure_still_attempts_off(monkeypatch):
    ctrl, mav = _controller(relay_ok=False)
    ctrl.arm_release()
    assert ctrl.trigger_water_shooter(200) is False
    assert mav.calls == [("set_relay", 2, True), ("set_relay", 2, False)]


def test_nonfinite_duration_rejected_before_energizing(monkeypatch):
    ctrl, mav = _controller()
    ctrl.arm_release()
    assert ctrl.trigger_water_shooter(math.nan) is False  # type: ignore[arg-type]
    assert mav.calls == []


def test_duration_is_clamped(monkeypatch):
    slept: list[float] = []
    ctrl, mav = _controller()
    monkeypatch.setattr(time, "sleep", slept.append)
    ctrl.arm_release()
    assert ctrl.trigger_water_shooter(60_000) is True  # 60 s requested
    assert slept == [5.0]


# --------------------------------------------------------------------------- #
# SR-PAY-01: range validation before transmission
# --------------------------------------------------------------------------- #
def test_set_channel_pwm_rejects_out_of_range():
    ctrl, mav = _controller()
    assert ctrl.set_channel_pwm(0, 1500) is False
    assert ctrl.set_channel_pwm(17, 1500) is False
    assert ctrl.set_channel_pwm(8, 499) is False
    assert ctrl.set_channel_pwm(8, 2501) is False
    assert mav.calls == []


def test_set_channel_pwm_sends_valid_command():
    ctrl, mav = _controller()
    assert ctrl.set_channel_pwm(8, 1500) is True
    assert mav.calls == [("trigger_payload", 1500, 8)]
