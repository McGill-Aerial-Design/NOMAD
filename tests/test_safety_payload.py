# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Unit tests for edge_core.safety.payload (SR-PAY-01/02/03, hazard H-06).

Covers the pure payload decisions: servo channel/PWM validation, the pump
duration clamp (including non-finite rejection), and the arm->release
interlock state machine — every refusal path leaves the interlock disarmed.
"""

from __future__ import annotations

import math

import pytest

from edge_core.safety.payload import (
    InterlockPolicy,
    InterlockState,
    arm_release,
    clamp_release_duration,
    evaluate_release,
    validate_servo_command,
)


# --------------------------------------------------------------------------- #
# SR-PAY-01: channel / PWM validation
# --------------------------------------------------------------------------- #
def test_validate_servo_command_accepts_full_range():
    assert validate_servo_command(1, 500).allowed is True
    assert validate_servo_command(16, 2500).allowed is True


def test_validate_servo_command_rejects_bad_channel():
    for channel in (0, -1, 17):
        decision = validate_servo_command(channel, 1500)
        assert decision.allowed is False
        assert decision.reason == "channel"


def test_validate_servo_command_rejects_bad_pwm():
    for pwm in (499, 2501, -100):
        decision = validate_servo_command(8, pwm)
        assert decision.allowed is False
        assert decision.reason == "pwm"


def test_channel_gate_precedes_pwm_gate():
    # Both invalid: the channel check fires first (stable gate order).
    assert validate_servo_command(0, 9999).reason == "channel"


# --------------------------------------------------------------------------- #
# SR-PAY-02: duration clamp
# --------------------------------------------------------------------------- #
def test_clamp_release_duration_clamps_both_ends():
    assert clamp_release_duration(0.001) == 0.05
    assert clamp_release_duration(60.0) == 5.0
    assert clamp_release_duration(0.2) == 0.2


def test_clamp_release_duration_rejects_nonfinite():
    for bad in (math.nan, math.inf, -math.inf):
        with pytest.raises(ValueError):
            clamp_release_duration(bad)


# --------------------------------------------------------------------------- #
# SR-PAY-03: arm -> release interlock
# --------------------------------------------------------------------------- #
def test_release_requires_prior_arm():
    state, decision = evaluate_release(InterlockPolicy(), InterlockState(), now=100.0)
    assert decision.allowed is False
    assert decision.reason == "interlock"
    assert state.armed_at is None


def test_armed_release_is_allowed_and_consumed():
    armed = arm_release(now=100.0)
    assert armed.armed_at == 100.0
    state, decision = evaluate_release(InterlockPolicy(), armed, now=101.0)
    assert decision.allowed is True
    # The arm is consumed: an immediate second release is refused.
    state, decision = evaluate_release(InterlockPolicy(), state, now=101.1)
    assert decision.allowed is False
    assert decision.reason == "interlock"


def test_release_at_window_boundary_is_allowed():
    policy = InterlockPolicy(arm_window_s=10.0)
    _, decision = evaluate_release(policy, arm_release(now=100.0), now=110.0)
    assert decision.allowed is True


def test_expired_arm_is_refused_and_disarmed():
    policy = InterlockPolicy(arm_window_s=10.0)
    state, decision = evaluate_release(policy, arm_release(now=100.0), now=110.01)
    assert decision.allowed is False
    assert decision.reason == "interlock"
    assert state.armed_at is None


def test_clock_anomaly_is_refused_and_disarmed():
    # now earlier than the arm timestamp: refuse and drop the arm.
    state, decision = evaluate_release(InterlockPolicy(), arm_release(now=100.0), now=99.0)
    assert decision.allowed is False
    assert decision.reason == "interlock"
    assert state.armed_at is None
