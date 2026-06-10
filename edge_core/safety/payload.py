# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Payload actuation decisions for the safety-critical core.

Tier SC. Pure validation and interlock logic for the servo/relay payload path
(requirements SR-PAY-01, SR-PAY-02, SR-PAY-03; hazard H-06). The I/O adapter is
``modules/payload/servo.py``: it asks this module whether a command is allowed,
then transmits over MAVLink and owns the de-energize-in-``finally`` guarantee.

The release interlock is a tiny explicit state machine: ``arm_release`` opens a
short window; ``evaluate_release`` permits exactly one release inside it and
disarms on every outcome (success, expiry, or clock anomaly), so a release
always requires a fresh, deliberate arm.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .envelope import Decision

# Payload limits — the single source of truth for the servo/relay envelope.
MIN_SERVO_CHANNEL = 1
MAX_SERVO_CHANNEL = 16
MIN_PWM_US = 500
MAX_PWM_US = 2500
MIN_RELEASE_S = 0.05
MAX_RELEASE_S = 5.0
DEFAULT_ARM_WINDOW_S = 10.0


def validate_servo_command(channel: int, pwm_us: int) -> Decision:
    """Validate a raw servo command (SR-PAY-01): channel 1-16, PWM 500-2500 us."""
    if channel < MIN_SERVO_CHANNEL or channel > MAX_SERVO_CHANNEL:
        return Decision(False, "channel", f"Invalid Cube servo channel: {channel}")
    if pwm_us < MIN_PWM_US or pwm_us > MAX_PWM_US:
        return Decision(False, "pwm", f"Invalid Cube servo PWM: {pwm_us}")
    return Decision(True)


def clamp_release_duration(duration_s: float) -> float:
    """Clamp a pump-on duration to [0.05, 5.0] s (SR-PAY-02).

    Raises:
        ValueError: if ``duration_s`` is NaN or +/-inf. The caller treats this
            as a command rejection rather than energizing the pump for an
            arbitrary time.
    """
    if not math.isfinite(duration_s):
        raise ValueError("release duration must be finite")
    return max(MIN_RELEASE_S, min(duration_s, MAX_RELEASE_S))


@dataclass(frozen=True)
class InterlockPolicy:
    """How long an arm stays valid before a release is refused again."""

    arm_window_s: float = DEFAULT_ARM_WINDOW_S


@dataclass(frozen=True)
class InterlockState:
    """Interlock memory: ``armed_at`` is the arm time, or None when safe."""

    armed_at: float | None = None


def arm_release(now: float) -> InterlockState:
    """Open the release window at ``now`` (monotonic seconds)."""
    return InterlockState(armed_at=now)


def evaluate_release(policy: InterlockPolicy, state: InterlockState, now: float) -> tuple[InterlockState, Decision]:
    """Decide whether a release may fire at ``now`` (SR-PAY-03).

    Returns the next interlock state and the decision. Every path disarms:
    a permitted release consumes the arm, and any anomaly (never armed, window
    expired, clock moved backwards) refuses and leaves the interlock safe.
    """
    disarmed = InterlockState()
    if state.armed_at is None:
        return disarmed, Decision(False, "interlock", "Payload release requires arming first - rejected")
    if now < state.armed_at:
        return disarmed, Decision(False, "interlock", "Payload arm timestamp is in the future - rejected, rearm")
    if now - state.armed_at > policy.arm_window_s:
        return disarmed, Decision(False, "interlock", "Payload arm window expired - rejected, rearm")
    return disarmed, Decision(True)
