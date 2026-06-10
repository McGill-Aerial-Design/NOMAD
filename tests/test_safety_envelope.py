# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Unit + fault-injection tests for edge_core.safety.envelope.

Covers the full gate ordering and every rejection branch (SR-VEL-01..05,
SR-VIO-01, SR-LNK-01). This is the failure-mode / abnormal-range suite the
safety case (docs/safety/hazards.md) calls for.
"""

from __future__ import annotations

import math

import pytest

from edge_core.safety.envelope import (
    EnvelopePolicy,
    FlightConditions,
    VelocityCommand,
    evaluate,
)

NOW = 1000.0


def _healthy_conditions(**overrides) -> FlightConditions:
    base = dict(
        connected=True,
        armed=True,
        flight_mode="GUIDED",
        last_heartbeat=NOW,
        vio_confidence=1.0,
        vio_healthy=True,
        vio_last_update=NOW,
    )
    base.update(overrides)
    return FlightConditions(**base)


def _evaluate(command=VelocityCommand(1.0, 0.0, 0.0, 0.0), policy=None, **cond_overrides):
    policy = policy or EnvelopePolicy()
    return evaluate(policy, _healthy_conditions(**cond_overrides), command, NOW)


# -- happy path -------------------------------------------------------------


def test_allows_and_frame_converts_when_all_gates_pass():
    decision = _evaluate(VelocityCommand(1.0, 1.5, 0.5, 0.4))
    assert decision.allowed is True
    assert decision.reason is None
    # FLU -> FRD: negate y, z, yaw_rate.
    assert decision.setpoint == (1.0, -1.5, -0.5, -0.4)


def test_allows_and_clamps_before_converting():
    decision = _evaluate(VelocityCommand(99.0, 99.0, 99.0, 99.0))
    assert decision.allowed is True
    # XY clamp 2.0, Z clamp 1.0, yaw clamp 1.0, then sign flip on y/z/yaw.
    assert decision.setpoint == (2.0, -2.0, -1.0, -1.0)


# -- rejection branches (fault injection) -----------------------------------


def test_rejects_when_disconnected():
    d = _evaluate(connected=False)
    assert d.allowed is False
    assert d.reason == "link"
    assert d.setpoint is None


def test_rejects_when_no_heartbeat_ever():
    d = _evaluate(last_heartbeat=0.0)
    assert d.allowed is False
    assert d.reason == "link"


def test_rejects_when_heartbeat_stale():
    d = _evaluate(last_heartbeat=NOW - 5.0)  # default timeout 3.0s
    assert d.reason == "link"


def test_rejects_when_not_armed():
    d = _evaluate(armed=False)
    assert d.allowed is False
    assert d.reason == "armed"


def test_rejects_when_not_guided():
    d = _evaluate(flight_mode="LOITER")
    assert d.reason == "mode"
    assert "LOITER" in (d.message or "")


def test_rejects_when_vio_unhealthy():
    d = _evaluate(vio_healthy=False)
    assert d.reason == "vio"


def test_rejects_when_vio_low_confidence():
    d = _evaluate(vio_confidence=0.1)  # default min 0.3
    assert d.reason == "vio"


def test_rejects_when_vio_stale():
    d = _evaluate(vio_last_update=NOW - 5.0)  # default max_age 1.0s
    assert d.reason == "vio"


@pytest.mark.parametrize(
    "bad",
    [
        VelocityCommand(math.nan, 0.0, 0.0, 0.0),
        VelocityCommand(0.0, math.inf, 0.0, 0.0),
        VelocityCommand(0.0, 0.0, -math.inf, 0.0),
        VelocityCommand(0.0, 0.0, 0.0, math.nan),
    ],
)
def test_rejects_nonfinite_command(bad):
    d = _evaluate(bad)
    assert d.allowed is False
    assert d.reason == "nonfinite"
    assert d.setpoint is None


# -- gate ordering ----------------------------------------------------------


def test_link_gate_precedes_armed_gate():
    # Disconnected AND disarmed -> link reported first.
    d = _evaluate(connected=False, armed=False)
    assert d.reason == "link"


def test_armed_gate_precedes_mode_gate():
    d = _evaluate(armed=False, flight_mode="LOITER")
    assert d.reason == "armed"


def test_mode_gate_precedes_vio_gate():
    d = _evaluate(flight_mode="LOITER", vio_healthy=False)
    assert d.reason == "mode"


def test_vio_gate_precedes_nonfinite_check():
    # A non-finite command with stale VIO is rejected on VIO first
    # (gates run before clamping).
    d = _evaluate(VelocityCommand(math.nan, 0.0, 0.0, 0.0), vio_healthy=False)
    assert d.reason == "vio"


# -- policy toggles ---------------------------------------------------------


def test_require_armed_false_allows_disarmed():
    policy = EnvelopePolicy(require_armed=False)
    d = _evaluate(policy=policy, armed=False)
    assert d.allowed is True


def test_require_guided_false_allows_other_mode():
    policy = EnvelopePolicy(require_guided=False)
    d = _evaluate(policy=policy, flight_mode="LOITER")
    assert d.allowed is True
