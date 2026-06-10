# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""The single "is this velocity command allowed right now?" entry point.

Tier SC. :func:`evaluate` composes the freshness/arm/mode gates
(:mod:`edge_core.safety.gates`) and the clamp envelope
(:mod:`edge_core.safety.limits`) into one pure decision, and — on accept —
returns the clamped setpoint already converted from ROS FLU body frame to
MAVLink FRD (BODY_OFFSET_NED), ready for the I/O adapter to transmit.

The gate **order and rejection messages are preserved verbatim** from the
historical ``MavlinkVelocityController.submit`` so this extraction is
behaviour-preserving.

Implements requirements SR-VEL-01..05, SR-VIO-01, SR-LNK-01.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from . import gates
from .limits import VelocityLimits


@dataclass(frozen=True)
class EnvelopePolicy:
    """Static configuration of the flight envelope (limits + gate thresholds)."""

    limits: VelocityLimits = field(default_factory=VelocityLimits)
    require_armed: bool = True
    require_guided: bool = True
    guided_mode: str = "GUIDED"
    heartbeat_timeout_s: float = 3.0
    vio_max_age_s: float = 1.0
    min_vio_confidence: float = 0.3


@dataclass(frozen=True)
class FlightConditions:
    """A snapshot of the dynamic state the envelope reasons about.

    The adapter builds this under its lock so :func:`evaluate` runs on a
    consistent, immutable view with no locking concerns of its own.
    """

    connected: bool
    armed: bool
    flight_mode: str
    last_heartbeat: float
    vio_confidence: float
    vio_healthy: bool
    vio_last_update: float


@dataclass(frozen=True)
class VelocityCommand:
    """A requested velocity in ROS REP-103 body frame (x fwd, y left, z up)."""

    vx: float
    vy: float
    vz: float
    yaw_rate: float


@dataclass(frozen=True)
class Decision:
    """Result of evaluating a command against the envelope.

    On accept: ``allowed`` is True and ``setpoint`` is the clamped FRD
    ``(vx, vy, vz, yaw_rate)`` to send. On reject: ``allowed`` is False,
    ``reason`` is a stable key ("link"/"armed"/"mode"/"vio"/"nonfinite") used
    for throttled warnings, and ``message`` is the operator-facing string.
    """

    allowed: bool
    reason: str | None = None
    message: str | None = None
    setpoint: tuple[float, float, float, float] | None = None


def evaluate(
    policy: EnvelopePolicy,
    conditions: FlightConditions,
    command: VelocityCommand,
    now: float,
) -> Decision:
    """Decide whether ``command`` may be sent given ``conditions`` at ``now``.

    Gates are checked in safety order; the first failure short-circuits with the
    safe default (reject, send nothing). Only a fully-gated, finite, clamped
    command yields ``allowed=True``.
    """
    if not conditions.connected or not gates.heartbeat_fresh(
        conditions.last_heartbeat, now, policy.heartbeat_timeout_s
    ):
        return Decision(False, "link", "No MAVLink heartbeat from flight controller - dropping cmd_vel")

    if policy.require_armed and not conditions.armed:
        return Decision(False, "armed", "Vehicle not armed - dropping cmd_vel")

    if policy.require_guided and conditions.flight_mode != policy.guided_mode:
        return Decision(False, "mode", f"Vehicle not in GUIDED ({conditions.flight_mode}) - dropping cmd_vel")

    if not gates.vio_ready(
        conditions.vio_healthy,
        conditions.vio_confidence,
        policy.min_vio_confidence,
        conditions.vio_last_update,
        now,
        policy.vio_max_age_s,
    ):
        return Decision(
            False,
            "vio",
            f"VIO unhealthy/stale (confidence={conditions.vio_confidence:.2f}) - dropping cmd_vel",
        )

    try:
        vx_c, vy_c, vz_c, yaw_c = policy.limits.clamp_command(command.vx, command.vy, command.vz, command.yaw_rate)
    except ValueError:
        return Decision(False, "nonfinite", "Non-finite cmd_vel value - dropping command")

    # ROS FLU -> MAVLink FRD (BODY_OFFSET_NED): negate y, z, yaw_rate.
    return Decision(True, setpoint=(vx_c, -vy_c, -vz_c, -yaw_c))
