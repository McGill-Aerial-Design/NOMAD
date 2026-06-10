# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Velocity/yaw clamps and finite-value checks for the safety-critical core.

Tier SC. Pure functions and immutable value objects only — no I/O, no threads,
no MAVLink. Extracted from ``ros_http_bridge.mavlink_velocity`` so the clamp
logic is exhaustively unit-testable in isolation.

Implements requirements SR-VEL-01..04 (see ``docs/safety/requirements.md``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Default safety velocity limits (m/s, rad/s). These mirror the historical
# constants on MavlinkVelocityController and are the single source of truth.
DEFAULT_MAX_VELOCITY_XY = 2.0
DEFAULT_MAX_VELOCITY_Z = 1.0
DEFAULT_MAX_YAW_RATE = 1.0


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp ``value`` to ``[lo, hi]``, rejecting non-finite input.

    Raises:
        ValueError: if ``value`` is NaN or +/-inf. The caller treats this as a
            command rejection (SR-VEL-03) rather than silently substituting a
            bound, because a non-finite setpoint signals upstream corruption.
    """
    if not math.isfinite(value):
        raise ValueError("velocity command values must be finite")
    return max(lo, min(hi, value))


@dataclass(frozen=True)
class VelocityLimits:
    """Immutable clamp envelope for a velocity setpoint."""

    max_velocity_xy: float = DEFAULT_MAX_VELOCITY_XY
    max_velocity_z: float = DEFAULT_MAX_VELOCITY_Z
    max_yaw_rate: float = DEFAULT_MAX_YAW_RATE

    def clamp_command(self, vx: float, vy: float, vz: float, yaw_rate: float) -> tuple[float, float, float, float]:
        """Clamp a full (vx, vy, vz, yaw_rate) command to the envelope.

        Raises ``ValueError`` (via :func:`clamp`) if any component is non-finite,
        so a single bad axis rejects the whole command.
        """
        return (
            clamp(vx, -self.max_velocity_xy, self.max_velocity_xy),
            clamp(vy, -self.max_velocity_xy, self.max_velocity_xy),
            clamp(vz, -self.max_velocity_z, self.max_velocity_z),
            clamp(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate),
        )
