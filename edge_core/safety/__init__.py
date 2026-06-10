# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Safety-critical (Tier SC) decision core for NOMAD.

Dependency-light, pure-logic package: no FastAPI, no ROS, no pymavlink, no
threads or sockets. The I/O shells (``ros_http_bridge.mavlink_velocity`` and the
ROS node) are thin adapters that *ask* this package whether a command is
permitted and what to emit. Keeping the decisions here pure makes them
exhaustively unit-testable and fault-injectable.

See ``docs/safety/`` for the partition, hazards, requirements, and traceability
that this package implements.
"""

from __future__ import annotations

from .envelope import Decision, EnvelopePolicy, FlightConditions, VelocityCommand, evaluate
from .limits import (
    DEFAULT_MAX_VELOCITY_XY,
    DEFAULT_MAX_VELOCITY_Z,
    DEFAULT_MAX_YAW_RATE,
    VelocityLimits,
    clamp,
)
from .watchdog import DEFAULT_COMMAND_TIMEOUT_S, WatchdogResult, watchdog_decision

__all__ = [
    "Decision",
    "EnvelopePolicy",
    "FlightConditions",
    "VelocityCommand",
    "evaluate",
    "VelocityLimits",
    "clamp",
    "DEFAULT_MAX_VELOCITY_XY",
    "DEFAULT_MAX_VELOCITY_Z",
    "DEFAULT_MAX_YAW_RATE",
    "WatchdogResult",
    "watchdog_decision",
    "DEFAULT_COMMAND_TIMEOUT_S",
]
