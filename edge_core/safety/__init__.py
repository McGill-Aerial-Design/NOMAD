# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Safety-critical (Tier SC) decision core for NOMAD.

Dependency-light, pure-logic package: no FastAPI, no ROS, no pymavlink, no
threads or sockets. The I/O shells (``ros_http_bridge.mavlink_velocity``, the
ROS node, and the ``services/mavlink/commands.py`` command surface) are thin
adapters that *ask* this package whether a command is permitted and what to
emit. Keeping the decisions here pure makes them exhaustively unit-testable and
fault-injectable.

See ``docs/safety/`` for the partition, hazards, requirements, and traceability
that this package implements.
"""

from __future__ import annotations

from .envelope import Decision, EnvelopePolicy, FlightConditions, VelocityCommand, evaluate
from .gates import heartbeat_fresh, heartbeat_from_vehicle, vio_fresh, vio_ready
from .geofence import FencePolicy, Point, evaluate_position
from .limits import (
    DEFAULT_MAX_VELOCITY_XY,
    DEFAULT_MAX_VELOCITY_Z,
    DEFAULT_MAX_YAW_RATE,
    VelocityLimits,
    clamp,
)
from .payload import (
    MAX_PWM_US,
    MAX_RELEASE_S,
    MAX_SERVO_CHANNEL,
    MIN_PWM_US,
    MIN_RELEASE_S,
    MIN_SERVO_CHANNEL,
    InterlockPolicy,
    InterlockState,
    arm_release,
    clamp_release_duration,
    evaluate_release,
    validate_servo_command,
)
from .watchdog import DEFAULT_COMMAND_TIMEOUT_S, WatchdogResult, watchdog_decision

__all__ = [
    "DEFAULT_COMMAND_TIMEOUT_S",
    "DEFAULT_MAX_VELOCITY_XY",
    "DEFAULT_MAX_VELOCITY_Z",
    "DEFAULT_MAX_YAW_RATE",
    "MAX_PWM_US",
    "MAX_RELEASE_S",
    "MAX_SERVO_CHANNEL",
    "MIN_PWM_US",
    "MIN_RELEASE_S",
    "MIN_SERVO_CHANNEL",
    "Decision",
    "EnvelopePolicy",
    "FencePolicy",
    "FlightConditions",
    "InterlockPolicy",
    "InterlockState",
    "Point",
    "VelocityCommand",
    "VelocityLimits",
    "WatchdogResult",
    "arm_release",
    "clamp",
    "clamp_release_duration",
    "evaluate",
    "evaluate_position",
    "evaluate_release",
    "heartbeat_fresh",
    "heartbeat_from_vehicle",
    "validate_servo_command",
    "vio_fresh",
    "vio_ready",
    "watchdog_decision",
]
