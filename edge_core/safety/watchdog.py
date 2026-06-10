# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Command-timeout / VIO-stale failsafe decision for the safety-critical core.

Tier SC. The watchdog itself is a pure decision: given the controller's current
state at ``now``, decide whether to command zero velocity and why. The threaded
loop that *acts* on the decision lives in the I/O adapter, not here.

Implements requirements SR-LNK-02 and SR-VIO-02 (see
``docs/safety/requirements.md``).
"""

from __future__ import annotations

from dataclasses import dataclass

# Default: stop the vehicle if no fresh command arrives within this window.
DEFAULT_COMMAND_TIMEOUT_S = 0.5


@dataclass(frozen=True)
class WatchdogResult:
    """Outcome of one watchdog evaluation."""

    stop: bool
    reason: str | None = None  # "command timeout" | "VIO stale" | None


def watchdog_decision(
    *,
    active: bool,
    last_command_time: float,
    now: float,
    command_timeout_s: float,
    vio_is_fresh: bool,
) -> WatchdogResult:
    """Decide whether to zero velocity.

    Only fires while ``active`` (i.e. the controller has sent at least one
    setpoint and has not already been stopped). A command-timeout takes
    precedence over a VIO-stale reason when both hold, matching the historical
    behaviour. When it does not fire, ``stop`` is False and ``reason`` is None.
    """
    if not active:
        return WatchdogResult(stop=False)
    stale_cmd = (now - last_command_time) > command_timeout_s
    vio_stale = not vio_is_fresh
    if stale_cmd or vio_stale:
        return WatchdogResult(stop=True, reason="command timeout" if stale_cmd else "VIO stale")
    return WatchdogResult(stop=False)
