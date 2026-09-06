# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Simulation-only perception status for the Isaac route.

This is the single sim-aware branch behind ``GET /api/isaac/status``. It is
gated entirely by ``NOMAD_SIM_MODE``, which is off on the real Jetson, so none
of this executes on hardware — it exists purely so the Gazebo / ROS sim stacks
can report perception health through the same endpoint the device uses.

Kept in its own module (rather than inline in ``isaac.py``) so the boundary
between device behaviour and sim scaffolding is a file boundary, not a buried
``if``.
"""

from __future__ import annotations

import os
from typing import Any

from edge_core.env import env_bool


def _configured_sim_runtime() -> str | None:
    runtime = os.environ.get("NOMAD_SIM_PERCEPTION_RUNTIME", "").strip()
    if runtime.lower() in {"", "0", "false", "none", "off"}:
        return None
    return runtime


def _sim_status(
    runtime: str,
    source: str,
    now: float,
    vio_healthy: bool,
    vio_age_s: float | None,
) -> dict:
    """Build the synthetic status payload for a configured sim runtime."""
    return {
        "container_running": True,
        "container_name": runtime,
        "nvblox_running": False,
        "vehicle_running": vio_healthy,
        "runtime": runtime,
        "simulated": True,
        "source": source,
        "vio_healthy": vio_healthy,
        "vio_age_s": vio_age_s,
        "timestamp": now,
    }


def sim_perception_status(app_state: Any, now: float) -> dict | None:
    """Return a synthetic Isaac status when running under the sim stacks.

    Returns ``None`` when not in sim mode (so the caller falls through to the
    real docker probe) or when there is no configured runtime and no sim VIO.
    """
    if not env_bool("NOMAD_SIM_MODE"):
        return None

    runtime = _configured_sim_runtime()
    vio = getattr(app_state, "external_vio_state", None)
    if not vio:
        if runtime:
            return _sim_status(runtime, "unavailable", now, False, None)
        return None

    timestamp = float(vio.get("timestamp", 0.0))
    age_s = max(0.0, now - timestamp) if timestamp > 0 else None
    # No (or zero) timestamp means we cannot prove the adapter is publishing, so
    # treat it the same as a stale update rather than reporting it healthy.
    stale = age_s is None or age_s > 5.0
    source = vio.get("source", "external")
    if runtime is None and source in {"gazebo", "ros_sim", "zed_sim"}:
        runtime = source

    if stale and runtime:
        return _sim_status(runtime, source, now, False, age_s)
    if stale:
        return None

    return _sim_status(runtime or "ros_sim", source, now, True, age_s)
