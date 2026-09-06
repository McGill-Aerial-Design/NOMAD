# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
import asyncio
import os
import time

from fastapi import Request

from edge_core.services.service_probes import (
    probe_mavlink_router,
    probe_mediamtx,
    probe_novnc,
)

try:
    import msgpack
except ImportError:  # pragma: no cover - optional Jetson dependency
    msgpack = None


_ISAAC_CONTAINER_NAME = os.environ.get("ISAAC_CONTAINER_NAME", "nomad_isaac_ros")

# TTL cache for the subprocess probe results (systemctl/pgrep). Mission
# Planner polls /api/services/status at ~1-2 Hz, which would otherwise fan
# out to ~10 subprocesses per poll and thrash the OS process table.
# The cache lives on app.state so separate apps stay isolated.
_PROC_CACHE_TTL_S = 2.0


def _probe_isaac_container() -> dict:
    """Check if the Isaac ROS container is running (blocking)."""
    import subprocess

    try:
        r = subprocess.run(
            ["docker", "ps", "--filter", f"name={_ISAAC_CONTAINER_NAME}", "--format", "{{.Status}}"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        return {"container_running": bool(r.stdout.strip())}
    except Exception:
        return {"container_running": False}


def _blocking_proc_checks() -> tuple:
    """Collect service states that require blocking subprocesses.

    Must run on a worker thread — never on the event loop.
    """
    return (
        probe_mavlink_router(),
        probe_mediamtx(),
        probe_novnc(),
        _probe_isaac_container(),
    )


async def _get_cached_proc_services(app) -> tuple[dict, dict]:
    """Return (proc_services, runtime_state), refreshing the TTL cache."""
    if not hasattr(app.state, "services_proc_cache"):
        app.state.services_proc_cache = {"ts": 0.0, "data": None, "lock": asyncio.Lock()}
    cache = app.state.services_proc_cache

    now = time.time()
    if cache["data"] is not None and (now - cache["ts"]) < _PROC_CACHE_TTL_S:
        return cache["data"]

    async with cache["lock"]:
        now = time.time()
        if cache["data"] is not None and (now - cache["ts"]) < _PROC_CACHE_TTL_S:
            return cache["data"]
        loop = asyncio.get_running_loop()
        mavlink, mediamtx, novnc, runtime_state = await loop.run_in_executor(None, _blocking_proc_checks)
        cache["data"] = (
            {
                "mavlink_router": mavlink,
                "mediamtx": mediamtx,
                "novnc": novnc,
            },
            runtime_state,
        )
        cache["ts"] = time.time()
    return cache["data"]


def _add_vio_status(services: dict, request: Request) -> None:
    """VIO status — provided by modules that register VIO state."""
    external_vio_state = getattr(request.app.state, "external_vio_state", None)
    if external_vio_state:
        services["vio"] = {
            "status": "active",
            "running": True,
            "source": external_vio_state.get("source", "external"),
            "confidence": external_vio_state.get("confidence", 0),
        }
    else:
        services["vio"] = {
            "status": "not_initialized",
            "running": False,
        }


def register_services_routes(app, ctx) -> None:
    @app.get("/api/services/status", tags=["System"])
    async def services_status(request: Request):
        """
        Get status of all NOMAD services.

        Returns status of:
        - mavlink-router: MAVLink routing to the flight controller
        - mediamtx: RTSP video server
        - novnc: Browser-based remote desktop service
        - edge_core: This API service (always running if you see this)
        - isaac_ros: Isaac ROS bridge status
        - vio: VIO pipeline status
        """
        proc_services, runtime_state = await _get_cached_proc_services(request.app)
        services = dict(proc_services)
        container_running = runtime_state["container_running"]

        # Edge Core is always running (we're responding)
        services["edge_core"] = {"status": "active", "running": True}

        # Isaac ROS status (container probe only; no in-process bridge exists)
        services["isaac_ros"] = {
            "status": "active" if container_running else "not_initialized",
            "running": container_running,
            "container_running": container_running,
        }
        services["isaac_ros_container"] = {
            "status": "running" if container_running else "not_running",
            "running": container_running,
        }

        _add_vio_status(services, request)
        return services
