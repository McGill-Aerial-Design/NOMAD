# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Isaac ROS container and C++ ROS adapter node management routes.

Controls the real Jetson Isaac ROS container (nvblox + ZED) and the
in-container nomad_vehicle_node process (the retired ROS-HTTP bridge's
replacement) via systemd.
"""

from __future__ import annotations

import asyncio
import logging
import os
import subprocess
import time
from typing import Any

from fastapi import APIRouter, HTTPException, Request

from edge_core.api_routes._isaac_sim import sim_perception_status
from edge_core.core import AppContext, BaseModule, ModuleMetadata

logger = logging.getLogger("edge_core.api.isaac")


def _container_name() -> str:
    return os.environ.get("ISAAC_CONTAINER_NAME", "nomad_isaac_ros")


def _docker_exec_pgrep(container: str, pattern: str, timeout_s: int = 5) -> bool | None:
    """Return process-match state inside container, or None on probe failure."""
    try:
        result = subprocess.run(
            ["docker", "exec", container, "pgrep", "-f", pattern],
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
        return result.returncode == 0
    except Exception:
        return None


def _container_is_running(cache: dict, now: float) -> bool:
    """Probe the container status, honouring the short-lived status cache."""
    if now - cache["timestamp"] < 3.0:
        return bool(cache["container_running"])
    try:
        result = subprocess.run(
            ["docker", "ps", "--filter", f"name={_container_name()}", "--format", "{{.Status}}"],
            capture_output=True,
            text=True,
            timeout=3,
        )
        running = bool(result.stdout.strip())
    except Exception:
        running = False
    cache["container_running"] = running
    cache["timestamp"] = now
    return running


async def _systemctl(unit: str, action: str, success_message: str, failure_prefix: str) -> dict:
    """Run one systemctl action on a NOMAD unit, raising on failure."""
    try:
        await asyncio.to_thread(
            subprocess.run,
            ["sudo", "-n", "systemctl", action, unit],
            check=True,
            timeout=10,
        )
        return {"success": True, "message": success_message}
    except Exception as error:
        raise HTTPException(status_code=500, detail=f"{failure_prefix}: {error}")


async def get_isaac_status(request: Request) -> dict:
    """Get detailed status of the container, nvblox, and the ROS adapter node."""
    cache = request.app.state.isaac_runtime_cache
    now = time.time()

    sim_status = sim_perception_status(request.app.state, now)
    if sim_status is not None:
        return sim_status

    container_running = _container_is_running(cache, now)
    nvblox_running = False
    vehicle_running = False
    if container_running:
        loop = asyncio.get_running_loop()
        nv_task = loop.run_in_executor(None, _docker_exec_pgrep, _container_name(), "nvblox_node", 3)
        node_task = loop.run_in_executor(None, _docker_exec_pgrep, _container_name(), "nomad_vehicle_node", 3)
        nvblox_running = await nv_task or False
        vehicle_running = await node_task or False

    return {
        "container_running": container_running,
        "container_name": _container_name(),
        "nvblox_running": nvblox_running,
        "vehicle_running": vehicle_running,
        "timestamp": now,
    }


async def start_isaac_container() -> dict:
    """Start the Isaac ROS container via systemd."""
    return await _systemctl(
        "nomad-isaac-ros-container.service", "start", "Container started", "Failed to start container"
    )


async def stop_isaac_container() -> dict:
    """Stop the Isaac ROS container."""
    return await _systemctl(
        "nomad-isaac-ros-container.service", "stop", "Container stopped", "Failed to stop container"
    )


async def start_ros_vehicle() -> dict:
    """Start the in-container C++ ROS adapter node via systemd."""
    return await _systemctl(
        "nomad-ros-vehicle.service", "start", "Adapter node started", "Failed to start adapter node"
    )


async def stop_ros_vehicle() -> dict:
    """Stop the in-container C++ ROS adapter node."""
    return await _systemctl("nomad-ros-vehicle.service", "stop", "Adapter node stopped", "Failed to stop adapter node")


class IsaacModule(BaseModule):
    """Pluggable router for Docker and ROS adapter node processes."""

    metadata = ModuleMetadata(
        name="isaac_mgmt",
        version="1.1.0",
        description="Controls Isaac ROS container start/stop and the C++ ROS adapter node lifecycle",
    )

    def configure(self, ctx: AppContext) -> None:
        self.state_mgr = ctx.require_service("state_manager")

    def register_routes(self, app: Any) -> None:
        router = APIRouter(prefix="/api/isaac", tags=["Isaac ROS"])
        router.add_api_route("/status", get_isaac_status, methods=["GET"])
        router.add_api_route("/start", start_isaac_container, methods=["POST"])
        router.add_api_route("/stop", stop_isaac_container, methods=["POST"])
        router.add_api_route("/bridge/start", start_ros_vehicle, methods=["POST"])
        router.add_api_route("/bridge/stop", stop_ros_vehicle, methods=["POST"])
        app.include_router(router)
