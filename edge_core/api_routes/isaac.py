# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Isaac ROS container and ros_http_bridge management routes.

Controls the real Jetson Isaac ROS container (nvblox + ZED) and the in-container
ros_http_bridge process via systemd.
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


class IsaacModule(BaseModule):
    """Pluggable router for Docker and ROS bridge processes."""

    metadata = ModuleMetadata(
        name="isaac_mgmt",
        version="1.0.0",
        description="Controls Isaac ROS container start/stop and ROS-HTTP bridge lifecycles",
    )

    def configure(self, ctx: AppContext) -> None:
        self.state_mgr = ctx.require_service("state_manager")
        self.cmd_success = _docker_exec_pgrep
        self._container_name = os.environ.get("ISAAC_CONTAINER_NAME", "nomad_isaac_ros")

    def register_routes(self, app: Any) -> None:
        router = APIRouter(prefix="/api/isaac", tags=["Isaac ROS"])
        container_name = self._container_name

        @router.get("/status")
        async def get_isaac_status(request: Request):
            """Get detailed status of the container, nvblox, and bridge."""
            cache = request.app.state.isaac_runtime_cache
            now = time.time()

            sim_status = sim_perception_status(request.app.state, now)
            if sim_status is not None:
                return sim_status

            if now - cache["timestamp"] < 3.0:
                container_running = cache["container_running"]
            else:
                try:
                    r = await asyncio.to_thread(
                        subprocess.run,
                        ["docker", "ps", "--filter", f"name={container_name}", "--format", "{{.Status}}"],
                        capture_output=True,
                        text=True,
                        timeout=3,
                    )
                    container_running = bool(r.stdout.strip())
                except Exception:
                    container_running = False
                cache["container_running"] = container_running
                cache["timestamp"] = now

            nvblox_running = False
            bridge_running = False

            if container_running:
                loop = asyncio.get_running_loop()
                nv_task = loop.run_in_executor(None, self.cmd_success, container_name, "nvblox_node", 3)
                br_task = loop.run_in_executor(None, self.cmd_success, container_name, "ros_http_bridge\\.py", 3)
                nvblox_running = await nv_task or False
                bridge_running = await br_task or False

            return {
                "container_running": container_running,
                "container_name": container_name,
                "nvblox_running": nvblox_running,
                "bridge_running": bridge_running,
                "timestamp": now,
            }

        @router.post("/start")
        async def start_isaac_container():
            """Start the Isaac ROS container via systemd."""
            try:
                await asyncio.to_thread(
                    subprocess.run,
                    ["sudo", "-n", "systemctl", "start", "nomad-isaac-ros-container.service"],
                    check=True,
                    timeout=10,
                )
                return {"success": True, "message": "Container started"}
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"Failed to start container: {e}")

        @router.post("/stop")
        async def stop_isaac_container():
            """Stop the Isaac ROS container."""
            try:
                await asyncio.to_thread(
                    subprocess.run,
                    ["sudo", "-n", "systemctl", "stop", "nomad-isaac-ros-container.service"],
                    check=True,
                    timeout=10,
                )
                return {"success": True, "message": "Container stopped"}
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"Failed to stop container: {e}")

        @router.post("/bridge/start")
        async def start_ros_bridge():
            """Start the in-container ros_http_bridge process via systemd."""
            try:
                await asyncio.to_thread(
                    subprocess.run,
                    ["sudo", "-n", "systemctl", "start", "nomad-ros-http-bridge.service"],
                    check=True,
                    timeout=10,
                )
                return {"success": True, "message": "Bridge started"}
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"Failed to start bridge: {e}")

        @router.post("/bridge/stop")
        async def stop_ros_bridge():
            """Stop the in-container ros_http_bridge."""
            try:
                await asyncio.to_thread(
                    subprocess.run,
                    ["sudo", "-n", "systemctl", "stop", "nomad-ros-http-bridge.service"],
                    check=True,
                    timeout=10,
                )
                return {"success": True, "message": "Bridge stopped"}
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"Failed to stop bridge: {e}")

        app.include_router(router)
