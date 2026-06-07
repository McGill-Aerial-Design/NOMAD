# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Isaac ROS Container and ros_http_bridge management routes.

Supports both the real Jetson Isaac ROS container and the local
Isaac Sim container for hardware-free simulation.
"""

from __future__ import annotations

import asyncio
import logging
import os
import subprocess
import time
from typing import Any

from fastapi import APIRouter, HTTPException, Request

from edge_core.core import AppContext, BaseModule, ModuleMetadata

logger = logging.getLogger("edge_core.api.isaac")

_ISAAC_SIM_MODE = os.environ.get("ISAAC_SIM_MODE", "").strip().lower() in ("1", "true", "yes", "on")


class IsaacModule(BaseModule):
    """Pluggable router for Docker and ROS bridge processes."""

    metadata = ModuleMetadata(
        name="isaac_mgmt",
        version="1.0.0",
        description="Controls Isaac ROS container start/stop and ROS-HTTP bridge lifecycles",
    )

    def configure(self, ctx: AppContext) -> None:
        self.state_mgr = ctx.require_service("state_manager")
        self.cmd_success = ctx.get_service("docker_exec_bash_success")
        self._container_name = (
            os.environ.get("ISAAC_SIM_CONTAINER_NAME", "nomad_isaac_sim")
            if _ISAAC_SIM_MODE
            else os.environ.get("ISAAC_CONTAINER_NAME", "nomad_isaac_ros")
        )
        self._sim_mode = _ISAAC_SIM_MODE

    def register_routes(self, app: Any) -> None:
        router = APIRouter(prefix="/api/isaac", tags=["Isaac ROS"])
        container_name = self._container_name

        @router.get("/status")
        async def get_isaac_status(request: Request):
            """Get detailed status of the container, nvblox, and bridge."""
            cache = request.app.state.isaac_runtime_cache
            now = time.time()

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

            if container_running and self.cmd_success:
                loop = asyncio.get_running_loop()
                nv_task = loop.run_in_executor(None, self.cmd_success, container_name, "nvblox_node", 3)
                br_task = loop.run_in_executor(None, self.cmd_success, container_name, "ros_http_bridge\\.py", 3)
                nvblox_running = await nv_task or False
                bridge_running = await br_task or False

            return {
                "container_running": container_running,
                "container_name": container_name,
                "sim_mode": self._sim_mode,
                "nvblox_running": nvblox_running,
                "bridge_running": bridge_running,
                "timestamp": now,
            }

        @router.post("/start")
        async def start_isaac_container():
            """Start the Isaac container via systemd or docker compose."""
            if self._sim_mode:
                try:
                    await asyncio.to_thread(
                        subprocess.run,
                        ["docker", "start", container_name],
                        check=True,
                        capture_output=True,
                        timeout=15,
                    )
                    return {"success": True, "message": f"Sim container {container_name} started"}
                except Exception as e:
                    raise HTTPException(status_code=500, detail=f"Failed to start sim container: {e}")
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
            """Stop the Isaac container."""
            if self._sim_mode:
                try:
                    await asyncio.to_thread(
                        subprocess.run,
                        ["docker", "stop", container_name],
                        check=True,
                        capture_output=True,
                        timeout=15,
                    )
                    return {"success": True, "message": f"Sim container {container_name} stopped"}
                except Exception as e:
                    raise HTTPException(status_code=500, detail=f"Failed to stop sim container: {e}")
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
            """Start the in-container ros_http_bridge process."""
            if self._sim_mode:
                try:
                    await asyncio.to_thread(
                        subprocess.run,
                        [
                            "docker",
                            "exec",
                            "-d",
                            container_name,
                            "bash",
                            "-c",
                            "source /opt/ros/humble/setup.bash 2>/dev/null; "
                            "source /opt/zed_ros_ws/install/setup.bash 2>/dev/null; "
                            "source /opt/isaac_ros_installed/setup.bash 2>/dev/null; "
                            "export PYTHONPATH=/workspaces/nomad-sim/edge_core:/opt/nomad:${PYTHONPATH:-}; "
                            "python3 -m edge_core.ros_http_bridge.main "
                            "--host localhost --port 8000 "
                            "--rate ${ROS_HTTP_BRIDGE_RATE:-5} "
                            "--vio-topic ${ROS_HTTP_BRIDGE_VIO_TOPIC:-/zed/zed_node/odom} "
                            "--mag-topic ${ROS_HTTP_BRIDGE_MAG_TOPIC:-/zed/zed_node/imu/mag} "
                            "--disable-mesh --disable-detections "
                            "> /tmp/ros_http_bridge.log 2>&1",
                        ],
                        timeout=10,
                    )
                    return {"success": True, "message": "Sim bridge started"}
                except Exception as e:
                    raise HTTPException(status_code=500, detail=f"Failed to start sim bridge: {e}")
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
            if self._sim_mode:
                try:
                    await asyncio.to_thread(
                        subprocess.run,
                        ["docker", "exec", container_name, "pkill", "-f", "ros_http_bridge"],
                        capture_output=True,
                        timeout=5,
                    )
                    return {"success": True, "message": "Sim bridge stopped"}
                except Exception as e:
                    raise HTTPException(status_code=500, detail=f"Failed to stop sim bridge: {e}")
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
