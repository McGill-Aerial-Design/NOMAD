# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Isaac Sim simulation management API routes.

Provides endpoints for controlling the ZED Isaac Sim container from
Edge Core. These routes are only active when ISAAC_SIM_MODE=true,
enabling full drone functionality testing on a dev workstation
without Jetson hardware.

The simulation publishes the same ROS2 topics as the real ZED wrapper,
so downstream consumers (ros_http_bridge, nvblox, Mission Planner)
work identically in sim and on the real drone.
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

logger = logging.getLogger("edge_core.api.isaac_sim")

_ISAAC_SIM_MODE = os.environ.get("ISAAC_SIM_MODE", "").strip().lower() in ("1", "true", "yes", "on")


class IsaacSimModule(BaseModule):
    """Manages the ZED Isaac Sim container and sim-specific services.

    Only active when ISAAC_SIM_MODE=true. When enabled, replaces the
    hardware-only Isaac routes with sim-aware equivalents that control
    the Isaac Sim container via Docker instead of systemd.
    """

    metadata = ModuleMetadata(
        name="isaac_sim",
        version="1.0.0",
        description="Controls Isaac Sim container for hardware-free drone simulation",
        enable_flag="ISAAC_SIM_MODE",
        enabled_by_default=False,
    )

    def configure(self, ctx: AppContext) -> None:
        self.state_mgr = ctx.require_service("state_manager")
        self._container_name = os.environ.get("ISAAC_SIM_CONTAINER_NAME", "nomad_isaac_sim")
        self._image_name = os.environ.get("ISAAC_SIM_IMAGE_NAME", "nomad-isaac-sim:latest")

    def register_routes(self, app: Any) -> None:
        router = APIRouter(prefix="/api/sim", tags=["Isaac Sim"])
        container_name = self._container_name

        @router.get("/status")
        async def get_sim_status(request: Request):
            """Get Isaac Sim container status and simulation state."""
            now = time.time()
            container_running = False
            zed_publishing = False
            bridge_running = False

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
                pass

            if container_running:
                try:
                    r = await asyncio.to_thread(
                        subprocess.run,
                        [
                            "docker",
                            "exec",
                            container_name,
                            "bash",
                            "-c",
                            "source /opt/ros/humble/setup.bash 2>/dev/null; "
                            "source /opt/zed_ros_ws/install/setup.bash 2>/dev/null; "
                            "ros2 topic info --no-daemon /zed/zed_node/odom 2>/dev/null",
                        ],
                        capture_output=True,
                        text=True,
                        timeout=5,
                    )
                    zed_publishing = "Publisher count: 1" in r.stdout or "Publisher count: 2" in r.stdout
                except Exception:
                    pass

                try:
                    r = await asyncio.to_thread(
                        subprocess.run,
                        ["docker", "exec", container_name, "pgrep", "-f", "ros_http_bridge"],
                        capture_output=True,
                        text=True,
                        timeout=3,
                    )
                    bridge_running = r.returncode == 0
                except Exception:
                    pass

            return {
                "sim_mode": True,
                "container_running": container_running,
                "container_name": container_name,
                "zed_publishing": zed_publishing,
                "bridge_running": bridge_running,
                "timestamp": now,
            }

        @router.post("/start")
        async def start_sim():
            """Start the Isaac Sim container."""
            try:
                await asyncio.to_thread(
                    subprocess.run,
                    ["docker", "start", container_name],
                    check=True,
                    capture_output=True,
                    timeout=15,
                )
                return {"success": True, "message": f"Container {container_name} started"}
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"Failed to start container: {e}")

        @router.post("/stop")
        async def stop_sim():
            """Stop the Isaac Sim container."""
            try:
                await asyncio.to_thread(
                    subprocess.run,
                    ["docker", "stop", container_name],
                    check=True,
                    capture_output=True,
                    timeout=15,
                )
                return {"success": True, "message": f"Container {container_name} stopped"}
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"Failed to stop container: {e}")

        @router.post("/zed/start")
        async def start_zed_sim():
            """Start the ZED simulation publisher inside the container."""
            try:
                env_args = []
                for var in (
                    "ZED_CAMERA_MODEL",
                    "ZED_CAMERA_NAME",
                    "ZED_GRAB_RESOLUTION",
                    "ZED_DEPTH_MODE",
                    "ISAAC_SIM_HEADLESS",
                    "ISAAC_SIM_WORLD",
                    "ISAAC_SIM_DRONE_START_X",
                    "ISAAC_SIM_DRONE_START_Y",
                    "ISAAC_SIM_DRONE_START_Z",
                    "ISAAC_SIM_DRONE_START_YAW",
                ):
                    val = os.environ.get(var, "")
                    if val:
                        env_args.extend(["-e", f"{var}={val}"])

                await asyncio.to_thread(
                    subprocess.run,
                    [
                        "docker",
                        "exec",
                        *env_args,
                        "-d",
                        container_name,
                        "bash",
                        "-c",
                        "source /opt/ros/humble/setup.bash 2>/dev/null; "
                        "source /opt/zed_ros_ws/install/setup.bash 2>/dev/null; "
                        "source /opt/isaac_ros_installed/setup.bash 2>/dev/null; "
                        "export PYTHONPATH=/opt/nomad/isaac_sim:${PYTHONPATH:-}; "
                        "python3 /opt/nomad/isaac_sim/zed_sim_publisher.py "
                        "> /tmp/zed_sim_publisher.log 2>&1",
                    ],
                    timeout=10,
                )
                return {"success": True, "message": "ZED sim publisher started"}
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"Failed to start ZED sim: {e}")

        @router.post("/zed/stop")
        async def stop_zed_sim():
            """Stop the ZED simulation publisher."""
            try:
                await asyncio.to_thread(
                    subprocess.run,
                    ["docker", "exec", container_name, "pkill", "-f", "zed_sim_publisher"],
                    capture_output=True,
                    timeout=5,
                )
                return {"success": True, "message": "ZED sim publisher stopped"}
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"Failed to stop ZED sim: {e}")

        @router.get("/pose")
        async def get_sim_pose():
            """Get the current simulated drone pose from VIO topic."""
            try:
                r = await asyncio.to_thread(
                    subprocess.run,
                    [
                        "docker",
                        "exec",
                        container_name,
                        "bash",
                        "-c",
                        "source /opt/ros/humble/setup.bash 2>/dev/null; "
                        "timeout 2 ros2 topic echo --once /zed/zed_node/odom "
                        "--field pose.pose 2>/dev/null",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                if r.returncode == 0 and r.stdout.strip():
                    return {"success": True, "pose": r.stdout.strip()}
                return {"success": False, "message": "No VIO data available"}
            except Exception as e:
                return {"success": False, "message": str(e)}

        app.include_router(router)
