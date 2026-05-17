import asyncio
import json
import os
import re
import shlex
import shutil
import subprocess
import time
from datetime import datetime, timezone
from typing import Any, Optional

from fastapi import HTTPException, Query, Request, WebSocket
from fastapi.encoders import jsonable_encoder
from fastapi.responses import FileResponse
from fastapi.websockets import WebSocketDisconnect
from pydantic import BaseModel
from starlette.responses import JSONResponse

from ..api_models import (
    COMMAND_WHITELIST,
    MSGPACK_AVAILABLE,
    Task1CapturesList,
    Task2HitRequest,
    TerminalCommandRequest,
    TerminalCommandResponse,
    TerminalExecRequest,
    VIOAreaLoadRequest,
    VIOAreaSaveRequest,
    VIOUpdateRequest,
    NavPositionRequest,
    NavVelocityRequest,
)

try:
    import msgpack
except ImportError:  # pragma: no cover - optional Jetson dependency
    msgpack = None

def register_isaac_routes(app, ctx) -> None:
    logger = ctx.logger
    _probe_isaac_runtime_state = ctx.probe_isaac_runtime_state

    # ==================== Isaac ROS Bridge Endpoints ====================

    @app.get("/api/isaac/status", tags=["Isaac ROS"])
    async def isaac_status(request: Request):
        """
        Get Isaac ROS bridge status.

        Returns information about the perception backend,
        VIO state, and exclusion map status.
        """
        runtime_state = await asyncio.to_thread(
            _probe_isaac_runtime_state,
            force_refresh=True,
        )
        container_running = runtime_state["container_running"]
        nvblox_running = runtime_state["nvblox_running"]
        bridge_running = runtime_state["bridge_running"]

        isaac_bridge = request.app.state.isaac_bridge
        if not isaac_bridge:
            # No Python-side bridge, but the external ROS-HTTP bridge may be active
            # NOTE: available requires container + bridge, NOT nvblox (supports ZED-only operation)
            external_bridge_active = container_running and bridge_running
            return {
                "available": external_bridge_active,
                "backend": "ros_http_bridge"
                if external_bridge_active
                else "not_initialized",
                "container_running": container_running,
                "nvblox_running": nvblox_running,
                "bridge_running": bridge_running,
                "message": "Active via external ROS-HTTP bridge"
                if external_bridge_active
                else "Isaac ROS not running",
            }

        return {
            "available": True,
            "container_running": container_running,
            "nvblox_running": nvblox_running,
            "bridge_running": bridge_running,
            **isaac_bridge.get_status(),
        }

    # Per-service systemd units that make up the Isaac ROS stack. Listed in
    # dependency order — start in this order, stop in reverse.
    ISAAC_STACK_UNITS = (
        "nomad-isaac-ros-container.service",
        "nomad-zed-wrapper.service",
        "nomad-ros-http-bridge.service",
    )

    def _systemctl(verb: str, units: tuple[str, ...]) -> dict:
        """Call `sudo -n systemctl <verb> <unit>...` and surface the result."""
        cmd = ["sudo", "-n", "systemctl", verb, *units]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            return {
                "success": proc.returncode == 0,
                "stdout": proc.stdout,
                "stderr": proc.stderr,
                "command": " ".join(cmd),
            }
        except subprocess.TimeoutExpired:
            return {"success": False, "error": "systemctl timed out", "command": " ".join(cmd)}
        except Exception as exc:
            return {"success": False, "error": str(exc), "command": " ".join(cmd)}

    @app.post("/api/isaac/start", tags=["Isaac ROS"])
    async def isaac_start():
        """
        Start the Isaac ROS stack (container + ZED wrapper + ROS-HTTP bridge).

        Delegates to systemd. nvblox is intentionally NOT started here — it is
        opt-in via /api/isaac/launch-nvblox or `systemctl start nomad-nvblox`.
        """
        # If everything is already healthy, short-circuit.
        runtime_state = await asyncio.to_thread(
            _probe_isaac_runtime_state,
            force_refresh=True,
        )
        if runtime_state["container_running"] and runtime_state["bridge_running"]:
            return {
                "success": True,
                "message": "Isaac ROS stack already running.",
                "already_running": True,
            }

        app.state.isaac_startup_last_initiated = time.time()
        result = await asyncio.get_running_loop().run_in_executor(
            None, lambda: _systemctl("start", ISAAC_STACK_UNITS)
        )
        if result["success"]:
            result["message"] = (
                "Isaac ROS stack starting via systemd. Check status in 30-60s."
            )
        return result

    @app.post("/api/isaac/stop", tags=["Isaac ROS"])
    async def isaac_stop():
        """Stop the Isaac ROS stack (also stops nvblox if running)."""
        # Stop in reverse order so dependents go down first.
        units = ("nomad-nvblox.service",) + tuple(reversed(ISAAC_STACK_UNITS))
        result = await asyncio.get_running_loop().run_in_executor(
            None, lambda: _systemctl("stop", units)
        )
        return result

    @app.post("/api/isaac/launch-nvblox", tags=["Isaac ROS"])
    async def isaac_launch_nvblox(request: Request):
        """
        Bring the single optional nvblox mapper up via systemd.
        """
        loop = asyncio.get_running_loop()
        result = await loop.run_in_executor(
            None, lambda: _systemctl("start", ("nomad-nvblox.service",))
        )
        if result.get("success"):
            request.app.state.detection_enabled = False
            request.app.state.detection_last_update = 0.0
        return result

    def _start_ros_http_bridge(bridge_api_key: str = "", bridge_internal_token: str = "") -> dict:
        """Start ros_http_bridge inside the Isaac ROS container (idempotent)."""
        container = "nomad_isaac_ros"
        try:
            ps = subprocess.run(
                ["docker", "ps", "--filter", f"name={container}", "--format", "{{.Names}}"],
                capture_output=True, text=True, timeout=5,
            )
            if container not in ps.stdout:
                return {"success": False, "error": "Isaac ROS container not running"}
        except Exception as e:
            return {"success": False, "error": str(e)}

        # Check if already running
        probe = subprocess.run(
            ["docker", "exec", container, "pgrep", "-f", "ros_http_bridge"],
            capture_output=True, timeout=5,
        )
        if probe.returncode == 0:
            return {"success": True, "message": "ros_http_bridge already running"}

        env_exports = ""
        if bridge_api_key:
            env_exports += f"export NOMAD_API_KEY='{bridge_api_key}'\n"
        if bridge_internal_token:
            env_exports += f"export NOMAD_INTERNAL_TOKEN='{bridge_internal_token}'\n"

        cmd = (
            f"{env_exports}"
            "source /opt/ros/humble/setup.bash 2>/dev/null; "
            "source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null; "
            "setsid bash -c '"
            "while true; do "
            "python3 /workspaces/isaac_ros-dev/edge_core/ros_http_bridge.py "
            "--host localhost --port 8000 --rate 30 "
            "--vio-topic /zed/zed_node/odom "
            "--mag-topic /zed/zed_node/imu/mag "
            "--mesh-topic /nvblox_node/color_layer_marker "
            "--high-rate-transport http; "
            "echo \"ros_http_bridge exited (rc=$?), restarting in 10s...\"; "
            "sleep 10; "
            "done' >> /tmp/ros_bridge.log 2>&1 &"
        )
        try:
            subprocess.run(
                ["docker", "exec", "-d", container, "bash", "-c", cmd],
                capture_output=True, timeout=10, check=True,
            )
        except subprocess.CalledProcessError as e:
            return {"success": False, "error": f"Failed to start bridge: {e.stderr or e}"}

        # Brief wait to confirm it started
        import time as _time
        for _ in range(5):
            _time.sleep(1)
            chk = subprocess.run(
                ["docker", "exec", container, "pgrep", "-f", "ros_http_bridge"],
                capture_output=True, timeout=5,
            )
            if chk.returncode == 0:
                return {"success": True, "message": "ros_http_bridge started"}

        return {"success": False, "error": "ros_http_bridge process did not appear within 5s"}

    @app.post("/api/isaac/bridge/start", tags=["Isaac ROS"])
    async def isaac_bridge_start():
        """
        Start the ROS-HTTP bridge inside the Isaac ROS container.

        Delegates to nomad-ros-http-bridge.service so systemd remains
        authoritative. Idempotent: systemctl start is a no-op when already
        active. Auth env (NOMAD_API_KEY, NOMAD_INTERNAL_TOKEN) is read from
        config/nomad.env at unit-start time.
        """
        return await asyncio.get_running_loop().run_in_executor(
            None, lambda: _systemctl("start", ("nomad-ros-http-bridge.service",))
        )

    @app.post("/api/isaac/bridge/stop", tags=["Isaac ROS"])
    async def isaac_bridge_stop():
        """Stop the ROS-HTTP bridge via its systemd unit."""
        return await asyncio.get_running_loop().run_in_executor(
            None, lambda: _systemctl("stop", ("nomad-ros-http-bridge.service",))
        )

    @app.post("/api/isaac/nvblox/start", tags=["Isaac ROS"])
    async def isaac_nvblox_start():
        """Bring nvblox up via systemd. Idempotent."""
        return await asyncio.get_running_loop().run_in_executor(
            None, lambda: _systemctl("start", ("nomad-nvblox.service",))
        )

    @app.post("/api/isaac/nvblox/stop", tags=["Isaac ROS"])
    async def isaac_nvblox_stop():
        """
        Stop nvblox via systemd. ZED, ros_http_bridge, target_localizer, and
        the video bridge are unaffected (they're owned by separate units and
        don't depend on nvblox).
        """
        return await asyncio.get_running_loop().run_in_executor(
            None, lambda: _systemctl("stop", ("nomad-nvblox.service",))
        )

    # Backward-compatible alias used by the older Mission Planner button.
    @app.post("/api/isaac/stop-nvblox", tags=["Isaac ROS"])
    async def isaac_stop_nvblox():
        return await asyncio.get_running_loop().run_in_executor(
            None, lambda: _systemctl("stop", ("nomad-nvblox.service",))
        )

    # Subprocess endpoints below offload blocking docker-exec / tail calls to
    # a worker thread so the Uvicorn event loop stays responsive to MAVLink
    # bridging, heartbeats, and video commands during polling.
    async def _run_subprocess(*args, **kwargs):
        return await asyncio.to_thread(subprocess.run, *args, **kwargs)

    @app.get("/api/isaac/logs", tags=["Isaac ROS"])
    async def isaac_logs(
        log_type: str = Query(
            default="all", description="Log type: all, zed, bridge"
        ),
    ):
        """Get Isaac ROS container logs."""
        # docker-exec tail blocks ~5s on timeout; offload to a worker thread so
        # the event loop keeps servicing MAVLink/video traffic.
        def _tail(path: str, lines: int) -> subprocess.CompletedProcess:
            return subprocess.run(
                ["docker", "exec", "nomad_isaac_ros", "tail", f"-{lines}", path],
                capture_output=True,
                text=True,
                timeout=5,
            )

        try:
            if log_type == "zed":
                result = await asyncio.to_thread(_tail, "/tmp/zed_nvblox.log", 50)
            elif log_type == "bridge":
                result = await asyncio.to_thread(_tail, "/tmp/ros_bridge.log", 50)
            else:
                zed_result, bridge_result = (
                    await asyncio.to_thread(_tail, "/tmp/zed_nvblox.log", 25),
                    await asyncio.to_thread(_tail, "/tmp/ros_bridge.log", 25),
                )
                return {
                    "zed_nvblox": zed_result.stdout
                    if zed_result.returncode == 0
                    else zed_result.stderr,
                    "ros_bridge": bridge_result.stdout
                    if bridge_result.returncode == 0
                    else bridge_result.stderr,
                }

            return {
                "log_type": log_type,
                "logs": result.stdout if result.returncode == 0 else result.stderr,
            }
        except Exception as e:
            return {"error": str(e)}

    @app.get("/api/isaac/vio", tags=["Isaac ROS"])
    async def isaac_vio(request: Request):
        """Get current VIO state from Isaac ROS VSLAM or ZED."""
        isaac_bridge = request.app.state.isaac_bridge
        if not isaac_bridge:
            raise HTTPException(status_code=503, detail="Isaac bridge not initialized")

        vio = isaac_bridge.vio_state
        if not vio:
            return {"valid": False, "message": "No VIO data available"}

        return {
            "valid": vio.valid,
            "timestamp": vio.timestamp,
            "position": {"x": vio.x, "y": vio.y, "z": vio.z},
            "orientation": {"roll": vio.roll, "pitch": vio.pitch, "yaw": vio.yaw},
            "velocity": {"vx": vio.vx, "vy": vio.vy, "vz": vio.vz},
            "confidence": vio.confidence,
            "source": vio.source,
        }

    @app.get("/api/isaac/detections", tags=["Isaac ROS"])
    async def isaac_detections(request: Request):
        """Get current YOLO detections from Isaac ROS."""
        isaac_bridge = request.app.state.isaac_bridge
        if not isaac_bridge:
            raise HTTPException(status_code=503, detail="Isaac bridge not initialized")

        detections = isaac_bridge.detections
        return {
            "count": len(detections),
            "detections": [
                {
                    "class_name": d.class_name,
                    "class_id": d.class_id,
                    "confidence": d.confidence,
                    "bbox": {
                        "x": d.bbox_x,
                        "y": d.bbox_y,
                        "w": d.bbox_w,
                        "h": d.bbox_h,
                    },
                    "world_pos": {
                        "x": d.world_x,
                        "y": d.world_y,
                        "z": d.world_z,
                    }
                    if d.world_x is not None
                    else None,
                }
                for d in detections
            ],
        }

    @app.get("/api/isaac/exclusion_map", tags=["Isaac ROS"])
    async def isaac_exclusion_map(request: Request):
        """Get exclusion map from Isaac ROS bridge (auto-managed)."""
        isaac_bridge = request.app.state.isaac_bridge
        exclusion_map = request.app.state.exclusion_map

        if not isaac_bridge:
            # Fall back to local exclusion map
            return {
                "backend": "local",
                "total_targets": len(exclusion_map),
                "targets": exclusion_map,
            }

        exclusion = isaac_bridge.exclusion_map
        return {
            "backend": "isaac_ros",
            "total_targets": len(exclusion),
            "targets": [
                {
                    "id": e.id,
                    "position": {"x": e.x, "y": e.y, "z": e.z},
                    "radius": e.radius,
                    "timestamp": e.timestamp,
                    "hit_count": e.hit_count,
                }
                for e in exclusion.values()
            ],
        }

    @app.post("/api/isaac/exclusion_map/add", tags=["Isaac ROS"])
    async def isaac_add_exclusion(hit_request: Task2HitRequest, request: Request):
        """Add target to Isaac ROS managed exclusion map."""
        isaac_bridge = request.app.state.isaac_bridge
        exclusion_map = request.app.state.exclusion_map

        if isaac_bridge:
            target_id = isaac_bridge.add_to_exclusion_map(
                x=hit_request.x, y=hit_request.y, z=hit_request.z
            )
            return {"success": True, "target_id": target_id}
        else:
            # Fall back to local
            exclusion_map.append(
                {
                    "x": hit_request.x,
                    "y": hit_request.y,
                    "z": hit_request.z,
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                }
            )
            return {"success": True, "target_id": f"local_{len(exclusion_map)}"}

    @app.post("/api/isaac/exclusion_map/clear", tags=["Isaac ROS"])
    async def isaac_clear_exclusion(request: Request):
        """Clear Isaac ROS managed exclusion map."""
        isaac_bridge = request.app.state.isaac_bridge

        if isaac_bridge:
            count = isaac_bridge.clear_exclusion_map()
            return {"success": True, "cleared": count}
        else:
            count = len(request.app.state.exclusion_map)
            request.app.state.exclusion_map = []
            return {"success": True, "cleared": count}

