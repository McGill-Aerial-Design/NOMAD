import asyncio
import json
import math
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

VIO_AREA_MAP_ROOT = os.path.realpath(
    os.environ.get(
        "NOMAD_VIO_AREA_MAP_ROOT",
        "/workspaces/isaac_ros-dev/data/area_maps",
    )
)


def normalize_vio_area_file_path_or_raise(
    file_path: str,
    *,
    root: Optional[str] = None,
) -> str:
    """Resolve a VIO/nvblox map path and confine it to the configured map root."""
    normalized = (file_path or "").strip()
    if not normalized:
        raise HTTPException(status_code=400, detail="file_path is required")

    root_real = os.path.realpath(root or VIO_AREA_MAP_ROOT)
    if not os.path.isabs(normalized):
        normalized = os.path.join(root_real, normalized)
    real = os.path.realpath(normalized)
    if not (real == root_real or real.startswith(root_real + os.sep)):
        raise HTTPException(
            status_code=403,
            detail=f"file_path must be under {root_real}",
        )
    return real


def register_task2_routes(app, ctx) -> None:
    logger = ctx.logger
    _apply_vio_update_from_request = ctx.apply_vio_update_from_request
    _get_vio_snapshot = ctx.get_vio_snapshot
    _dispatch_nav_velocity = ctx.dispatch_nav_velocity

    # ==================== Task 2: Extinguish (Indoor) ====================

    @app.post("/api/task/2/reset_map", tags=["Task 2"])
    async def task2_reset_map(request: Request):
        """
        Reset the exclusion map for Task 2.

        Clears all recorded target positions, allowing
        previously sprayed targets to be detected again.
        """
        request.app.state.exclusion_map = []
        logger.info("Task 2 exclusion map reset")

        return {
            "success": True,
            "message": "Exclusion map cleared",
            "total_targets": 0,
        }

    @app.post("/api/task/2/target_hit", tags=["Task 2"])
    async def task2_target_hit(hit_request: Task2HitRequest, request: Request):
        """
        Register a target hit for Task 2 exclusion map.

        Records the 3D position of a sprayed target to prevent
        re-engagement. Uses VIO frame coordinates.
        """
        target = {
            "x": hit_request.x,
            "y": hit_request.y,
            "z": hit_request.z,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }

        request.app.state.exclusion_map.append(target)
        logger.info(
            f"Task 2 target hit registered: ({hit_request.x}, {hit_request.y}, {hit_request.z})"
        )

        return {
            "success": True,
            "target": target,
            "total_targets": len(request.app.state.exclusion_map),
        }

    @app.get("/api/task/2/exclusion_map", tags=["Task 2"])
    async def task2_get_exclusion_map(request: Request):
        """Get current exclusion map targets."""
        return {
            "total_targets": len(request.app.state.exclusion_map),
            "targets": request.app.state.exclusion_map,
        }

    # ==================== VIO Endpoints ====================

    @app.get("/api/vio/status", tags=["VIO"])
    async def vio_status(request: Request):
        """Get VIO pipeline status."""
        # Check for external VIO state first (confidence on 0-1 scale)
        external_vio_state = _get_vio_snapshot()["external_vio_state"]
        if external_vio_state:
            # External VIO confidence is 0-1 scale
            confidence_0_1 = external_vio_state.get("confidence", 0)
            vio_fresh = bool(external_vio_state.get("fresh", False))
            return {
                "health": "healthy" if confidence_0_1 > 0.5 and vio_fresh else "degraded",
                "tracking_confidence": confidence_0_1,  # 0-1 scale
                "position_valid": vio_fresh,
                "age_seconds": external_vio_state.get("age_seconds"),
                "max_age_seconds": external_vio_state.get("max_age_seconds"),
                "message_rate_hz": 30.0,
                "reset_counter": 0,
                "source": external_vio_state.get("source", "external"),
            }

        return {
            "health": "unknown",
            "tracking_confidence": 0,  # 0-1 scale
            "position_valid": False,
            "message_rate_hz": 0,
            "reset_counter": 0,
            "source": "none",
        }

    @app.post("/api/vio/update", tags=["VIO"])
    async def vio_update(vio_request: VIOUpdateRequest, request: Request):
        """
        Receive VIO pose update from external source (ROS bridge).

        This endpoint is called by the ros_http_bridge.py script running
        inside the Isaac ROS container to send VIO data to edge_core.
        """
        trajectory_points = _apply_vio_update_from_request(vio_request)
        return {"success": True, "trajectory_points": trajectory_points}

    @app.get("/api/vio/pose", tags=["VIO"])
    async def vio_pose(request: Request):
        """Get current VIO pose (position and orientation)."""
        external_vio_state = _get_vio_snapshot()["external_vio_state"]
        if external_vio_state:
            payload = dict(external_vio_state)
            payload["source"] = "ros_http_bridge"
            payload["valid"] = bool(payload.get("fresh", False))
            return payload

        return {
            "valid": False,
            "message": "No external VIO data available",
            "source": "external_vio_required",
        }

    @app.get("/api/vio/trajectory", tags=["VIO"])
    async def vio_trajectory(
        request: Request, max_points: int = Query(default=100, le=1000)
    ):
        """
        Get VIO trajectory for visualization.

        Returns a list of (x, y, z) points representing the drone's path.
        Use max_points to limit the response size.
        """
        with request.app.state.vio_state_lock:
            trajectory = list(request.app.state.vio_trajectory)
        points = trajectory[-max_points:] if trajectory else []
        return {
            "total_points": len(trajectory),
            "returned_points": len(points),
            "trajectory": points,
        }

    @app.delete("/api/vio/trajectory", tags=["VIO"])
    async def vio_clear_trajectory(request: Request):
        """Clear the VIO trajectory history."""
        with request.app.state.vio_state_lock:
            count = len(request.app.state.vio_trajectory)
            request.app.state.vio_trajectory = []
        return {"success": True, "cleared_points": count}

    def _resolve_nvblox_empty_map_path(file_path: Optional[str] = None) -> str:
        """Resolve baseline map path used for non-restart nvblox clear workflow."""
        if file_path and str(file_path).strip():
            return normalize_vio_area_file_path_or_raise(str(file_path))

        env_path = (os.getenv("NVBLOX_EMPTY_MAP_PATH") or "").strip()
        if env_path:
            return normalize_vio_area_file_path_or_raise(env_path)

        return normalize_vio_area_file_path_or_raise("empty_map.nvblx")

    def _status_for_vio_area_failure(message: str, default_status: int) -> int:
        """Map backend failure text to HTTP status while preserving safe defaults."""
        text = (message or "").strip().lower()
        if not text:
            return default_status
        if "waiting for service to become available" in text:
            return 503
        if "success=false" in text or "success = false" in text:
            return 409
        if "not found" in text or "no such file" in text or "does not exist" in text:
            return 404
        if "reported success but file is missing" in text:
            return 500
        if "reported success but file is empty" in text:
            return 500
        if "missing" in text or "invalid" in text or "empty" in text:
            return 400
        if "must be enabled before saving" in text:
            return 409
        if "timed out waiting for area map export" in text:
            return 504
        if "export state api is unavailable" in text:
            return 503
        return default_status

    def _actionable_vio_area_detail(message: str, fallback: str) -> str:
        """Attach an operator-facing next action to backend errors."""
        base = (message or "").strip() or fallback
        lower = base.lower()

        if "must be enabled before saving" in lower:
            return (
                f"{base}. Action: start VIO/positional tracking first, then retry save."
            )
        if "not found" in lower or "missing" in lower or "no such file" in lower:
            return f"{base}. Action: verify the map path exists on Jetson and retry."
        if "timed out waiting for area map export" in lower:
            return f"{base}. Action: increase timeout_s or retry after camera motion/feature-rich view."
        if "export state api is unavailable" in lower:
            return f"{base}. Action: use wait_for_completion=false for this runtime or upgrade ZED SDK/runtime."
        if "failed to load area map for relocalization" in lower:
            return f"{base}. Action: confirm area file was created on this unit and is not corrupted."
        return base

    def _sanitize_ros2_service_output(output: str) -> str:
        """Strip noisy ROS2 CLI/runtime lines and keep actionable output."""
        cleaned_lines: list[str] = []
        for raw_line in (output or "").splitlines():
            line = raw_line.strip()
            if not line:
                continue

            lower = line.lower()
            if line.startswith("/usr/lib/python") and "runtimewarning" in lower:
                continue
            if "jtop.core.jetson_variables" in lower:
                continue
            if line.startswith("warn(RuntimeWarning"):
                continue
            if "waiting for service to become available" in lower:
                continue

            cleaned_lines.append(line)

        return "\n".join(cleaned_lines).strip()

    def _ensure_file_path_parent_dir_in_isaac_container_or_raise(
        service_type: str,
        request_payload: dict[str, Any],
    ) -> None:
        """Ensure parent dir exists for FilePath service calls inside container."""
        if service_type != "nvblox_msgs/srv/FilePath":
            return

        file_path = str(request_payload.get("file_path") or "").strip()
        if not file_path:
            return

        parent_dir = os.path.dirname(file_path)
        if not parent_dir:
            return

        mkdir_cmd = f"mkdir -p {shlex.quote(parent_dir)}"
        try:
            mkdir_result = subprocess.run(
                ["docker", "exec", "nomad_isaac_ros", "bash", "-lc", mkdir_cmd],
                capture_output=True,
                text=True,
                timeout=10,
            )
        except Exception as e:
            logger.error(f"Failed to prepare map directory in Isaac container: {e}")
            raise HTTPException(
                status_code=503,
                detail=f"Failed to prepare map directory in Isaac container: {parent_dir}",
            )

        if mkdir_result.returncode != 0:
            detail = (
                _sanitize_ros2_service_output(
                    "\n".join(
                        part
                        for part in [
                            (mkdir_result.stdout or "").strip(),
                            (mkdir_result.stderr or "").strip(),
                        ]
                        if part
                    )
                )
                or f"mkdir failed for {parent_dir}"
            )
            raise HTTPException(status_code=503, detail=detail)

    def _isaac_container_file_exists(file_path: str) -> bool:
        """Check if a file exists inside the Isaac ROS container."""
        normalized = (file_path or "").strip()
        if not normalized:
            return False

        test_cmd = f"test -f {shlex.quote(normalized)}"
        try:
            probe = subprocess.run(
                ["docker", "exec", "nomad_isaac_ros", "bash", "-lc", test_cmd],
                capture_output=True,
                text=True,
                timeout=10,
            )
            return probe.returncode == 0
        except Exception:
            return False

    _ROS_SERVICE_PROXY_URL = os.environ.get(
        "NOMAD_ROS_SERVICE_PROXY_URL",
        "http://127.0.0.1:8101/srv/call",
    )

    def _call_ros2_service_in_isaac_container_or_raise(
        service_name: str,
        service_type: str,
        request_payload: dict[str, Any],
        timeout_s: float = 30.0,
        skip_type_check: bool = False,
        force_refresh_runtime: bool = True,
    ) -> str:
        """Call a ROS2 service inside the active Isaac container stack via the
        nomad_ros_http_bridge service proxy (a long-lived rclpy node with cached
        service clients). This replaces the prior `docker exec ros2 service call`
        path, which paid 5-15s of Python/rclpy cold-start per call.

        Latency: ~5-50ms once the client is cached (first call may take longer
        while DDS discovery completes)."""
        # Runtime probes are intentionally skipped here: the proxy IS the
        # liveness signal. If the container/bridge is down we get URLError and
        # surface a 503; if a specific service isn't advertised we get the
        # proxy's own 503 via `service unavailable`. The old `docker exec`
        # probes added ~800ms-1.2s per call (four docker round-trips) for no
        # information we don't already learn from the call itself.

        _ensure_file_path_parent_dir_in_isaac_container_or_raise(
            service_type=service_type,
            request_payload=request_payload,
        )

        import urllib.request
        import urllib.error

        body = json.dumps(
            {
                "service": service_name,
                "type": service_type,
                "args": request_payload or {},
                "timeout_s": float(timeout_s),
            }
        ).encode("utf-8")
        req = urllib.request.Request(
            _ROS_SERVICE_PROXY_URL,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        # Allow proxy headroom over the in-proxy timeout so we observe its
        # 504 rather than killing the connection ourselves.
        http_timeout = max(timeout_s + 5.0, 10.0)
        try:
            with urllib.request.urlopen(req, timeout=http_timeout) as resp:
                data = json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            try:
                err_body = json.loads(e.read().decode("utf-8"))
                detail = err_body.get("detail") or str(e)
            except Exception:
                detail = str(e)
            if e.code == 504:
                raise HTTPException(
                    status_code=504,
                    detail=f"ROS2 service call timed out: {service_name}",
                )
            if e.code == 503:
                raise HTTPException(
                    status_code=503,
                    detail=detail or f"ROS2 service not available: {service_name}",
                )
            # 502 / 400 / other: service-reported failure or bad args.
            # Preserve the VIO-area status mapping for nvblox-style failures
            # so existing handlers continue to surface meaningful codes.
            if service_type == "std_srvs/srv/Trigger":
                status = 502
            else:
                status = _status_for_vio_area_failure(detail, default_status=e.code or 502)
            raise HTTPException(
                status_code=status,
                detail=detail or f"ROS2 service reported failure: {service_name}",
            )
        except urllib.error.URLError as e:
            reason = getattr(e, "reason", str(e))
            logger.error(f"ROS service proxy unreachable ({service_name}): {reason}")
            raise HTTPException(
                status_code=503,
                detail=f"ROS service proxy unreachable: {reason}",
            )
        except Exception as e:
            logger.error(f"ROS service proxy call failed ({service_name}): {e}")
            raise HTTPException(
                status_code=503,
                detail=f"ROS service proxy call failed: {e}",
            )

        message = (data or {}).get("message") or ""
        return message or f"ROS2 service call succeeded: {service_name}"

    ctx.resolve_nvblox_empty_map_path = _resolve_nvblox_empty_map_path
    ctx.isaac_container_file_exists = _isaac_container_file_exists
    ctx.call_ros2_service_in_isaac_container_or_raise = _call_ros2_service_in_isaac_container_or_raise

    @app.post("/api/vio/area/save", tags=["VIO"])
    async def vio_area_save(area_request: VIOAreaSaveRequest, request: Request):
        """Save relocalization map via direct ZED area map backend or nvblox map service."""
        file_path = normalize_vio_area_file_path_or_raise(area_request.file_path)
        camera_service = request.app.state.camera_service

        if camera_service and hasattr(camera_service, "save_area_map"):
            backend = "direct_zed_area_map"
            try:
                success, message = camera_service.save_area_map(
                    file_path=file_path,
                    wait_for_completion=area_request.wait_for_completion,
                    timeout_s=area_request.timeout_s,
                )
            except Exception as e:
                logger.error(f"Direct camera save_area_map failed: {e}")
                raise HTTPException(
                    status_code=500, detail=f"Direct camera save failed: {e}"
                )

            if not success:
                status = _status_for_vio_area_failure(message, default_status=500)
                raise HTTPException(
                    status_code=status,
                    detail=_actionable_vio_area_detail(
                        message,
                        "Failed to save area map",
                    ),
                )
        else:
            backend = "nvblox_map_service"
            message = _call_ros2_service_in_isaac_container_or_raise(
                service_name="/nvblox_node/save_map",
                service_type="nvblox_msgs/srv/FilePath",
                request_payload={"file_path": file_path},
                timeout_s=area_request.timeout_s,
            )

        return {
            "success": True,
            "message": message or f"Relocalization map saved via {backend}",
            "file_path": file_path,
            "backend": backend,
        }

    @app.post("/api/vio/area/load", tags=["VIO"])
    async def vio_area_load(area_request: VIOAreaLoadRequest, request: Request):
        """Load relocalization map via direct ZED area map backend or nvblox map service."""
        file_path = normalize_vio_area_file_path_or_raise(area_request.file_path)
        camera_service = request.app.state.camera_service

        if camera_service and hasattr(camera_service, "load_area_map"):
            backend = "direct_zed_area_map"
            try:
                success, message = camera_service.load_area_map(file_path)
            except Exception as e:
                logger.error(f"Direct camera load_area_map failed: {e}")
                raise HTTPException(
                    status_code=500, detail=f"Direct camera load failed: {e}"
                )

            if not success:
                status = _status_for_vio_area_failure(message, default_status=500)
                raise HTTPException(
                    status_code=status,
                    detail=_actionable_vio_area_detail(
                        message,
                        "Failed to load area map",
                    ),
                )
        else:
            backend = "nvblox_map_service"
            message = _call_ros2_service_in_isaac_container_or_raise(
                service_name="/nvblox_node/load_map",
                service_type="nvblox_msgs/srv/FilePath",
                request_payload={"file_path": file_path},
            )

        return {
            "success": True,
            "message": message or f"Relocalization map loaded via {backend}",
            "file_path": file_path,
            "backend": backend,
        }

    @app.post("/api/vio/area/relocalize", tags=["VIO"])
    async def vio_area_relocalize(area_request: VIOAreaLoadRequest, request: Request):
        """Load relocalization map and trigger tracking reset with backend-explicit response."""
        file_path = normalize_vio_area_file_path_or_raise(area_request.file_path)
        camera_service = request.app.state.camera_service

        if camera_service and hasattr(camera_service, "load_area_map"):
            backend = "direct_zed_area_map"
            try:
                load_success, load_message = camera_service.load_area_map(file_path)
            except Exception as e:
                logger.error(f"Direct camera relocalize load failed: {e}")
                raise HTTPException(
                    status_code=500, detail=f"Direct camera relocalize load failed: {e}"
                )

            if not load_success:
                status = _status_for_vio_area_failure(load_message, default_status=500)
                raise HTTPException(
                    status_code=status,
                    detail=_actionable_vio_area_detail(
                        load_message,
                        "Failed to load area map",
                    ),
                )

            reset_ok = True
            reset_message = "Tracking reset not available on direct backend"
            if hasattr(camera_service, "reset_tracking"):
                try:
                    reset_ok = bool(camera_service.reset_tracking())
                    reset_message = "Tracking reset triggered"
                except Exception as e:
                    logger.error(
                        f"Direct camera reset_tracking failed during relocalize: {e}"
                    )
                    raise HTTPException(
                        status_code=500,
                        detail=f"Direct camera reset_tracking failed: {e}",
                    )

            if not reset_ok:
                raise HTTPException(
                    status_code=500,
                    detail="Direct camera reset_tracking returned failure",
                )

            return {
                "success": True,
                "message": f"{load_message}; {reset_message}",
                "file_path": file_path,
                "backend": backend,
            }

        backend = "nvblox_map_service"
        load_message = _call_ros2_service_in_isaac_container_or_raise(
            service_name="/nvblox_node/load_map",
            service_type="nvblox_msgs/srv/FilePath",
            request_payload={"file_path": file_path},
        )
        reset_message = _call_ros2_service_in_isaac_container_or_raise(
            service_name="/zed/zed_node/reset_pos_tracking",
            service_type="std_srvs/srv/Trigger",
            request_payload={},
        )

        return {
            "success": True,
            "message": f"{load_message}; {reset_message}",
            "file_path": file_path,
            "backend": backend,
        }

    @app.post("/api/vio/reset_origin", tags=["VIO"])
    async def vio_reset_origin(request: Request):
        """Reset VIO tracking origin with backend-explicit status."""
        camera_service = request.app.state.camera_service
        if camera_service and hasattr(camera_service, "reset_tracking"):
            backend = "direct_zed_area_map"
            try:
                reset_ok = bool(camera_service.reset_tracking())
            except Exception as e:
                logger.error(f"Direct camera reset_tracking failed: {e}")
                raise HTTPException(
                    status_code=500,
                    detail=f"Direct ZED reset_tracking failed: {e}",
                )
            if not reset_ok:
                raise HTTPException(
                    status_code=500,
                    detail="Direct camera reset_tracking returned failure",
                )
            with request.app.state.vio_state_lock:
                request.app.state.vio_trajectory = []
            return {
                "success": True,
                "reset_counter": 0,
                "backend": backend,
                "message": "Tracking origin reset via direct ZED backend; trajectory cleared",
            }

        backend = "ros_service_proxy"
        message = _call_ros2_service_in_isaac_container_or_raise(
            service_name="/zed/zed_node/reset_pos_tracking",
            service_type="std_srvs/srv/Trigger",
            request_payload={},
        )
        with request.app.state.vio_state_lock:
            request.app.state.vio_trajectory = []
        return {
            "success": True,
            "reset_counter": 0,
            "backend": backend,
            "message": f"{message}; trajectory cleared",
        }

    @app.get("/api/vio/calibration", tags=["VIO"])
    async def vio_calibration_status(request: Request):
        """Get VIO calibration validation results (deprecated -- VIO via ros_http_bridge)."""
        raise HTTPException(
            status_code=503,
            detail="VIO calibration not available: VIO is now handled by ros_http_bridge",
        )

    # ==================== Navigation Endpoints ====================
    # Jetson-centric navigation: Isaac ROS nav2/nvblox -> Edge Core -> ArduPilot GUIDED

    @app.get("/api/nav/status", tags=["Navigation"])
    async def nav_status(request: Request):
        """
        Get navigation controller status.

        Returns the current navigation mode, health, and commanded velocities.
        This is the Jetson-centric navigation controller that bridges
        ROS2 nav2/nvblox to ArduPilot GUIDED mode.
        """
        nav_controller = request.app.state.nav_controller
        if not nav_controller:
            return {
                "available": False,
                "mode": "disabled",
                "message": "Navigation controller not initialized",
            }

        status = nav_controller.status
        return {
            "available": True,
            **status.to_dict(),
        }

    @app.post("/api/nav/velocity", tags=["Navigation"])
    async def nav_velocity(nav_request: NavVelocityRequest, request: Request):
        """
        Send velocity command for autonomous navigation.

        This is the primary endpoint for Jetson-centric navigation.
        Isaac ROS nav2/nvblox generates /cmd_vel which ros_http_bridge
        forwards here. Edge Core then sends SET_POSITION_TARGET_LOCAL_NED
        to ArduPilot in GUIDED mode.

        Velocity convention (ROS REP 103):
        - vx: Forward velocity (m/s, positive = forward)
        - vy: Lateral velocity (m/s, positive = left)
        - vz: Vertical velocity (m/s, positive = up)
        - yaw_rate: Yaw rate (rad/s, positive = CCW)
        """
        try:
            success = _dispatch_nav_velocity(nav_request)
        except ValueError as e:
            raise HTTPException(status_code=409, detail=str(e))
        except RuntimeError as e:
            raise HTTPException(status_code=503, detail=str(e))

        return {
            "success": success,
            "timestamp": nav_request.timestamp,
            "commanded": {
                "vx": nav_request.vx,
                "vy": nav_request.vy,
                "vz": nav_request.vz,
                "yaw_rate": nav_request.yaw_rate,
            },
        }

    @app.post("/api/nav/position", tags=["Navigation"])
    async def nav_position(pos_request: NavPositionRequest, request: Request):
        """
        Send position target for navigation.

        Position is in local NED frame relative to VIO origin.
        """
        nav_controller = request.app.state.nav_controller
        if not nav_controller:
            raise HTTPException(
                status_code=503, detail="Navigation controller not initialized"
            )

        success = nav_controller.send_position(
            x=pos_request.x,
            y=pos_request.y,
            z=pos_request.z,
            yaw=pos_request.yaw,
            source=pos_request.source,
        )
        if not success:
            raise HTTPException(status_code=409, detail="Position target rejected")

        return {
            "success": success,
            "target": {
                "x": pos_request.x,
                "y": pos_request.y,
                "z": pos_request.z,
                "yaw": pos_request.yaw,
            },
        }

    @app.post("/api/nav/stop", tags=["Navigation"])
    async def nav_stop(request: Request):
        """
        Emergency stop - send zero velocity command.

        Use this to immediately halt all movement. The vehicle will
        attempt to hold position (requires VIO/GPS).
        """
        nav_controller = request.app.state.nav_controller
        if not nav_controller:
            raise HTTPException(
                status_code=503, detail="Navigation controller not initialized"
            )

        success = nav_controller.stop_movement()
        return {"success": success, "message": "Stop command sent"}

    @app.post("/api/nav/enable_guided", tags=["Navigation"])
    async def nav_enable_guided(request: Request):
        """
        Request ArduPilot to enter GUIDED mode.

        GUIDED mode is required for Jetson navigation commands to work.
        This sends a MAVLink mode change request to the flight controller.
        """
        nav_controller = request.app.state.nav_controller
        if not nav_controller:
            raise HTTPException(
                status_code=503, detail="Navigation controller not initialized"
            )

        success = nav_controller.enable_guided_mode()
        return {
            "success": success,
            "message": "GUIDED mode requested"
            if success
            else "Failed to request GUIDED mode",
        }

    # ==================== Auto Takeoff / Auto Land (CONOPS 5.2.4) =============
    # 5pts for autonomous takeoff + 5pts for autonomous landing. A single
    # successful demonstration of each is sufficient - the operator is
    # expected to press these buttons once during the flight window.

    @app.post("/api/flight/takeoff", tags=["Flight"])
    async def flight_takeoff(request: Request):
        """Autonomous takeoff to ``altitude_m`` AGL (default 30 m).

        Body: ``{"altitude_m": 30.0}`` - clamped to 1.0-60.0 m on the backend.

        Sequence: switch to GUIDED -> arm motors -> MAV_CMD_NAV_TAKEOFF. The
        pilot keeps the RC mode switch as a safety override at all times.
        """
        nav_controller = request.app.state.nav_controller
        if not nav_controller:
            raise HTTPException(status_code=503, detail="Navigation controller not initialized")

        try:
            body = await request.json() if (await request.body()) else {}
            if not isinstance(body, dict):
                body = {}
        except Exception:
            body = {}
        altitude_m = float(body.get("altitude_m", 30.0))

        result = nav_controller.auto_takeoff(altitude_m=altitude_m)
        if not result.get("success"):
            raise HTTPException(status_code=409, detail=result.get("error", "Auto-takeoff failed"))
        return result

    @app.post("/api/flight/land", tags=["Flight"])
    async def flight_land(request: Request):
        """Autonomous landing at the current position (ArduCopter LAND mode)."""
        nav_controller = request.app.state.nav_controller
        if not nav_controller:
            raise HTTPException(status_code=503, detail="Navigation controller not initialized")

        result = nav_controller.auto_land()
        if not result.get("success"):
            raise HTTPException(status_code=409, detail=result.get("error", "Auto-land failed"))
        return result

    # ==================== Target Color (Pre-flight) ===========================
    # Phase 2 Q&A #5: Task 2 targets will be a single solid color, "depending
    # on what we can source." Operator observes the actual color at flightline
    # and records it here for the pre-flight checklist and judge-facing record.
    # The live Task 2 overlay remains shape/range based by default.

    _COLOR_CONFIG_PATH = os.path.expanduser("~/.nomad/task2_color.json")
    _ALLOWED_COLORS = ("purple", "blue", "red", "orange", "yellow", "green",
                       "black", "white", "magenta", "cyan")

    @app.get("/api/task/2/target_color", tags=["Task 2"])
    async def task2_get_target_color():
        """Return the operator-set Task 2 target color (or default purple)."""
        try:
            if os.path.exists(_COLOR_CONFIG_PATH):
                with open(_COLOR_CONFIG_PATH) as f:
                    data = json.load(f)
                color = str(data.get("color", "purple")).lower()
                return {"color": color, "source": "file", "path": _COLOR_CONFIG_PATH}
        except Exception as e:
            logger.warning(f"Failed to read target color config: {e}")
        return {"color": "purple", "source": "default", "path": _COLOR_CONFIG_PATH}

    @app.post("/api/task/2/target_color", tags=["Task 2"])
    async def task2_set_target_color(request: Request):
        """Set the Task 2 target color observed at flightline.

        Body: ``{"color": "purple"}``. Accepted values: purple, blue, red,
        orange, yellow, green, black, white, magenta, cyan.
        """
        body = await _parse_request_json_object(request)
        color = str(body.get("color", "")).strip().lower()
        if color not in _ALLOWED_COLORS:
            raise HTTPException(
                status_code=400,
                detail=f"color must be one of: {', '.join(_ALLOWED_COLORS)}",
            )
        try:
            os.makedirs(os.path.dirname(_COLOR_CONFIG_PATH), exist_ok=True)
            with open(_COLOR_CONFIG_PATH, "w") as f:
                json.dump({"color": color, "updated_at": datetime.now(timezone.utc).isoformat()}, f)
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to persist color: {e}")
        logger.info(f"Task 2 target color set to '{color}'")
        return {"success": True, "color": color, "path": _COLOR_CONFIG_PATH}

    # ==================== Pre-Flight Checklist ===============================
    # Aggregates the backend-checkable bits of the operator's pre-flight
    # checklist into one snapshot so the GCS can show a green/red dashboard.

    @app.get("/api/task/2/preflight", tags=["Task 2"])
    async def task2_preflight(request: Request):
        """Return pre-flight readiness for Task 2 autonomy.

        Each item has ``{ok: bool, label: str, detail: str}``. The UI renders
        green/red rows so the pilot can resolve blockers before the demo.
        """
        items: list[dict] = []

        # 1) MAVLink connection + flight mode visibility
        mavlink_svc = getattr(request.app.state, "mavlink_service", None)
        nav_controller = getattr(request.app.state, "nav_controller", None)
        flight_mode = "UNKNOWN"
        armed = False
        connected = False
        try:
            from ..state import StateManager
            s = StateManager.instance().get_state()
            flight_mode = s.flight_mode
            armed = bool(s.armed)
            connected = bool(s.connected)
        except Exception:
            pass
        items.append({
            "key": "mavlink",
            "label": "MAVLink connected to autopilot",
            "ok": connected and bool(mavlink_svc),
            "detail": f"flight_mode={flight_mode} armed={armed}",
        })

        # 2) Spray controller wired
        spray_ctrl = getattr(request.app.state, "spray_controller", None)
        items.append({
            "key": "spray_controller",
            "label": "Spray controller initialized",
            "ok": spray_ctrl is not None,
            "detail": "Available" if spray_ctrl else "Not initialized",
        })

        # 3) Spray calibration file present (TARGET_CAMERA_RANGE_M etc.)
        from ..spray_controller import CALIBRATION_FILE as _CAL_FILE
        cal_ok = os.path.exists(_CAL_FILE)
        items.append({
            "key": "spray_calibration",
            "label": "Spray calibration on disk",
            "ok": cal_ok,
            "detail": _CAL_FILE if cal_ok else f"Missing - run bench calibration to create {_CAL_FILE}",
        })

        # 4) Target color set
        color = "purple"
        color_source = "default"
        try:
            if os.path.exists(_COLOR_CONFIG_PATH):
                with open(_COLOR_CONFIG_PATH) as f:
                    color = str(json.load(f).get("color", "purple")).lower()
                color_source = "file"
        except Exception:
            pass
        items.append({
            "key": "target_color",
            "label": "Task 2 target color set at flightline",
            "ok": color_source == "file",
            "detail": f"color={color} ({color_source})",
        })

        # 5) Google Drive auth ready
        gdrive_ok = False
        gdrive_detail = "Module unavailable"
        try:
            from ..gdrive_upload import gdrive_ready
            gdrive_ok = bool(gdrive_ready())
            gdrive_detail = "OAuth token present" if gdrive_ok else (
                "Token missing - run: python -m edge_core.gdrive_upload --setup <client_secret>.json"
            )
        except Exception as e:
            gdrive_detail = f"Import failed: {e}"
        items.append({
            "key": "gdrive",
            "label": "Google Drive upload ready",
            "ok": gdrive_ok,
            "detail": gdrive_detail,
        })

        # 6) Camera/video bridge snapshot endpoint reachable
        bridge_ok = False
        bridge_detail = ""
        try:
            import urllib.request as _ur
            bridge_port = int(os.environ.get("NOMAD_BRIDGE_HTTP_PORT", "9200"))
            snap_url = f"http://127.0.0.1:{bridge_port}/snapshot"
            with _ur.urlopen(snap_url, timeout=1.5) as r:
                ctype = r.headers.get("Content-Type", "")
                bridge_ok = r.status == 200 and ctype.startswith("image")
                bridge_detail = f"{snap_url} ({ctype})"
        except Exception as e:
            bridge_detail = f"Snapshot endpoint unreachable: {e}"
        items.append({
            "key": "video_bridge",
            "label": "Video bridge snapshot reachable",
            "ok": bridge_ok,
            "detail": bridge_detail,
        })

        # 7) NavController available for GUIDED velocity ops. Task 2 currently
        # uses image-space velocity commands, not position targets, so VIO is
        # diagnostic here rather than a hard preflight gate.
        nav_ok = False
        nav_detail = "Navigation controller not initialized"
        if nav_controller is not None:
            try:
                ns = nav_controller.status
                nav_ok = True
                nav_detail = (
                    f"mode={getattr(ns.mode, 'value', ns.mode)} "
                    f"health={getattr(ns.health, 'value', ns.health)} "
                    f"guided={ns.guided_mode_active} "
                    f"vio_healthy={ns.vio_healthy} vio_age_ms={ns.vio_age_ms}"
                )
            except Exception as e:
                nav_detail = f"Nav status read failed: {e}"
        items.append({
            "key": "nav_controller",
            "label": "Navigation controller ready for GUIDED velocity",
            "ok": nav_ok,
            "detail": nav_detail,
        })

        ready = all(item["ok"] for item in items)
        return {
            "ready": ready,
            "items": items,
            "checked_at": datetime.now(timezone.utc).isoformat(),
        }

    # Nav2 was removed -- Task 2 autonomy now relies on pure visual servoing.
    # The helper below is kept because other endpoints in this module use it.
    async def _parse_request_json_object(request: Request) -> dict[str, Any]:
        """Parse request JSON and enforce object payloads with controlled 400s."""
        try:
            body = await request.json()
        except (json.JSONDecodeError, UnicodeDecodeError, ValueError) as exc:
            raise HTTPException(status_code=400, detail="Invalid JSON body") from exc

        if not isinstance(body, dict):
            raise HTTPException(status_code=400, detail="JSON body must be an object")

        return body

    ctx.parse_request_json_object = _parse_request_json_object

    # ==================== Spray Controller (SP-001 to SP-008) =====================

    @app.get("/api/spray/status", tags=["Spray"])
    async def get_spray_status(request: Request):
        """Get current spray sequence status."""
        spray_ctrl = getattr(request.app.state, "spray_controller", None)
        if not spray_ctrl:
            return {"state": "idle", "error": "Spray controller not initialized"}
        return spray_ctrl.status.to_dict()

    @app.post("/api/spray/trigger", tags=["Spray"])
    async def trigger_spray(request: Request):
        """
        Trigger spray sequence on a target (SP-001).

        Body fields:
            target_id, x, y, z, label, confidence, image_only, range_m
            require_autonomy (bool, default False): when true, the
                trigger is refused if the drone is already inside
                AUTONOMY_MIN_RANGE_M (2.5 m). This is the toggle used
                by the GCS "Auto Spray (autonomy gate)" button - it
                prevents the operator from silently forfeiting the
                CONOPS 20-pt autonomy gate by triggering too close.
                Manual sprays leave it false.

        Drone must be within TRIGGER_MAX_DISTANCE_M (default 5.5 m).
        Sequence (autonomy path): APPROACH -> AIM -> SPRAY -> VERIFY -> UPLOAD.
        Sequence (manual / image-only / inside 2 m): AIM immediately.
        """
        spray_ctrl = getattr(request.app.state, "spray_controller", None)
        if not spray_ctrl:
            raise HTTPException(
                status_code=503, detail="Spray controller not initialized"
            )

        body = await _parse_request_json_object(request)
        from ..spray_controller import SprayTarget

        target = SprayTarget(
            target_id=body.get("target_id", 0),
            x=body.get("x", 0.0),
            y=body.get("y", 0.0),
            z=body.get("z", 0.0),
            label=body.get("label", ""),
            confidence=body.get("confidence", 0.0),
            image_only=bool(body.get("image_only", False)),
            range_m=body.get("range_m", body.get("distance_m")),
        )

        require_autonomy = bool(body.get("require_autonomy", False))
        result = spray_ctrl.trigger(target, require_autonomy=require_autonomy)
        if not result["success"]:
            raise HTTPException(status_code=400, detail=result.get("error"))
        return result

    # ============================================================
    # Task 2 detections (live shape-detector results from video bridge)
    # ============================================================
    @app.get("/api/task/2/detections", tags=["Task 2"])
    async def task2_detections(request: Request):
        """Return the live Task 2 (shape) circle detections drawn by the
        video overlay. Used by the Mission Planner Detect & Spray tab so the
        operator picks from the same circles they see boxed on the video.

        Envelope mirrors /api/detections so the existing UI can consume it
        unchanged: targets are image-only (no world coordinates) since the
        shape detector runs on the 2D RGB frame.
        """
        from ..video_stream_manager import get_video_stream_manager

        mgr = get_video_stream_manager()
        if not mgr:
            return {"current": {"detections": []}, "history": {"detections": []}}

        raw = mgr.get_overlay_detections(source="task2")
        out = []
        for idx, d in enumerate(raw.get("detections", [])):
            cx = d.get("bbox_x", 0.0) + d.get("bbox_w", 0.0) / 2.0
            cy = d.get("bbox_y", 0.0) + d.get("bbox_h", 0.0) / 2.0
            # range_m is populated by the bridge when ZED depth is available
            # at the circle center. When present we drop image_only so the
            # spray controller can run a real approach instead of assuming
            # the drone is already in firing range.
            range_m = d.get("range_m")
            try:
                range_val = float(range_m) if range_m is not None else None
            except (TypeError, ValueError):
                range_val = None
            has_range = range_val is not None and math.isfinite(range_val)
            # Bridge stores confidence as a 0–1 fraction; the rest of the
            # detection API (and the Mission Planner UI) expects a 0–100
            # percentage, so scale here once.
            try:
                conf_frac = float(d.get("confidence", 0.0) or 0.0)
            except (TypeError, ValueError):
                conf_frac = 0.0
            conf_pct = conf_frac * 100.0 if conf_frac <= 1.0 else conf_frac
            out.append({
                "target_id": idx,
                "label": d.get("label", "circle"),
                "confidence": conf_pct,
                "source": d.get("_method") or d.get("_detector") or "task2",
                "seen_count": 1,
                # No world-frame coords — shape detector is 2D — so the
                # target stays image_only=True. range_m (when present) lets
                # the spray controller decide between skip-approach and a
                # real image-space approach driven by the bbox/range, instead
                # of always assuming the drone is already in firing range.
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "image_only": True,
                "range_m": range_val,
                "distance_m": range_val,
                # Pixel info (handy for visual servo / UI debugging).
                "pixel_x": cx,
                "pixel_y": cy,
                "bbox_x": d.get("bbox_x"),
                "bbox_y": d.get("bbox_y"),
                "bbox_w": d.get("bbox_w"),
                "bbox_h": d.get("bbox_h"),
                "src_w": d.get("_src_w"),
                "src_h": d.get("_src_h"),
            })

        with request.app.state.detection_state_lock:
            external = list(getattr(request.app.state, "detected_objects", []))
            external_age_s = (
                time.time() - request.app.state.detection_last_update
                if getattr(request.app.state, "detection_last_update", 0.0) > 0
                else None
            )
        max_external_age_s = float(os.environ.get(
            "NOMAD_TASK2_OFFBOARD_MAX_AGE_S", "1.0"
        ))
        if external_age_s is not None and external_age_s <= max_external_age_s:
            for d in external:
                if not isinstance(d, dict):
                    continue
                source = str(d.get("source", "") or "")
                if source not in ("groundstation_task2", "offboard_task2"):
                    continue
                idx = len(out)
                out.append({
                    "target_id": idx,
                    "label": d.get("label", "circle"),
                    "confidence": d.get("confidence", 0.0),
                    "source": source,
                    "seen_count": d.get("seen_count", 1),
                    "x": d.get("x", 0.0),
                    "y": d.get("y", 0.0),
                    "z": d.get("z", 0.0),
                    "image_only": bool(d.get("image_only", True)),
                    "range_m": d.get("range_m"),
                    "distance_m": d.get("distance_m", d.get("range_m")),
                    "pixel_x": d.get("pixel_x", d.get("cx")),
                    "pixel_y": d.get("pixel_y", d.get("cy")),
                    "bbox_x": d.get("bbox_x"),
                    "bbox_y": d.get("bbox_y"),
                    "bbox_w": d.get("bbox_w"),
                    "bbox_h": d.get("bbox_h"),
                    "src_w": d.get("src_w"),
                    "src_h": d.get("src_h"),
                    "age_seconds": external_age_s,
                })
        return {
            "current": {"detections": out, "count": len(out)},
            "history": {"detections": [], "count": 0},
        }

    # ============================================================
    # Task 2 spray artifacts (manual flow + last-artifacts download)
    # ============================================================
    @app.post("/api/task/2/spray/manual/start", tags=["Task 2", "Spray"])
    async def task2_spray_manual_start():
        """Begin a manual spray session: capture before-image, start video."""
        from ..task2_spray_artifacts import get_artifact_manager
        sess = get_artifact_manager().start_session(source="manual")
        return sess.to_dict()

    @app.post("/api/task/2/spray/manual/stop", tags=["Task 2", "Spray"])
    async def task2_spray_manual_stop():
        """End the active manual spray session: capture after-image, stop video."""
        from ..task2_spray_artifacts import get_artifact_manager
        sess = get_artifact_manager().stop_session()
        if sess is None:
            raise HTTPException(status_code=400, detail="No active spray session")
        return sess.to_dict()

    @app.get("/api/task/2/spray/last_artifacts", tags=["Task 2", "Spray"])
    async def task2_spray_last_artifacts():
        """Return the most recently completed spray session (auto or manual)."""
        from ..task2_spray_artifacts import get_artifact_manager
        sess = get_artifact_manager().last()
        if sess is None:
            raise HTTPException(status_code=404, detail="No spray sessions yet")
        return sess.to_dict()

    @app.post("/api/task/2/spray/upload", tags=["Task 2", "Spray"])
    async def task2_spray_upload(request: Request):
        """Upload spray artifacts to Google Drive directly from the Jetson.

        Avoids the Mission Planner round-trip: instead of downloading every
        artifact over the link and re-uploading, this endpoint uses the
        already-wired ``gdrive_upload`` module on the edge. Mission Planner
        only triggers the upload and renders the resulting Drive file ids.

        Request body (all optional — defaults to last session's paths):
            {
              "session_id": "abcd1234",        # optional, lookup by id
              "before_image_path": "/.../before.jpg",
              "after_image_path":  "/.../after.jpg",
              "video_path":        "/.../spray.mp4",
              "name_prefix": "task2"           # filename prefix
            }
        Returns:
            {"ok": N, "fail": M, "results": [
                {"path": "...", "name": "...", "file_id": "...", "error": null},
                ...
            ]}
        """
        from ..task2_spray_artifacts import SESSIONS_ROOT, get_artifact_manager
        try:
            body = await request.json()
            if not isinstance(body, dict):
                body = {}
        except Exception:
            body = {}

        sess_dict: dict = {}
        sid = body.get("session_id")
        if sid:
            last = get_artifact_manager().last()
            if last and last.session_id == sid:
                sess_dict = last.to_dict()
        if not sess_dict and not any(
            body.get(k) for k in ("before_image_path", "after_image_path", "video_path")
        ):
            last = get_artifact_manager().last()
            if last is None:
                raise HTTPException(status_code=404, detail="No spray sessions available")
            sess_dict = last.to_dict()

        before_path = body.get("before_image_path") or sess_dict.get("before_image_path")
        after_path = body.get("after_image_path") or sess_dict.get("after_image_path")
        video_path = body.get("video_path") or sess_dict.get("video_path")
        prefix = (body.get("name_prefix") or "task2").strip() or "task2"

        try:
            from ..gdrive_upload import upload_to_gdrive, gdrive_ready
        except ImportError as e:
            raise HTTPException(
                status_code=503, detail=f"Google Drive client unavailable: {e}"
            )
        if not gdrive_ready():
            raise HTTPException(
                status_code=503,
                detail="Google Drive not configured on Jetson (no OAuth token)",
            )

        root = os.path.realpath(SESSIONS_ROOT)

        def _safe(path: Optional[str]) -> Optional[str]:
            if not path:
                return None
            real = os.path.realpath(path)
            if not (real.startswith(root + os.sep) or real == root):
                return None
            return real if os.path.exists(real) else None

        task2_folder_id = os.environ.get("GDRIVE_TASK2_FOLDER_ID", "")

        def _target_number() -> int:
            for key in ("target_number", "target_num"):
                try:
                    n = int(body.get(key))
                    if n >= 1:
                        return n
                except (TypeError, ValueError):
                    pass
            for key in ("target_id",):
                try:
                    return max(1, int(body.get(key)) + 1)
                except (TypeError, ValueError):
                    pass
            try:
                if sess_dict.get("target_id") is not None:
                    return max(1, int(sess_dict.get("target_id")) + 1)
            except (TypeError, ValueError):
                pass
            return 1

        target_number = _target_number()

        targets = [
            (f"target_{target_number}_before", _safe(before_path)),
            (f"target_{target_number}_after", _safe(after_path)),
            (f"target_{target_number}_spray", _safe(video_path)),
        ]

        results = []
        ok = 0
        fail = 0
        for label, path in targets:
            if path is None:
                continue
            ext = os.path.splitext(path)[1] or ".bin"
            fname = f"{label}{ext}"
            try:
                file_id = upload_to_gdrive(path, fname, folder_id=task2_folder_id or None)
                if file_id:
                    ok += 1
                    results.append({"path": path, "name": fname, "file_id": file_id, "error": None})
                else:
                    fail += 1
                    results.append({"path": path, "name": fname, "file_id": None, "error": "upload returned no id"})
            except Exception as e:
                fail += 1
                results.append({"path": path, "name": fname, "file_id": None, "error": str(e)})

        return {"ok": ok, "fail": fail, "results": results}

    @app.get("/api/task/2/spray/artifact", tags=["Task 2", "Spray"])
    async def task2_spray_artifact(path: str = Query(..., description="Server-side artifact path")):
        """Stream an artifact file (image or video) recorded by the spray session.

        Restricted to files under ~/.nomad/spray_sessions to prevent path traversal.
        """
        from ..task2_spray_artifacts import SESSIONS_ROOT
        from fastapi.responses import FileResponse
        real = os.path.realpath(path)
        root = os.path.realpath(SESSIONS_ROOT)
        if not real.startswith(root + os.sep) and real != root:
            raise HTTPException(status_code=403, detail="Path outside session root")
        if not os.path.exists(real):
            raise HTTPException(status_code=404, detail="Artifact not found")
        return FileResponse(real)

    @app.post("/api/spray/abort", tags=["Spray"])
    async def abort_spray(request: Request):
        """Abort the current spray sequence."""
        spray_ctrl = getattr(request.app.state, "spray_controller", None)
        if not spray_ctrl:
            raise HTTPException(
                status_code=503, detail="Spray controller not initialized"
            )
        return spray_ctrl.abort()

    @app.get("/api/spray/calibration", tags=["Spray"])
    async def get_spray_calibration(request: Request):
        """Return Task 2 field calibration values for spray alignment."""
        spray_ctrl = getattr(request.app.state, "spray_controller", None)
        if not spray_ctrl:
            from ..spray_controller import SprayController
            return SprayController.get_calibration()
        return spray_ctrl.get_calibration()

    @app.post("/api/spray/calibration", tags=["Spray"])
    async def update_spray_calibration(request: Request):
        """
        Update Task 2 field calibration values.

        These values are intentionally field-tunable from Mission Planner so
        test-flight calibration can adjust the fixed firing range, water landing
        pixel, nozzle angle, spray duration, and visual-servo gains without code
        changes.
        """
        body = await _parse_request_json_object(request)
        persist = bool(body.pop("persist", True))
        spray_ctrl = getattr(request.app.state, "spray_controller", None)
        if not spray_ctrl:
            from ..spray_controller import SprayController
            return SprayController.update_calibration(body, persist=persist)
        return spray_ctrl.update_calibration(body, persist=persist)

    # ==================== Operational Mode (Section 9) ============================

    @app.get("/api/mode", tags=["Mode"])
    async def get_operational_mode(request: Request):
        """Get current operational mode and available modes."""
        mode_mgr = getattr(request.app.state, "mode_manager", None)
        if not mode_mgr:
            return {
                "current_mode": "outdoor_transit",
                "available_modes": [],
                "error": "Mode manager not initialized",
            }
        return {
            "status": mode_mgr.status.to_dict(),
            "available_modes": mode_mgr.get_available_modes(),
        }

    @app.post("/api/mode/set", tags=["Mode"])
    async def set_operational_mode(request: Request, mode: str = Query(...)):
        """
        Switch operational mode.

        Coordinates servo, VIO source, and obstacle avoidance. nvblox is an
        optional service controlled separately from Mission Planner.

        Valid modes: outdoor_transit, outdoor_survey, indoor_nav,
                     spray_approach, emergency
        """
        mode_mgr = getattr(request.app.state, "mode_manager", None)
        if not mode_mgr:
            raise HTTPException(status_code=503, detail="Mode manager not initialized")
        result = mode_mgr.switch_mode(mode)
        if not result["success"]:
            raise HTTPException(
                status_code=400, detail=result.get("error", "Switch failed")
            )
        return result

    # ==================== Obstacle Distance (NV-008) =============================

    @app.post("/api/obstacle_distance", tags=["Navigation"])
    async def receive_obstacle_distance(request: Request):
        """
        Receive obstacle distances from the ROS obstacle_distance_bridge
        and forward to ArduPilot via MAVLink OBSTACLE_DISTANCE message.

        Called by obstacle_distance_bridge.py at ~5 Hz with 72 angular
        sectors of distance data (5-degree increments).
        """
        body = await _parse_request_json_object(request)
        distances = body.get("distances", [])
        if len(distances) != 72:
            raise HTTPException(
                status_code=400,
                detail=f"Expected 72 distances, got {len(distances)}",
            )

        # SP-005: Override excluded sectors to max_distance so obstacle
        # avoidance ignores the sector containing the spray target
        excluded = getattr(request.app.state, "excluded_sectors", set())
        if excluded:
            max_dist = body.get("max_distance", 2000)
            distances = list(distances)
            for idx in excluded:
                if 0 <= idx < 72:
                    distances[idx] = max_dist

        mavlink_svc = request.app.state.mavlink_service
        if not mavlink_svc:
            raise HTTPException(status_code=503, detail="MAVLink service not available")

        increment = body.get("increment", 5)
        min_distance_cm = body.get("min_distance", 20)
        max_distance_cm = body.get("max_distance", 2000)

        success = mavlink_svc.send_obstacle_distance(
            distances=distances,
            increment=increment,
            min_distance=min_distance_cm,
            max_distance=max_distance_cm,
            angle_offset=body.get("angle_offset", 0),
            frame=body.get("frame", 0),
        )

        # Cache snapshot for UI consumers (nearest-obstacle readout, radar)
        valid_distances = [
            (i, d) for i, d in enumerate(distances)
            if d is not None and min_distance_cm <= d < max_distance_cm
        ]
        if valid_distances:
            nearest_idx, nearest_cm = min(valid_distances, key=lambda t: t[1])
            # Each sector spans `increment` degrees; angle is sector-center.
            nearest_bearing_deg = (nearest_idx * increment + increment / 2.0) % 360
        else:
            nearest_idx = None
            nearest_cm = None
            nearest_bearing_deg = None

        request.app.state.obstacle_distance_last = {
            "timestamp": time.time(),
            "distances": list(distances),
            "increment_deg": increment,
            "min_distance_cm": min_distance_cm,
            "max_distance_cm": max_distance_cm,
            "nearest_sector": nearest_idx,
            "nearest_distance_cm": nearest_cm,
            "nearest_bearing_deg": nearest_bearing_deg,
        }
        return {"success": success}

    @app.get("/api/obstacle_distance", tags=["Navigation"])
    async def get_obstacle_distance(request: Request):
        """
        Return the most recent obstacle distance snapshot received from
        obstacle_distance_bridge, along with nearest-obstacle summary
        fields for lightweight UI readouts.
        """
        snapshot = getattr(request.app.state, "obstacle_distance_last", None)
        if not snapshot:
            return {"valid": False, "message": "No obstacle distance data received"}

        age_s = time.time() - snapshot.get("timestamp", 0.0)
        return {
            "valid": age_s < 5.0,
            "age_seconds": age_s,
            "timestamp": snapshot.get("timestamp"),
            "increment_deg": snapshot.get("increment_deg"),
            "min_distance_cm": snapshot.get("min_distance_cm"),
            "max_distance_cm": snapshot.get("max_distance_cm"),
            "nearest_sector": snapshot.get("nearest_sector"),
            "nearest_distance_cm": snapshot.get("nearest_distance_cm"),
            "nearest_bearing_deg": snapshot.get("nearest_bearing_deg"),
            "distances": snapshot.get("distances", []),
        }

