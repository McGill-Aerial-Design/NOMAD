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

from ..video_stream_manager import get_video_stream_manager

def register_detection_routes(app, ctx) -> None:
    logger = ctx.logger
    _probe_isaac_runtime_state = ctx.probe_isaac_runtime_state
    _docker_exec_bash_success = ctx.docker_exec_bash_success
    _launch_nvblox_bridge_with_od = ctx.launch_nvblox_bridge_with_od
    _parse_request_json_object = ctx.parse_request_json_object
    _apply_detections_update = ctx.apply_detections_update

    # ==================== Object Detection Endpoints ====================
    # ROS2 target detections via ZED custom OD pipeline.
    # Detections are received from ros_http_bridge and served to Mission Planner.

    @app.post("/api/detections/start", tags=["Detections"])
    async def start_detections(request: Request):
        """
        Start ROS2 target detection by relaunching nvblox with OD enabled.

        This keeps launch behavior consistent with NOMAD's custom launch file.
        """
        runtime_state = _probe_isaac_runtime_state(force_refresh=True)
        stack_running = (
            runtime_state.get("container_running", False)
            and runtime_state.get("nvblox_running", False)
            and runtime_state.get("bridge_running", False)
        )

        if stack_running:
            service_ready = _docker_exec_bash_success(
                "nomad_isaac_ros",
                "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
                "source /workspaces/isaac_ros-dev/install/setup.bash >/dev/null 2>&1; "
                "ROS2CLI_DISABLE_DAEMON=1 ros2 service list 2>/dev/null | "
                "grep -q '/target_localizer/capture_target'",
                timeout_s=6,
            )
            rgb_pub_ready = _docker_exec_bash_success(
                "nomad_isaac_ros",
                "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
                "source /workspaces/isaac_ros-dev/install/setup.bash >/dev/null 2>&1; "
                "ROS2CLI_DISABLE_DAEMON=1 ros2 topic info /zed/zed_node/rgb/color/rect/image 2>/dev/null | "
                "grep -Eq 'Publisher count: [1-9]'",
                timeout_s=6,
            )
            depth_pub_ready = _docker_exec_bash_success(
                "nomad_isaac_ros",
                "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
                "source /workspaces/isaac_ros-dev/install/setup.bash >/dev/null 2>&1; "
                "ROS2CLI_DISABLE_DAEMON=1 ros2 topic info /zed/zed_node/depth/depth_registered 2>/dev/null | "
                "grep -Eq 'Publisher count: [1-9]'",
                timeout_s=6,
            )

            if bool(service_ready) and bool(rgb_pub_ready) and bool(depth_pub_ready):
                with request.app.state.detection_state_lock:
                    request.app.state.detection_enabled = True

                mgr = get_video_stream_manager()
                if mgr and not mgr.set_overlay(True):
                    logger.warning(
                        "Detections already running, but video overlay could not be enabled"
                    )

                return {
                    "success": True,
                    "message": "Detections already running; skipped relaunch.",
                    "detection_enabled": True,
                    "already_running": True,
                }

        result = _launch_nvblox_bridge_with_od(enable_od=True)
        if result.get("success"):
            with request.app.state.detection_state_lock:
                request.app.state.detection_enabled = True
                request.app.state.detection_last_update = 0.0
                request.app.state.detected_objects = []

            # Keep operator UX consistent: enabling detections should enable overlay.
            mgr = get_video_stream_manager()
            if mgr:
                if mgr.set_overlay(True):
                    logger.info("Video overlay enabled after detections/start")
                else:
                    logger.warning(
                        "Detections started, but video overlay could not be enabled"
                    )
        return result

    @app.post("/api/detections/stop", tags=["Detections"])
    async def stop_detections(request: Request):
        """
        Stop ROS2 target detection by relaunching nvblox with OD disabled.

        nvblox mapping remains available; only custom object detection is disabled.
        """
        result = _launch_nvblox_bridge_with_od(enable_od=False)
        if result.get("success"):
            with request.app.state.detection_state_lock:
                request.app.state.detection_enabled = False
                request.app.state.detection_last_update = 0.0
                request.app.state.detected_objects = []

            mgr = get_video_stream_manager()
            if mgr:
                mgr.set_overlay(False)
        return result

    @app.get("/api/detections/status", tags=["Detections"])
    async def get_detections_status(request: Request):
        """Get ROS2 detection runtime status for Mission Planner polling."""
        import time as _time

        with request.app.state.detection_state_lock:
            detection_enabled = bool(
                getattr(request.app.state, "detection_enabled", True)
            )
            last_update = request.app.state.detection_last_update
            current_count = len(request.app.state.detected_objects)
            history_count = len(request.app.state.detection_history)
        age_seconds = _time.time() - last_update if last_update > 0 else None
        fresh_stream = age_seconds is not None and age_seconds <= 3.0

        return {
            "detection_enabled": detection_enabled,
            "fresh_stream": fresh_stream,
            "age_seconds": age_seconds,
            "current_count": current_count,
            "history_count": history_count,
        }

    @app.post("/api/detections/update", tags=["Detections"])
    async def update_detections(request: Request):
        """
        Receive object detections from ROS-HTTP bridge.

        Called by ros_http_bridge at ~5Hz with current frame detections.
        Stores current detections and adds new unique targets to history.
        Also receivable via ZMQ IPC (preferred low-latency path).
        """
        body = await _parse_request_json_object(request)
        detections = body.get("detections")
        if not isinstance(detections, list):
            raise HTTPException(
                status_code=400,
                detail="Invalid payload: 'detections' must be a list of objects",
            )
        if any(not isinstance(det, dict) for det in detections):
            raise HTTPException(
                status_code=400,
                detail="Invalid payload: each item in 'detections' must be an object",
            )
        source_timestamp = body.get("source_timestamp")
        _apply_detections_update(detections, source_timestamp=source_timestamp)
        with request.app.state.detection_state_lock:
            history_size = len(request.app.state.detection_history)
        return {"accepted": len(detections), "history_size": history_size}

    @app.get("/api/detections", tags=["Detections"])
    async def get_detections(request: Request):
        """
        Get current object detections and persistent detection history.

        Returns both the latest frame detections and the full history
        of unique detected objects with 3D positions.
        """
        import time as _time

        with request.app.state.detection_state_lock:
            current = list(request.app.state.detected_objects)
            history = list(request.app.state.detection_history)
            last_update = request.app.state.detection_last_update

        return {
            "current": {
                "count": len(current),
                "detections": current,
                "age_seconds": _time.time() - last_update if last_update > 0 else None,
            },
            "history": {
                "count": len(history),
                "detections": history,
            },
        }

    @app.get("/api/detections/summary", tags=["Detections"])
    async def get_detection_summary(request: Request):
        """
        Get a summary of detected objects grouped by class label.

        Useful for Task 1 AI description to know which targets are present.
        """
        with request.app.state.detection_state_lock:
            history = list(request.app.state.detection_history)

        # Group by label
        by_label = {}
        for det in history:
            label = det.get("label", "unknown")
            if label not in by_label:
                by_label[label] = {
                    "count": 0,
                    "avg_confidence": 0.0,
                    "positions": [],
                }
            entry = by_label[label]
            entry["count"] += 1
            entry["avg_confidence"] += det.get("confidence", 0)
            if det.get("x") is not None:
                entry["positions"].append(
                    {
                        "x": det["x"],
                        "y": det["y"],
                        "z": det["z"],
                    }
                )

        # Compute averages
        for label, entry in by_label.items():
            if entry["count"] > 0:
                entry["avg_confidence"] /= entry["count"]

        return {
            "total_unique_targets": len(history),
            "by_class": by_label,
        }

    @app.delete("/api/detections/history", tags=["Detections"])
    async def clear_detection_history(request: Request):
        """Clear the persistent detection history."""
        with request.app.state.detection_state_lock:
            count = len(request.app.state.detection_history)
            request.app.state.detection_history = []
        return {"cleared": count}

