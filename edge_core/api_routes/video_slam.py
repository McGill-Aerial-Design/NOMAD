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

def register_video_slam_routes(app, ctx) -> None:
    logger = ctx.logger
    _get_vio_snapshot = ctx.get_vio_snapshot
    _resolve_nvblox_empty_map_path = ctx.resolve_nvblox_empty_map_path
    _isaac_container_file_exists = ctx.isaac_container_file_exists
    _launch_nvblox_bridge_with_od = ctx.launch_nvblox_bridge_with_od

    def _call_ros2_service_in_isaac_container_or_raise(*args, **kwargs):
        return ctx.call_ros2_service_in_isaac_container_or_raise(*args, **kwargs)

    # ==================== Video Streaming Endpoints ====================
    # Isaac ROS H.264 video streaming with dynamic topic switching

    from ..video_stream_manager import get_video_stream_manager

    @app.get("/api/video/topics", tags=["Video"])
    async def get_video_topics():
        """
        List available ROS image topics from ZED camera.

        Returns topics with both full path and trimmed display names for UI:
        - Full: /zed/zed_node/rgb/color/rect/image
        - Display: zed: rgb/color/rect/image

        Use the full name when switching topics via POST /api/video/source.
        """
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(
                status_code=503, detail="Video stream manager not initialized"
            )

        loop = asyncio.get_event_loop()
        topics = await loop.run_in_executor(None, mgr.list_topics)
        return {"topics": [t.to_dict() for t in topics], "count": len(topics)}

    @app.get("/api/video/status", tags=["Video"])
    async def get_video_status():
        """
        Get current video stream status.

        Returns:
        - streaming: Whether the stream is active
        - current_topic: The ROS topic currently being streamed
        - rtsp_url: The constant RTSP URL (does not change on topic switch)
        - fps: Current frame rate
        - frame_count: Total frames streamed
        - error_count: Number of encoding/streaming errors
        """
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(
                status_code=503, detail="Video stream manager not initialized"
            )

        status = mgr.get_status()
        return status.to_dict()

    @app.post("/api/video/source", tags=["Video"])
    async def switch_video_source(
        topic: str = Query(..., description="ROS image topic to stream"),
    ):
        """
        Switch the video stream to a different ROS topic.

        The RTSP URL stays constant - only the content changes.
        Mission Planner video player does not need to reconnect.

        Available topics can be listed via GET /api/video/topics.

        Example:
            POST /api/video/source?topic=/zed/zed_node/left/image_rect_color
        """
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(
                status_code=503, detail="Video stream manager not initialized"
            )

        success = mgr.switch_topic(topic)
        if not success:
            raise HTTPException(status_code=500, detail="Failed to switch video source")

        status = mgr.get_status()
        return {
            "success": True,
            "topic": topic,
            "rtsp_url": mgr.get_rtsp_url(),
            "status": status.to_dict(),
        }

    @app.get("/api/video/source", tags=["Video"])
    async def get_video_source():
        """
        Get the current video source topic and RTSP URL.
        """
        mgr = get_video_stream_manager()
        if not mgr:
            return {"active": False, "topic": None, "rtsp_url": None}

        status = mgr.get_status()
        return {
            "active": status.streaming,
            "topic": status.current_topic,
            "rtsp_url": mgr.get_rtsp_url(),
        }

    @app.post("/api/video/start", tags=["Video"])
    async def start_video_stream(request: Request):
        """
        Start the video streaming pipeline.

        Launches the video relay node inside the Isaac ROS container.
        This is typically called automatically on startup.
        """
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(
                status_code=503, detail="Video stream manager not initialized"
            )

        success, reason = mgr.start_with_reason()
        if not success:
            raise HTTPException(
                status_code=500, detail=f"Failed to start video stream: {reason}"
            )

        overlay_enabled = False
        with request.app.state.detection_state_lock:
            detection_enabled = bool(
                getattr(request.app.state, "detection_enabled", True)
            )
        if detection_enabled:
            overlay_enabled = mgr.set_overlay(True)

        return {
            "success": True,
            "rtsp_url": mgr.get_rtsp_url(),
            "message": "Video pipeline started",
            "overlay_enabled": overlay_enabled,
        }

    @app.post("/api/video/stop", tags=["Video"])
    async def stop_video_stream():
        """
        Stop the video streaming pipeline.
        """
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(
                status_code=503, detail="Video stream manager not initialized"
            )

        success = mgr.stop()
        return {
            "success": success,
            "message": "Video pipeline stopped" if success else "Failed to stop",
        }

    @app.post("/api/video/restart", tags=["Video"])
    async def restart_video_stream(request: Request):
        """
        Restart the video streaming pipeline.

        Useful for recovery from errors or after container restart.
        """
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(
                status_code=503, detail="Video stream manager not initialized"
            )

        mgr.stop()
        import asyncio

        await asyncio.sleep(2)

        success, reason = mgr.start_with_reason()
        if not success:
            raise HTTPException(
                status_code=500, detail=f"Failed to restart video stream: {reason}"
            )

        overlay_enabled = False
        with request.app.state.detection_state_lock:
            detection_enabled = bool(
                getattr(request.app.state, "detection_enabled", True)
            )
        if detection_enabled:
            overlay_enabled = mgr.set_overlay(True)

        return {
            "success": True,
            "rtsp_url": mgr.get_rtsp_url(),
            "message": "Video pipeline restarted",
            "overlay_enabled": overlay_enabled,
        }

    @app.post("/api/video/bridges/start", tags=["Video"])
    async def start_video_bridges(request: Request):
        """
        Start video bridges (legacy endpoint for compatibility).

        This is an alias for /api/video/start since we simplified to a single bridge.
        Mission Planner Service Control Panel calls this endpoint.
        """
        return await start_video_stream(request)

    @app.get("/api/video/logs", tags=["Video"])
    async def get_video_logs(
        lines: int = Query(50, description="Number of log lines to return"),
    ):
        """
        Get recent logs from the video relay process.

        Useful for debugging video streaming issues.
        """
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(
                status_code=503, detail="Video stream manager not initialized"
            )

        logs = mgr.get_logs(lines)
        return {"logs": logs}

    @app.get("/api/video/bridges", tags=["Video"])
    async def get_video_bridges_status():
        """
        Get status of video bridges in legacy multi-bridge format.

        Returns status compatible with Mission Planner Service Control Panel.
        Maps our single video bridge to "primary" bridge.
        "secondary" bridge is marked as unavailable (we simplified to single bridge).
        """
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(
                status_code=503, detail="Video stream manager not initialized"
            )

        status = mgr.get_status()

        # Map our single bridge status to primary/secondary format
        # "playing" = streaming active, "stopped" = not streaming
        primary_state = "playing" if status.streaming else "stopped"

        return {
            "bridges": {
                "primary": {
                    "state": primary_state,
                    "topic": status.current_topic if status.streaming else None,
                    "fps": status.fps,
                    "frame_count": status.frame_count,
                    "error_count": status.error_count,
                },
                "secondary": {
                    "state": "unavailable",
                    "reason": "Single bridge configuration",
                },
            }
        }

    # ---- Video Overlay (ROS2 detection bounding boxes on stream) ----

    @app.post("/api/video/overlay/enable", tags=["Video"])
    async def enable_video_overlay():
        """
        Enable ROS2 detection overlay on the video stream.

        When enabled, the video bridge draws bounding boxes from the
        ROS2 detection pipeline directly onto the RTSP frames in real time.
        Toggle off with POST /api/video/overlay/disable.
        """
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(
                status_code=503, detail="Video stream manager not initialized"
            )

        success = mgr.set_overlay(True)
        if not success:
            raise HTTPException(
                status_code=500,
                detail="Failed to enable overlay (video bridge not running?)",
            )
        return {"success": True, "overlay": True}

    @app.post("/api/video/overlay/disable", tags=["Video"])
    async def disable_video_overlay():
        """Disable ROS2 detection overlay on the video stream."""
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(
                status_code=503, detail="Video stream manager not initialized"
            )

        success = mgr.set_overlay(False)
        if not success:
            raise HTTPException(status_code=500, detail="Failed to disable overlay")
        return {"success": True, "overlay": False}

    @app.post("/api/video/overlay/detectors", tags=["Video"])
    async def set_video_overlay_detectors(
        task1: bool = Query(False, description="Enable Task 1 (HSV color circles) detector"),
        task2: bool = Query(False, description="Enable Task 2 (shape circles) detector"),
    ):
        """Independently enable/disable the Task 1 and Task 2 overlay detectors.
        Disable both to skip overlay processing entirely (saves CPU)."""
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(status_code=503, detail="Video stream manager not initialized")
        ok = mgr.set_overlay_detectors(task1, task2)
        if not ok:
            raise HTTPException(status_code=500, detail="Failed to set detectors")
        return {"success": True, "task1_enabled": task1, "task2_enabled": task2}

    @app.post("/api/video/overlay/mode", tags=["Video"])
    async def set_video_overlay_mode(
        mode: str = Query(..., description="Overlay detector mode: task1 | task2")
    ):
        """Switch the live overlay detector. task1=color HSV circles,
        task2=color-agnostic shape (Hough+contour) circles."""
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(status_code=503, detail="Video stream manager not initialized")
        ok = mgr.set_overlay_mode(mode)
        if not ok:
            raise HTTPException(status_code=400, detail=f"Failed to set overlay mode: {mode}")
        return {"success": True, "mode": mode}

    @app.get("/api/video/overlay/status", tags=["Video"])
    async def get_video_overlay_status():
        """Get current overlay status (enabled/disabled and detection count)."""
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(
                status_code=503, detail="Video stream manager not initialized"
            )

        return mgr.get_overlay_status()

    # ==================== SLAM 3D Mesh Endpoints ====================
    # These endpoints stream nvblox 3D mesh data for Mission Planner visualization
    # Mesh data is received from ros_http_bridge running inside the Isaac ROS container

    @app.get("/api/task/2/slam/mesh/mode", tags=["Task 2", "SLAM"])
    async def get_slam_mesh_mode(request: Request):
        """Get current runtime mesh output mode requested by Edge Core."""
        mode = "voxel"
        request.app.state.slam_mesh_output_mode = mode
        return {
            "mesh_output_mode": mode,
            "supported_modes": ["voxel"],
            "runtime_toggle": False,
        }

    @app.post("/api/task/2/slam/mesh/mode", tags=["Task 2", "SLAM"])
    async def set_slam_mesh_mode(
        request: Request, mode: str = Query(..., description="Mesh output mode: voxel")
    ):
        """Set runtime mesh output mode for ros_http_bridge without restart."""
        normalized = "voxel"

        previous = (
            str(getattr(request.app.state, "slam_mesh_output_mode", "voxel"))
            .strip()
            .lower()
        )
        request.app.state.slam_mesh_output_mode = normalized
        return {
            "success": True,
            "mesh_output_mode": normalized,
            "previous_mesh_output_mode": previous,
            "applies_without_restart": False,
            "bridge_poll_interval_s": 0.0,
        }

    @app.post("/api/task/2/slam/mesh/update", tags=["Task 2", "SLAM"])
    async def update_slam_mesh(request: Request):
        """
        Receive mesh update from ros_http_bridge (internal use).

        This endpoint receives mesh data from the ros_http_bridge running
        inside the Isaac ROS container. The mesh data is stored and served
        to Mission Planner via the GET /api/task/2/slam/mesh endpoint.

        Posted by: ros_http_bridge.py (inside Isaac ROS container)
        """
        # Size check -- raised from 5 MB to 32 MB to avoid dropping dense
        # nvblox mesh updates in large environments. Oversize payloads still
        # return 413 but now also bump a visible counter the UI can surface
        # instead of disappearing silently.
        max_payload_bytes = 32 * 1024 * 1024
        try:
            raw = await request.body()
        except Exception:
            return JSONResponse({"error": "Invalid payload"}, status_code=400)
        if len(raw) > max_payload_bytes:
            request.app.state.slam_mesh_drops = (
                getattr(request.app.state, "slam_mesh_drops", 0) + 1
            )
            request.app.state.slam_mesh_last_drop_bytes = len(raw)
            request.app.state.slam_mesh_last_drop_reason = "payload_too_large"
            logger.warning(
                "mesh/update dropped: payload %d bytes > limit %d (drop count %d)",
                len(raw),
                max_payload_bytes,
                request.app.state.slam_mesh_drops,
            )
            return JSONResponse(
                {
                    "error": "Payload too large",
                    "bytes": len(raw),
                    "limit": max_payload_bytes,
                },
                status_code=413,
            )

        try:
            content_type = request.headers.get("content-type", "application/json")
            if "msgpack" in content_type and MSGPACK_AVAILABLE:
                mesh_data = msgpack.unpackb(raw, raw=False)
            else:
                mesh_data = json.loads(raw)
        except Exception:
            return JSONResponse({"error": "Invalid payload"}, status_code=400)

        # Validate required field: mode
        mode = mesh_data.get("mode")
        if mode not in ("block", "voxel"):
            return JSONResponse(
                {"error": "mode must be 'block' or 'voxel'"}, status_code=400
            )

        # Validate required list for the chosen mode
        if mode == "block" and not isinstance(mesh_data.get("blocks"), list):
            return JSONResponse({"error": "blocks must be a list"}, status_code=400)
        if mode == "voxel" and not isinstance(mesh_data.get("voxels"), list):
            return JSONResponse({"error": "voxels must be a list"}, status_code=400)
        # Validate optional numeric fields
        for field in ("block_size", "voxel_size"):
            if field in mesh_data and not isinstance(mesh_data[field], (int, float)):
                return JSONResponse(
                    {"error": f"{field} must be a number"}, status_code=400
                )

        try:
            # Store in app state
            if not hasattr(request.app.state, "slam_mesh_data"):
                request.app.state.slam_mesh_data = {}

            # Compute item count based on mode
            item_count = len(mesh_data.get("blocks", mesh_data.get("voxels", [])))
            total_items = mesh_data.get(
                "total_blocks", mesh_data.get("total_voxels", 0)
            )

            # Canonical SLAM frame identifier is "map". Anything else
            # from the bridge is a contract violation -- normalize and log once
            # so silent drift cannot corrupt Mission Planner visualization.
            incoming_frame = mesh_data.get("frame_id", "map")
            if incoming_frame != "map":
                if not getattr(
                    request.app.state, "_mesh_ingest_frame_mismatch_logged", False
                ):
                    logger.warning(
                        "mesh/update frame_id mismatch: got %r, expected 'map'",
                        incoming_frame,
                    )
                    request.app.state._mesh_ingest_frame_mismatch_logged = True

            request.app.state.slam_mesh_data = {
                "mesh": mesh_data,
                "received_at": datetime.now(timezone.utc).isoformat(),
                "block_count": item_count,
                "total_blocks": total_items,
                "mode": mode,
                "frame_id": "map",
            }

            # Store drone pose from mesh data (from TF lookup in ros_http_bridge)
            if "drone_position" in mesh_data and mesh_data["drone_position"]:
                request.app.state.slam_mesh_data["drone_position"] = mesh_data[
                    "drone_position"
                ]
            if "drone_attitude" in mesh_data and mesh_data["drone_attitude"]:
                request.app.state.slam_mesh_data["drone_attitude"] = mesh_data[
                    "drone_attitude"
                ]

            # Increment version counter for delta tracking
            request.app.state.slam_mesh_version = (
                getattr(request.app.state, "slam_mesh_version", 0) + 1
            )

            return {"status": "ok", "items_received": item_count, "mode": mode}

        except Exception as e:
            logger.error(f"SLAM mesh update error: {e}")
            raise HTTPException(status_code=400, detail=str(e))

    @app.get("/api/task/2/slam/mesh", tags=["Task 2", "SLAM"])
    async def get_slam_mesh(
        request: Request, format: str = Query("full", description="'full' or 'summary'")
    ):
        """
        Get current 3D SLAM mesh from nvblox.

        This endpoint returns the real-time 3D occupancy map built by nvblox
        from ZED camera depth data. Used by Mission Planner for 3D visualization.

        Mesh data is received from ros_http_bridge running inside the Isaac ROS
        container via POST /api/task/2/slam/mesh/update.

        Args:
            format: 'full' for complete mesh data, 'summary' for metadata only

        Returns:
            - mesh: The mesh data with voxels/blocks and optional colors
            - drone_position: Current VIO position
            - drone_attitude: Current VIO orientation (roll, pitch, yaw)
            - timestamp: ISO format timestamp

        Update Rate: Bounded by configured bridge send rate (default 30 Hz); effective rate may vary
        """
        try:
            # Check for stored mesh data from ros_http_bridge
            if (
                hasattr(request.app.state, "slam_mesh_data")
                and request.app.state.slam_mesh_data
            ):
                stored = request.app.state.slam_mesh_data

                if format == "summary":
                    result = {
                        "available": True,
                        "timestamp": stored.get("received_at"),
                        "block_count": stored.get("block_count", 0),
                        "total_blocks": stored.get("total_blocks", 0),
                        "mode": stored.get("mode", "voxel"),
                        "drops": getattr(request.app.state, "slam_mesh_drops", 0),
                        "last_drop_bytes": getattr(
                            request.app.state, "slam_mesh_last_drop_bytes", 0
                        ),
                        "last_drop_reason": getattr(
                            request.app.state, "slam_mesh_last_drop_reason", None
                        ),
                    }
                else:
                    result = {
                        "available": True,
                        "timestamp": stored.get("received_at"),
                        "mesh": stored.get("mesh"),
                    }

                # Add drone pose from mesh data (TF lookup from ros_http_bridge)
                if stored.get("drone_position"):
                    result["drone_position"] = stored["drone_position"]
                if stored.get("drone_attitude"):
                    result["drone_attitude"] = stored["drone_attitude"]

                # Fallback to ROS-frame VIO if mesh didn't include pose
                # (must use slam_vio_ros_frame, not external_vio_state which is NED)
                if "drone_position" not in result:
                    ros_vio = _get_vio_snapshot()["slam_vio_ros_frame"]
                    if ros_vio:
                        result["drone_position"] = {
                            "x": ros_vio.get("x", 0),
                            "y": ros_vio.get("y", 0),
                            "z": ros_vio.get("z", 0),
                        }
                        result["drone_attitude"] = {
                            "roll": (
                                ros_vio.get("body_roll")
                                if ros_vio.get("body_roll") is not None
                                else ros_vio.get("roll", 0)
                            ),
                            "pitch": (
                                ros_vio.get("body_pitch")
                                if ros_vio.get("body_pitch") is not None
                                else ros_vio.get("pitch", 0)
                            ),
                            "yaw": (
                                ros_vio.get("body_yaw")
                                if ros_vio.get("body_yaw") is not None
                                else ros_vio.get("yaw", 0)
                            ),
                        }

                return result

            return {
                "available": False,
                "error": "No mesh data available",
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "mesh": None,
                "drone_position": None,
                "drone_attitude": None,
            }
        except Exception as e:
            logger.error(f"SLAM mesh endpoint error: {e}")
            return {
                "available": False,
                "error": str(e),
                "timestamp": datetime.now(timezone.utc).isoformat(),
            }

    @app.get("/api/task/2/slam/mesh/delta", tags=["Task 2", "SLAM"])
    async def get_slam_mesh_delta(request: Request):
        """
        Get incremental mesh updates (delta) since last request.

        Uses a version counter on app.state that is bumped on each
        POST to /mesh/update.  Pass ?since=N with the last known version
        to receive only newer data.

        Returns:
            - changed: Whether new data is available
            - version: Current mesh version counter
            - mesh: Full mesh payload (only when changed is True)
        """
        try:
            since = int(request.query_params.get("since", 0))
        except (ValueError, TypeError):
            since = 0

        current_version = getattr(request.app.state, "slam_mesh_version", 0)

        if since >= current_version:
            return {
                "available": True,
                "changed": False,
                "version": current_version,
                "timestamp": datetime.now(timezone.utc).isoformat(),
            }

        mesh_state = getattr(request.app.state, "slam_mesh_data", None)
        if not mesh_state:
            return {
                "available": True,
                "changed": False,
                "version": current_version,
                "timestamp": datetime.now(timezone.utc).isoformat(),
            }

        return {
            "available": True,
            "changed": True,
            "version": current_version,
            "mesh": mesh_state.get("mesh"),
            "drone_position": mesh_state.get("drone_position"),
            "drone_attitude": mesh_state.get("drone_attitude"),
            "timestamp": mesh_state.get(
                "received_at", datetime.now(timezone.utc).isoformat()
            ),
        }

    @app.get("/api/task/2/slam/status", tags=["Task 2", "SLAM"])
    async def get_slam_status(request: Request):
        """
        Get nvblox SLAM system status.

        Returns status information about the 3D mapping pipeline.
        """
        # Check for stored mesh data from ros_http_bridge first
        if (
            hasattr(request.app.state, "slam_mesh_data")
            and request.app.state.slam_mesh_data
        ):
            stored = request.app.state.slam_mesh_data
            return {
                "available": True,
                "running": True,
                "source": "ros_http_bridge",
                "block_count": stored.get("block_count", 0),
                "total_voxels": stored.get("total_blocks", 0),
                "last_update": stored.get("received_at"),
            }

        return {
            "available": False,
            "running": False,
            "error": "No mesh data available",
        }

    @app.post("/api/task/2/slam/empty-map/create", tags=["Task 2", "SLAM"])
    async def create_slam_empty_map(file_path: Optional[str] = Query(default=None)):
        """
        Save the current nvblox map as the baseline map used for non-restart clears.

        Recommended workflow:
        1) Ensure the environment is in the desired "empty" baseline state.
        2) Call this endpoint once.
        3) Future /api/task/2/slam/clear calls can reload this baseline map.
        """
        target_path = _resolve_nvblox_empty_map_path(file_path)
        message = _call_ros2_service_in_isaac_container_or_raise(
            service_name="/nvblox_node/save_map",
            service_type="nvblox_msgs/srv/FilePath",
            request_payload={"file_path": target_path},
            timeout_s=30.0,
        )

        return {
            "success": True,
            "message": message
            or "Baseline map saved. /api/task/2/slam/clear can now load it without relaunch.",
            "file_path": target_path,
        }

    @app.post("/api/task/2/slam/clear", tags=["Task 2", "SLAM"])
    async def clear_slam_mesh(
        request: Request,
        relaunch_if_needed: bool = False,
        empty_map_path: Optional[str] = None,
        prefer_load_map: bool = True,
        auto_create_empty_map_if_missing: bool = True,
    ):
        """
        Clear the current SLAM mesh and reset the map without a full relaunch when possible.

        Strategy:
        1) Ensure baseline empty-map file exists (auto-create if missing).
        2) Load baseline map snapshot (manual-reset flow).
        3) Fallback to native nvblox clear service.
        4) Fallback to ZED tracking reset services.
        5) Relaunch only when explicitly requested via relaunch_if_needed=true.
        """
        nvblox_cleared = False
        nvblox_message = ""
        nvblox_error_status = 503
        clear_strategy = "none"
        clear_warnings: list[str] = []

        def _load_empty_map_and_reset_or_raise(
            path_hint: Optional[str],
        ) -> tuple[str, str]:
            baseline_path = _resolve_nvblox_empty_map_path(path_hint)
            bootstrap_messages: list[str] = []

            if auto_create_empty_map_if_missing and not _isaac_container_file_exists(
                baseline_path
            ):
                bootstrap_clear_msg = _call_ros2_service_in_isaac_container_or_raise(
                    service_name="/nvblox_node/clear_map",
                    service_type="std_srvs/srv/Empty",
                    request_payload={},
                    timeout_s=10.0,
                )
                bootstrap_save_msg = _call_ros2_service_in_isaac_container_or_raise(
                    service_name="/nvblox_node/save_map",
                    service_type="nvblox_msgs/srv/FilePath",
                    request_payload={"file_path": baseline_path},
                    timeout_s=20.0,
                )
                bootstrap_messages = [
                    part
                    for part in [
                        bootstrap_clear_msg,
                        bootstrap_save_msg
                        or f"Created baseline empty map: {baseline_path}",
                    ]
                    if part
                ]
                logger.info(
                    f"Created missing baseline empty map for SLAM clear: {baseline_path}"
                )

            load_msg = _call_ros2_service_in_isaac_container_or_raise(
                service_name="/nvblox_node/load_map",
                service_type="nvblox_msgs/srv/FilePath",
                request_payload={"file_path": baseline_path},
                timeout_s=20.0,
            )
            reset_msg = _call_ros2_service_in_isaac_container_or_raise(
                service_name="/zed/zed_node/reset_pos_tracking",
                service_type="std_srvs/srv/Trigger",
                request_payload={},
                timeout_s=10.0,
            )
            combined_message = "; ".join(
                part for part in [*bootstrap_messages, load_msg, reset_msg] if part
            )
            if not combined_message:
                combined_message = f"Loaded baseline map: {baseline_path}"
            return baseline_path, combined_message

        # Preferred manual-reset path: load a known-empty baseline map.
        if prefer_load_map:
            try:
                baseline_path, combined_message = _load_empty_map_and_reset_or_raise(
                    empty_map_path
                )
                nvblox_cleared = True
                clear_strategy = "load_empty_map"
                nvblox_message = combined_message
                logger.info(
                    f"nvblox map reset by loading baseline map: {baseline_path}"
                )
            except HTTPException as e:
                clear_warnings.append(f"load baseline map failed: {e.detail}")
            except Exception as e:
                clear_warnings.append(f"load baseline map failed: {e}")

        # Fallback path: native nvblox clear service (no restart).
        if not nvblox_cleared:
            try:
                clear_msg = _call_ros2_service_in_isaac_container_or_raise(
                    service_name="/nvblox_node/clear_map",
                    service_type="std_srvs/srv/Empty",
                    request_payload={},
                    timeout_s=10.0,
                )
                nvblox_cleared = True
                clear_strategy = "nvblox_clear_map_service"
                nvblox_message = clear_msg or "nvblox clear_map service succeeded"
                logger.info("nvblox map cleared via /nvblox_node/clear_map")
            except HTTPException as e:
                clear_warnings.append(f"clear_map service failed: {e.detail}")
            except Exception as e:
                clear_warnings.append(f"clear_map service failed: {e}")

        # Optional fallback for callers that disable the load-map-first behavior.
        if not nvblox_cleared and not prefer_load_map:
            try:
                baseline_path, combined_message = _load_empty_map_and_reset_or_raise(
                    empty_map_path
                )
                nvblox_cleared = True
                clear_strategy = "load_empty_map_fallback"
                nvblox_message = combined_message
                logger.info(
                    f"nvblox map reset by loading baseline map: {baseline_path}"
                )
            except HTTPException as e:
                clear_warnings.append(f"load baseline map failed: {e.detail}")
            except Exception as e:
                clear_warnings.append(f"load baseline map failed: {e}")

        # Fallback path: reset tracking/odometry in-place to start a fresh map.
        if not nvblox_cleared:
            for service_name in [
                "/zed/zed_node/reset_pos_tracking",
                "/zed/zed_node/reset_odometry",
            ]:
                try:
                    reset_msg = _call_ros2_service_in_isaac_container_or_raise(
                        service_name=service_name,
                        service_type="std_srvs/srv/Trigger",
                        request_payload={},
                        timeout_s=10.0,
                    )
                    nvblox_cleared = True
                    clear_strategy = "zed_tracking_reset"
                    nvblox_message = reset_msg or f"{service_name} succeeded"
                    logger.info(f"Requested non-restart SLAM reset via {service_name}")
                    break
                except HTTPException as e:
                    clear_warnings.append(f"{service_name} failed: {e.detail}")
                except Exception as e:
                    clear_warnings.append(f"{service_name} failed: {e}")

        # Optional hard fallback: relaunch stack only if explicitly requested.
        if not nvblox_cleared and relaunch_if_needed:
            try:
                with request.app.state.detection_state_lock:
                    detection_enabled = bool(
                        getattr(request.app.state, "detection_enabled", True)
                    )

                relaunch_result = _launch_nvblox_bridge_with_od(
                    enable_od=detection_enabled
                )
                if relaunch_result.get("success"):
                    nvblox_cleared = True
                    clear_strategy = "stack_relaunch"
                    nvblox_message = relaunch_result.get(
                        "message", "nvblox stack relaunched"
                    )
                    logger.info("nvblox map reset via stack relaunch")
                else:
                    nvblox_message = relaunch_result.get(
                        "error", "Failed to relaunch nvblox stack"
                    )
                    logger.warning(f"Failed to reset nvblox map: {nvblox_message}")
            except HTTPException as e:
                logger.warning(f"Failed to reset nvblox map: {e.detail}")
                nvblox_message = str(e.detail)
                nvblox_error_status = e.status_code
            except Exception as e:
                logger.warning(f"Failed to reset nvblox map: {e}")
                nvblox_message = str(e)

        if not nvblox_cleared and not nvblox_message:
            nvblox_message = "No non-restart map clear service is available in current nvblox runtime"

        # Preserve voxel size from the previous mesh data if available.
        prev_voxel_size = 0.05
        if hasattr(request.app.state, "slam_mesh_data") and isinstance(
            request.app.state.slam_mesh_data, dict
        ):
            prev_mesh = request.app.state.slam_mesh_data.get("mesh")
            if isinstance(prev_mesh, dict):
                prev_voxel_size = prev_mesh.get("voxel_size", 0.05)

        # Replace with a cleared-but-valid state so the GET endpoint
        # still returns available=True and clients see clear=True
        request.app.state.slam_mesh_data = {
            "mesh": {
                "voxels": [],
                "voxel_size": prev_voxel_size,
                "total_voxels": 0,
                "mode": "voxel",
                "clear": True,
            },
            "received_at": datetime.now(timezone.utc).isoformat(),
            "block_count": 0,
            "total_blocks": 0,
            "mode": "voxel",
        }
        request.app.state.slam_mesh_version = (
            getattr(request.app.state, "slam_mesh_version", 0) + 1
        )

        response_payload = {
            "success": nvblox_cleared,
            "nvblox_cleared": nvblox_cleared,
            "nvblox_message": nvblox_message,
            "clear_strategy": clear_strategy,
            "warnings": clear_warnings,
            "message": "Mesh cache cleared"
            + (", nvblox map reset" if nvblox_cleared else " (nvblox clear failed)"),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        if not nvblox_cleared:
            return JSONResponse(
                status_code=nvblox_error_status, content=response_payload
            )
        return response_payload

