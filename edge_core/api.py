"""
NOMAD Edge Core - REST API

Provides HTTP endpoints for system status, health monitoring,
Task 1/Task 2 operations, and video streaming.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

import asyncio
import hmac
import ipaddress
import json
import logging
import math
import os
import re
import shlex
import shutil
import subprocess
import threading
import time
from datetime import datetime, timezone
from typing import TYPE_CHECKING, Any, Optional

from fastapi import FastAPI, HTTPException, Request, WebSocket, Query
from fastapi.encoders import jsonable_encoder
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.websockets import WebSocketDisconnect
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

from .state import StateManager
from .api_context import ApiRouteContext

try:
    from .ipc import (
        DEFAULT_ROS_HIGH_RATE_ENDPOINT,
        HIGH_RATE_MSG_TYPE_CMD_VEL,
        HIGH_RATE_MSG_TYPE_DETECTIONS,
        HIGH_RATE_MSG_TYPE_VIO,
        IPCMessage,
        ZMQSubscriber,
    )

    IPC_AVAILABLE = True
    IPC_IMPORT_ERROR = ""
except Exception as e:
    IPC_AVAILABLE = False
    IPC_IMPORT_ERROR = str(e)
    DEFAULT_ROS_HIGH_RATE_ENDPOINT = "tcp://127.0.0.1:5557"
    HIGH_RATE_MSG_TYPE_VIO = "ROS_VIO_UPDATE"
    HIGH_RATE_MSG_TYPE_CMD_VEL = "ROS_CMD_VEL"
    HIGH_RATE_MSG_TYPE_DETECTIONS = "ROS_DETECTIONS"
    IPCMessage = Any  # type: ignore
    ZMQSubscriber = Any  # type: ignore

if TYPE_CHECKING:
    from .health_monitor import JetsonHealthMonitor
    from .isaac_ros_bridge import IsaacROSBridge
    from .nav_controller import NavController

logger = logging.getLogger("edge_core.api")


from .api_models import (
    GDriveUploadRequest,
    GDriveUploadResponse,
    NavPositionRequest,
    NavVelocityRequest,
    Task1CapturesList,
    Task2HitRequest,
    TerminalCommandRequest,
    TerminalCommandResponse,
    TerminalExecRequest,
    VIOAreaLoadRequest,
    VIOAreaSaveRequest,
    VIOUpdateRequest,
)

# ==================== Setter Functions for Dependency Injection ====================
# These receive app parameter and store services in app.state (thread-safe)


def set_health_monitor(app: FastAPI, monitor: "JetsonHealthMonitor") -> None:
    """Register health monitor with API via app.state."""
    app.state.health_monitor = monitor


def set_isaac_bridge(app: FastAPI, bridge: "IsaacROSBridge") -> None:
    """Register Isaac ROS bridge with API via app.state."""
    app.state.isaac_bridge = bridge


def set_tailscale_manager(app: FastAPI, manager: Any) -> None:
    """Register Tailscale manager with API via app.state."""
    app.state.tailscale_manager = manager


def set_network_monitor(app: FastAPI, monitor: Any) -> None:
    """Register network monitor with API via app.state."""
    app.state.network_monitor = monitor


def set_nav_controller(app: FastAPI, controller: "NavController") -> None:
    """Register navigation controller with API via app.state."""
    app.state.nav_controller = controller


def set_camera_service(app: FastAPI, camera_service: Any) -> None:
    """Register ZED camera service with API via app.state (unused, kept for compat)."""
    app.state.camera_service = camera_service


def create_app(state_manager: StateManager) -> FastAPI:
    """
    Create the FastAPI application for Edge Core.

    Args:
        state_manager: StateManager instance for system state

    Returns:
        Configured FastAPI application
    """
    app = FastAPI(
        title="NOMAD Edge Core API",
        description="Drone-side API for NOMAD (AEAC 2026) - Task 1 & Task 2 Operations",
        version="1.0.0",
        docs_url="/docs",
        redoc_url="/redoc",
    )

    # CORS: restrict to GCS origin when configured, otherwise allow all (development)
    gcs_origin = os.environ.get("GCS_ORIGIN")
    allowed_origins = [gcs_origin] if gcs_origin else ["*"]
    allow_credentials = bool(gcs_origin and gcs_origin != "*")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=allowed_origins,
        allow_credentials=allow_credentials,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # API key authentication middleware
    # If NOMAD_API_KEY is set, require X-API-Key header on non-exempt endpoints.
    # If NOMAD_API_KEY is not set, skip authentication (development mode).
    _NOMAD_API_KEY = (os.environ.get("NOMAD_API_KEY") or "").strip() or None
    _AUTH_EXEMPT_PATHS = {"/", "/health", "/docs", "/redoc", "/openapi.json"}
    _INTERNAL_BRIDGE_TOKEN_HEADER = "X-NOMAD-Internal-Token"
    _INTERNAL_BRIDGE_TOKEN = (
        os.environ.get("NOMAD_INTERNAL_TOKEN") or ""
    ).strip() or None
    _INTERNAL_BRIDGE_ALLOWED_ROUTES: set[tuple[str, str]] = {
        ("POST", "/api/vio/update"),
        ("POST", "/api/task/2/slam/mesh/update"),
        ("GET", "/api/task/2/slam/mesh/mode"),
        ("POST", "/api/detections/update"),
        ("POST", "/api/nav/velocity"),
        ("POST", "/api/servo/camera/tilt"),
        ("POST", "/api/task/1/target/detection_status/update"),
    }
    _INTERNAL_BRIDGE_MIN_TOKEN_LEN = 32

    if (
        _INTERNAL_BRIDGE_TOKEN is not None
        and len(_INTERNAL_BRIDGE_TOKEN) < _INTERNAL_BRIDGE_MIN_TOKEN_LEN
    ):
        logger.warning(
            "NOMAD_INTERNAL_TOKEN is shorter than %d chars; disabling internal bridge bypass",
            _INTERNAL_BRIDGE_MIN_TOKEN_LEN,
        )
        _INTERNAL_BRIDGE_TOKEN = None

    if _NOMAD_API_KEY is not None and _INTERNAL_BRIDGE_TOKEN is None:
        logger.warning(
            "NOMAD_INTERNAL_TOKEN is not configured; internal bridge bypass disabled"
        )

    def _is_loopback_client(request: Request) -> bool:
        client_host = ""
        if request.client is not None and request.client.host is not None:
            client_host = request.client.host.strip().lower()
        if client_host in {"127.0.0.1", "::1", "localhost"}:
            return True
        if client_host.startswith("::ffff:"):
            client_host = client_host.split("::ffff:", 1)[1]
        try:
            return ipaddress.ip_address(client_host).is_loopback
        except ValueError:
            return False

    def _is_internal_bridge_request(request: Request, request_path: str) -> bool:
        if _INTERNAL_BRIDGE_TOKEN is None:
            return False
        if (
            request.method.upper(),
            request_path,
        ) not in _INTERNAL_BRIDGE_ALLOWED_ROUTES:
            return False
        if not _is_loopback_client(request):
            return False
        provided_token = request.headers.get(_INTERNAL_BRIDGE_TOKEN_HEADER) or ""
        if not provided_token:
            return False
        return hmac.compare_digest(provided_token, _INTERNAL_BRIDGE_TOKEN)

    class APIKeyMiddleware(BaseHTTPMiddleware):
        async def dispatch(self, request: Request, call_next):
            if _NOMAD_API_KEY is None:
                # Development mode - no authentication
                return await call_next(request)
            request_path = request.url.path.rstrip("/") or "/"
            if request_path in _AUTH_EXEMPT_PATHS:
                return await call_next(request)
            if _is_internal_bridge_request(request, request_path):
                return await call_next(request)
            provided_key = request.headers.get("X-API-Key")
            if provided_key != _NOMAD_API_KEY:
                return JSONResponse(
                    status_code=401,
                    content={"detail": "Invalid or missing API key"},
                )
            return await call_next(request)

    app.add_middleware(APIKeyMiddleware)

    # Initialize app.state with all service references (dependency injection)
    app.state.state_manager = state_manager
    app.state.health_monitor = None
    app.state.isaac_bridge = None
    app.state.nav_controller = None
    app.state.tailscale_manager = None
    app.state.network_monitor = None
    app.state.camera_service = None
    app.state.mavlink_service = None
    app.state.mode_manager = None
    app.state.spray_controller = None
    app.state.excluded_sectors: set = (
        set()
    )  # SP-005: sectors excluded from obstacle avoidance

    # Latest OBSTACLE_DISTANCE snapshot cached for UI consumers (GET /api/obstacle_distance)
    app.state.obstacle_distance_last: Optional[dict] = None

    # VIO state from external sources (ROS bridge)
    app.state.external_vio_state: Optional[dict] = None
    app.state.slam_vio_ros_frame: Optional[dict] = None  # ROS-frame pose for SLAM 3D
    configured_mesh_mode = (
        (os.environ.get("NOMAD_SLAM_MESH_OUTPUT_MODE") or "voxel").strip().lower()
    )
    if configured_mesh_mode != "voxel":
        logger.warning(
            "Invalid NOMAD_SLAM_MESH_OUTPUT_MODE='%s'; defaulting to 'voxel'",
            configured_mesh_mode,
        )
        configured_mesh_mode = "voxel"
    app.state.slam_mesh_output_mode: str = configured_mesh_mode
    app.state.vio_trajectory: list[dict] = []  # List of {x, y, z, timestamp} points
    app.state.vio_trajectory_max_points: int = 1000  # Keep last N points
    app.state.vio_state_lock = threading.Lock()
    # Dedicated lock for SLAM mesh state (large 30MB+ updates) so writers don't
    # race on slam_mesh_version / slam_mesh_data when multiple POSTs land.
    app.state.slam_mesh_lock = threading.Lock()
    app.state.exclusion_map: list[dict] = []

    # Object detection state (HSV circle detection via ZED custom OD)
    app.state.detected_objects: list[dict] = []  # Current frame detections
    app.state.detection_history: list[
        dict
    ] = []  # Persistent detected targets with 3D positions
    app.state.detection_history_max: int = 200  # Max persistent detections to keep
    app.state.detection_state_lock = threading.Lock()
    app.state.detection_last_update: float = 0.0
    app.state.detection_last_source_timestamp = None
    app.state.detection_enabled: bool = True  # Desired ZED OD mode for circle detection

    # Cached Task 1 detection status (populated by background poller, read by API)
    app.state.task1_det_cache: dict = {"circle_count": 0, "success": True}
    app.state.task1_det_cache_ts: float = 0.0
    app.state.isaac_runtime_cache = {
        "timestamp": 0.0,
        "container_running": False,
        "nvblox_running": False,
        "bridge_running": False,
        "target_localizer_running": False,
    }
    app.state.isaac_startup_last_initiated = 0.0
    app.state.high_rate_zmq_enabled = os.environ.get(
        "NOMAD_HIGH_RATE_ZMQ_ENABLED", "1"
    ).strip().lower() not in ("0", "false", "no")
    app.state.high_rate_zmq_sub_mode = (
        os.environ.get(
            "NOMAD_HIGH_RATE_ZMQ_SUB_MODE",
            "bind",
        )
        .strip()
        .lower()
    )
    if app.state.high_rate_zmq_sub_mode not in ("bind", "connect"):
        logger.warning(
            "Invalid NOMAD_HIGH_RATE_ZMQ_SUB_MODE='%s'; falling back to 'bind'",
            app.state.high_rate_zmq_sub_mode,
        )
        app.state.high_rate_zmq_sub_mode = "bind"
    configured_high_rate_zmq_endpoint = os.environ.get(
        "NOMAD_HIGH_RATE_ZMQ_ENDPOINT",
        "",
    ).strip()
    if configured_high_rate_zmq_endpoint:
        app.state.high_rate_zmq_endpoint = configured_high_rate_zmq_endpoint
    elif app.state.high_rate_zmq_sub_mode == "bind":
        try:
            scheme, endpoint_rest = DEFAULT_ROS_HIGH_RATE_ENDPOINT.split("://", 1)
            _, default_port = endpoint_rest.rsplit(":", 1)
            app.state.high_rate_zmq_endpoint = f"{scheme}://0.0.0.0:{default_port}"
        except Exception:
            app.state.high_rate_zmq_endpoint = "tcp://0.0.0.0:5557"
    else:
        app.state.high_rate_zmq_endpoint = DEFAULT_ROS_HIGH_RATE_ENDPOINT
    app.state.high_rate_zmq_stop_event = threading.Event()
    app.state.high_rate_zmq_thread = None
    app.state.high_rate_zmq_warn_interval_s = 2.0
    app.state.high_rate_zmq_last_warn: dict[str, float] = {}
    cmd_vel_max_age_raw = os.environ.get("NOMAD_CMD_VEL_MAX_AGE_S", "0.5").strip()
    try:
        app.state.nav_cmd_vel_max_age_s = float(cmd_vel_max_age_raw)
        if app.state.nav_cmd_vel_max_age_s <= 0.0:
            raise ValueError("max age must be positive")
    except Exception:
        app.state.nav_cmd_vel_max_age_s = 0.5
        logger.warning(
            "Invalid NOMAD_CMD_VEL_MAX_AGE_S='%s'; falling back to %.2fs",
            cmd_vel_max_age_raw,
            app.state.nav_cmd_vel_max_age_s,
        )
    app.state.nav_cmd_vel_last_timestamp_by_source: dict[str, float] = {}
    app.state.nav_cmd_vel_order_lock = threading.Lock()
    app.state.vio_last_timestamp_by_source: dict[str, float] = {}

    def _high_rate_warn(key: str, message: str) -> None:
        """Throttle repeated high-rate ZMQ warning logs."""
        now = time.time()
        last = app.state.high_rate_zmq_last_warn.get(key, 0.0)
        if now - last >= app.state.high_rate_zmq_warn_interval_s:
            logger.warning(message)
            app.state.high_rate_zmq_last_warn[key] = now

    def _apply_vio_update_from_request(vio_request: VIOUpdateRequest) -> int:
        """Apply VIO update to shared app state and return trajectory length."""
        with app.state.vio_state_lock:
            source = (vio_request.source or "external").strip() or "external"
            last_timestamp = app.state.vio_last_timestamp_by_source.get(source)
            if last_timestamp is not None and vio_request.timestamp <= last_timestamp:
                return len(app.state.vio_trajectory)
            app.state.vio_last_timestamp_by_source[source] = vio_request.timestamp

            # Store latest state (NED frame for other consumers)
            app.state.external_vio_state = {
                "timestamp": vio_request.timestamp,
                "x": vio_request.x,
                "y": vio_request.y,
                "z": vio_request.z,
                "roll": vio_request.roll,
                "pitch": vio_request.pitch,
                "yaw": vio_request.yaw,
                "vx": vio_request.vx,
                "vy": vio_request.vy,
                "vz": vio_request.vz,
                "confidence": vio_request.confidence,
                "source": source,
            }

            state_manager.update_state(
                vio_x=vio_request.x,
                vio_y=vio_request.y,
                vio_z=vio_request.z,
                vio_yaw=vio_request.yaw,
                vio_confidence=vio_request.confidence,
            )

            # Store map-frame pose for SLAM 3D WebSocket (same frame as nvblox mesh vertices).
            # REP-103 map frame (X-forward, Y-left, Z-up); stable under ZED loop-closure corrections.
            app.state.slam_vio_ros_frame = {
                "x": vio_request.ros_x,
                "y": vio_request.ros_y,
                "z": vio_request.ros_z,
                "roll": vio_request.ros_roll,
                "pitch": vio_request.ros_pitch,
                "yaw": vio_request.ros_yaw,
                "body_roll": vio_request.body_roll,
                "body_pitch": vio_request.body_pitch,
                "body_yaw": vio_request.body_yaw,
                "timestamp": vio_request.timestamp,
                # Canonical SLAM frame identifier end-to-end: "map".
                # nvblox global_frame=map; bridge TF-looks up map->camera_link for pose.
                "frame_id": getattr(vio_request, "frame_id", "map"),
            }

            # Add to trajectory
            app.state.vio_trajectory.append(
                {
                    "x": vio_request.x,
                    "y": vio_request.y,
                    "z": vio_request.z,
                    "timestamp": vio_request.timestamp,
                }
            )

            # Trim trajectory if too long
            if len(app.state.vio_trajectory) > app.state.vio_trajectory_max_points:
                app.state.vio_trajectory = app.state.vio_trajectory[
                    -app.state.vio_trajectory_max_points :
                ]

            return len(app.state.vio_trajectory)

    def _get_vio_snapshot(include_trajectory: bool = False) -> dict[str, Any]:
        """Read VIO state under one lock to avoid mixed-frame snapshots."""
        with app.state.vio_state_lock:
            external_vio_state = (
                dict(app.state.external_vio_state)
                if app.state.external_vio_state
                else None
            )
            slam_vio_ros_frame = (
                dict(app.state.slam_vio_ros_frame)
                if app.state.slam_vio_ros_frame
                else None
            )
            vio_trajectory = (
                list(app.state.vio_trajectory) if include_trajectory else None
            )
        return {
            "external_vio_state": external_vio_state,
            "slam_vio_ros_frame": slam_vio_ros_frame,
            "vio_trajectory": vio_trajectory,
        }

    def _dispatch_nav_velocity(nav_request: NavVelocityRequest) -> bool:
        """Forward velocity command to NavController using existing API semantics."""
        nav_controller = app.state.nav_controller
        if not nav_controller:
            raise RuntimeError("Navigation controller not initialized")

        source = (nav_request.source or "nav2").strip() or "nav2"
        cmd_timestamp = float(nav_request.timestamp)
        if not math.isfinite(cmd_timestamp):
            raise ValueError("Rejected cmd_vel with non-finite timestamp")

        now = time.time()
        max_age_s = app.state.nav_cmd_vel_max_age_s
        age_s = now - cmd_timestamp
        if age_s > max_age_s:
            raise ValueError(
                f"Rejected stale cmd_vel from source '{source}': "
                f"age={age_s:.3f}s exceeds max_age={max_age_s:.3f}s"
            )
        if cmd_timestamp > now + max_age_s:
            raise ValueError(
                f"Rejected cmd_vel from source '{source}': "
                f"timestamp is too far in the future (max_skew={max_age_s:.3f}s)"
            )

        with app.state.nav_cmd_vel_order_lock:
            last_timestamp = app.state.nav_cmd_vel_last_timestamp_by_source.get(source)
            if last_timestamp is not None and cmd_timestamp <= last_timestamp:
                raise ValueError(
                    f"Rejected non-monotonic cmd_vel from source '{source}': "
                    f"timestamp={cmd_timestamp:.6f} <= last_timestamp={last_timestamp:.6f}"
                )

            accepted = nav_controller.send_velocity(
                vx=nav_request.vx,
                vy=nav_request.vy,
                vz=nav_request.vz,
                yaw_rate=nav_request.yaw_rate,
                source=source,
            )
            if accepted:
                app.state.nav_cmd_vel_last_timestamp_by_source[source] = cmd_timestamp

            return accepted

    def _apply_detections_update(
        detections: list, source_timestamp: Optional[Any] = None
    ) -> None:
        """Apply detection update to app state (shared by HTTP and ZMQ paths)."""
        import time as _time
        import math as _math

        with app.state.detection_state_lock:
            normalized_source_timestamp: Optional[float] = None
            if source_timestamp is not None:
                try:
                    normalized_source_timestamp = float(source_timestamp)
                    if not _math.isfinite(normalized_source_timestamp):
                        normalized_source_timestamp = None
                except (TypeError, ValueError):
                    normalized_source_timestamp = None

            if normalized_source_timestamp is not None:
                last_source_timestamp = app.state.detection_last_source_timestamp
                if last_source_timestamp is not None and _math.isclose(
                    normalized_source_timestamp,
                    last_source_timestamp,
                    rel_tol=0.0,
                    abs_tol=1e-6,
                ):
                    return
                app.state.detection_last_source_timestamp = normalized_source_timestamp

            if isinstance(detections, list):
                dict_detections = [det for det in detections if isinstance(det, dict)]
            else:
                dict_detections = []

            app.state.detected_objects = dict_detections
            app.state.detection_last_update = _time.time()

            history = app.state.detection_history
            for det in dict_detections:
                # Task 2 (shape/circle) targets are image_only: they ship with
                # x=y=z=0 and rely on the spray pipeline's image-space approach
                # instead of 3D dedup. Skip them here so they don't pollute the
                # 3D detection history with phantom origin-clustered entries.
                if det.get("image_only"):
                    continue
                x_val = det.get("x")
                y_val = det.get("y")
                z_val = det.get("z")
                if x_val is None or y_val is None or z_val is None:
                    continue
                try:
                    if not (
                        isinstance(x_val, (int, float))
                        and isinstance(y_val, (int, float))
                        and isinstance(z_val, (int, float))
                    ):
                        continue
                    if not (
                        _math.isfinite(x_val)
                        and _math.isfinite(y_val)
                        and _math.isfinite(z_val)
                    ):
                        continue
                except (TypeError, ValueError):
                    continue

                is_duplicate = False
                for existing in history:
                    dx = x_val - existing["x"]
                    dy = y_val - existing["y"]
                    dz = z_val - existing["z"]
                    dist = (dx * dx + dy * dy + dz * dz) ** 0.5
                    if dist < 0.5 and det.get("label") == existing.get("label"):
                        existing["seen_count"] = existing.get("seen_count", 1) + 1
                        if det.get("confidence", 0) > existing.get("confidence", 0):
                            existing.update(det)
                            existing["seen_count"] = existing.get("seen_count", 1)
                        is_duplicate = True
                        break

                if not is_duplicate:
                    det["seen_count"] = 1
                    det["first_seen"] = _time.time()
                    history.append(det)
                    if len(history) > app.state.detection_history_max:
                        history.pop(0)

    def _handle_high_rate_ipc_message(message: IPCMessage) -> None:
        """Handle a single high-rate IPC message from ros_http_bridge."""
        if message.msg_type == HIGH_RATE_MSG_TYPE_VIO:
            try:
                vio_request = VIOUpdateRequest(**message.data)
            except Exception as e:
                _high_rate_warn("vio-parse", f"Invalid high-rate VIO payload: {e}")
                return
            _apply_vio_update_from_request(vio_request)
            return

        if message.msg_type == HIGH_RATE_MSG_TYPE_CMD_VEL:
            try:
                nav_request = NavVelocityRequest(**message.data)
            except Exception as e:
                _high_rate_warn("cmd-parse", f"Invalid high-rate cmd_vel payload: {e}")
                return
            try:
                _dispatch_nav_velocity(nav_request)
            except ValueError as e:
                _high_rate_warn("cmd-gate", f"Dropping high-rate cmd_vel: {e}")
            except RuntimeError as e:
                _high_rate_warn("cmd-nav", f"Ignoring high-rate cmd_vel: {e}")
            except Exception as e:
                _high_rate_warn("cmd-send", f"High-rate cmd_vel dispatch failed: {e}")
            return

        if message.msg_type == HIGH_RATE_MSG_TYPE_DETECTIONS:
            try:
                detections = message.data.get("detections", [])
                source_timestamp = message.data.get("source_timestamp")
                _apply_detections_update(detections, source_timestamp=source_timestamp)
            except Exception as e:
                _high_rate_warn("det-zmq", f"High-rate detection update failed: {e}")

    def _high_rate_zmq_listener_loop(stop_event: threading.Event) -> None:
        """Background loop receiving high-rate telemetry from ros_http_bridge over ZMQ."""
        if not IPC_AVAILABLE:
            logger.warning(
                f"High-rate ZMQ listener disabled: IPC unavailable ({IPC_IMPORT_ERROR})"
            )
            return

        endpoint = app.state.high_rate_zmq_endpoint
        socket_mode = app.state.high_rate_zmq_sub_mode
        logger.info(f"High-rate ZMQ listener starting on {endpoint} ({socket_mode})")

        subscriber: Optional[ZMQSubscriber] = None
        try:
            while not stop_event.is_set():
                try:
                    if subscriber is None:
                        subscriber = ZMQSubscriber(
                            endpoint=endpoint,
                            timeout_ms=500,
                            socket_mode=socket_mode,
                            rcv_hwm=1,
                            conflate=True,
                            linger_ms=0,
                        )
                        subscriber.start()
                        logger.info(
                            f"High-rate ZMQ listener ready on {endpoint} ({socket_mode})"
                        )

                    message = subscriber.receive()
                    if message is None:
                        continue

                    _handle_high_rate_ipc_message(message)
                except Exception as e:
                    _high_rate_warn(
                        "listener-loop",
                        f"High-rate ZMQ listener error on {endpoint}: {e}",
                    )
                    if subscriber is not None:
                        try:
                            subscriber.stop()
                        except Exception:
                            pass
                        subscriber = None
                    if stop_event.wait(1.0):
                        break
        finally:
            if subscriber is not None:
                try:
                    subscriber.stop()
                except Exception:
                    pass
            logger.info("High-rate ZMQ listener stopped")

    def _start_high_rate_zmq_listener() -> None:
        """Start background high-rate ZMQ listener thread."""
        if not app.state.high_rate_zmq_enabled:
            logger.info(
                "High-rate ZMQ listener disabled by NOMAD_HIGH_RATE_ZMQ_ENABLED"
            )
            return

        thread = app.state.high_rate_zmq_thread
        if thread and thread.is_alive():
            return

        stop_event = app.state.high_rate_zmq_stop_event
        stop_event.clear()
        thread = threading.Thread(
            target=_high_rate_zmq_listener_loop,
            args=(stop_event,),
            name="high-rate-zmq-listener",
            daemon=True,
        )
        app.state.high_rate_zmq_thread = thread
        thread.start()

    def _stop_high_rate_zmq_listener() -> None:
        """Stop background high-rate ZMQ listener thread."""
        app.state.high_rate_zmq_stop_event.set()
        thread = app.state.high_rate_zmq_thread
        if thread and thread.is_alive():
            thread.join(timeout=2.0)
        app.state.high_rate_zmq_thread = None

    ctx = ApiRouteContext(
        logger=logger,
        apply_vio_update_from_request=_apply_vio_update_from_request,
        get_vio_snapshot=_get_vio_snapshot,
        dispatch_nav_velocity=_dispatch_nav_velocity,
        apply_detections_update=_apply_detections_update,
        nomad_api_key=_NOMAD_API_KEY,
    )

    @app.on_event("startup")
    async def _startup_high_rate_zmq_listener() -> None:
        """Start high-rate ZMQ listener on API startup."""
        _start_high_rate_zmq_listener()

    # NOTE: mavlink-routerd respawn is now owned by nomad-mavlink-router.service
    # (Restart=on-failure). The in-process watchdog that used to live here was
    # removed to eliminate the race where Edge Core and systemd would both try
    # to relaunch the router on the same crash. The watchdog *function* below
    # is retained for now in case it needs to be re-enabled for a non-systemd
    # deployment, but it is no longer scheduled at startup.

    @app.on_event("startup")
    async def _startup_detection_status_poller() -> None:
        """Launch background Task 1 detection status poller."""
        asyncio.create_task(_detection_status_poll_loop())

    @app.on_event("shutdown")
    async def _shutdown_high_rate_zmq_listener() -> None:
        """Stop high-rate ZMQ listener on API shutdown."""
        _stop_high_rate_zmq_listener()

    async def _mavlink_watchdog_loop() -> None:
        """Restart mavlink-routerd automatically if it exits unexpectedly.

        Polls every 15 s. If /dev/ttyACM0 is present but mavlink-routerd is not
        running, re-issues the same nohup command used by start_nomad_full.sh.
        Backs off 60 s after each restart attempt to avoid thrashing.
        """
        # Give the rest of startup a head start before the first check.
        await asyncio.sleep(30)
        loop = asyncio.get_running_loop()
        restart_cooldown_until: float = 0.0

        while True:
            try:
                await asyncio.sleep(15)

                if time.time() < restart_cooldown_until:
                    continue

                # Check whether the device is present and process is alive.
                def _check() -> tuple[bool, bool]:
                    has_device = os.path.exists("/dev/ttyACM0")
                    try:
                        r = subprocess.run(
                            ["pgrep", "-f", "mavlink-routerd"],
                            capture_output=True, text=True, timeout=3,
                        )
                        process_alive = r.returncode == 0
                    except Exception:
                        process_alive = True  # assume ok on probe failure
                    return has_device, process_alive

                has_device, process_alive = await loop.run_in_executor(None, _check)

                if not has_device or process_alive:
                    continue

                # mavlink-routerd is dead but the CubePilot is connected — restart.
                logger.warning("mavlink-routerd not running (CubePilot present) — restarting")

                def _restart() -> bool:
                    gcs_ip = os.environ.get("GCS_IP", "100.76.127.17")
                    gcs_port_lte = os.environ.get("GCS_PORT_LTE", "14560")
                    gcs_port_local = os.environ.get("GCS_PORT_LOCAL", "14550")
                    mavlink_uart_dev = os.environ.get("MAVLINK_UART_DEV", "/dev/ttyACM0")
                    log_path = os.path.expanduser("~/nomad_logs/mavlink.log")
                    os.makedirs(os.path.dirname(log_path), exist_ok=True)
                    try:
                        # Ensure any stale instance is gone first
                        subprocess.run(
                            ["pkill", "-9", "-f", "mavlink-routerd"],
                            capture_output=True, timeout=5,
                        )
                        time.sleep(1)
                    except Exception:
                        pass
                    try:
                        with open(log_path, "a") as lf:
                            subprocess.Popen(
                                [
                                    "mavlink-routerd",
                                    "-e", f"{gcs_ip}:{gcs_port_lte}",
                                    "-e", f"127.0.0.1:{gcs_port_local}",
                                    mavlink_uart_dev,
                                ],
                                stdout=lf,
                                stderr=lf,
                                start_new_session=True,
                            )
                        time.sleep(2)
                        r = subprocess.run(
                            ["pgrep", "-f", "mavlink-routerd"],
                            capture_output=True, text=True, timeout=3,
                        )
                        return r.returncode == 0
                    except Exception as e:
                        logger.error("mavlink-routerd restart failed: %s", e)
                        return False

                ok = await loop.run_in_executor(None, _restart)
                if ok:
                    logger.info("mavlink-routerd restarted successfully")
                else:
                    logger.error("mavlink-routerd restart failed — will retry in 60 s")

                restart_cooldown_until = time.time() + 60.0

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error("mavlink watchdog error: %s", e)

    async def _detection_status_poll_loop() -> None:
        """Watchdog: logs a warning if target_localizer stops pushing detection status.

        target_localizer_node.py pushes updates via POST to
        /api/task/1/target/detection_status/update every ~0.5s.
        This loop only checks that the cache is still fresh — no docker exec.
        """
        _STALE_WARN_S = 10.0  # warn if no push for this many seconds
        await asyncio.sleep(15)  # let the node start before checking
        while True:
            await asyncio.sleep(5.0)
            try:
                age = time.time() - getattr(app.state, "task1_det_cache_ts", 0.0)
                if age > _STALE_WARN_S:
                    logger.warning(
                        "Task 1 detection status cache is stale (%.0fs) — "
                        "target_localizer may not be running", age
                    )
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.debug("detection status watchdog error: %s", e)

    def _docker_exec_pgrep(
        container: str, pattern: str, timeout_s: int = 5
    ) -> Optional[bool]:
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

    def _docker_exec_bash_success(
        container: str, command: str, timeout_s: int = 5
    ) -> Optional[bool]:
        """Run a bash command in-container and return True/False, or None on probe failure."""
        try:
            result = subprocess.run(
                ["docker", "exec", container, "bash", "-lc", command],
                capture_output=True,
                text=True,
                timeout=timeout_s,
            )
            return result.returncode == 0
        except Exception:
            return None

    def _probe_isaac_runtime_state(force_refresh: bool = False) -> dict[str, bool]:
        """Probe Isaac ROS container/bridge process state with short cache grace."""
        now = time.time()
        cache = getattr(app.state, "isaac_runtime_cache", {}) or {}
        cache_age_s = now - float(cache.get("timestamp", 0.0))
        cache_max_stale_s = 20.0

        # Throttle probe frequency to avoid expensive docker exec churn.
        if not force_refresh and cache and cache_age_s < 8.0:
            return {
                "container_running": bool(cache.get("container_running", False)),
                "nvblox_running": bool(cache.get("nvblox_running", False)),
                "bridge_running": bool(cache.get("bridge_running", False)),
                "target_localizer_running": bool(cache.get("target_localizer_running", False)),
            }

        container_probe: Optional[bool] = None
        try:
            result = subprocess.run(
                [
                    "docker",
                    "ps",
                    "--filter",
                    "name=nomad_isaac_ros",
                    "--format",
                    "{{.Status}}",
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )
            container_probe = bool(result.stdout.strip())
        except Exception:
            container_probe = None

        if container_probe is None and cache_age_s < cache_max_stale_s:
            container_running = bool(cache.get("container_running", False))
        else:
            container_running = bool(container_probe)

        nvblox_running = False
        bridge_running = False
        target_localizer_running = False
        if container_running:
            nvblox_probe = _docker_exec_pgrep(
                "nomad_isaac_ros",
                "nvblox_node|nvblox_container|nvblox_examples_bringup|nomad_zed_nvblox\\.launch\\.py",
                timeout_s=5,
            )
            bridge_probe = _docker_exec_pgrep(
                "nomad_isaac_ros",
                "ros_http_bridge.py|ros_http_bridge",
                timeout_s=5,
            )
            target_localizer_probe = _docker_exec_pgrep(
                "nomad_isaac_ros",
                "target_localizer_node|target_localizer\\.launch\\.py",
                timeout_s=5,
            )

            if nvblox_probe is None and cache_age_s < cache_max_stale_s:
                nvblox_running = bool(cache.get("nvblox_running", False))
            else:
                nvblox_running = bool(nvblox_probe)

            if bridge_probe is None and cache_age_s < cache_max_stale_s:
                bridge_running = bool(cache.get("bridge_running", False))
            else:
                bridge_running = bool(bridge_probe)

            if target_localizer_probe is None and cache_age_s < cache_max_stale_s:
                target_localizer_running = bool(cache.get("target_localizer_running", False))
            else:
                target_localizer_running = bool(target_localizer_probe)

        app.state.isaac_runtime_cache = {
            "timestamp": now,
            "container_running": container_running,
            "nvblox_running": nvblox_running,
            "bridge_running": bridge_running,
            "target_localizer_running": target_localizer_running,
        }
        if container_running and nvblox_running and bridge_running:
            app.state.isaac_startup_last_initiated = 0.0

        return {
            "container_running": container_running,
            "nvblox_running": nvblox_running,
            "bridge_running": bridge_running,
            "target_localizer_running": target_localizer_running,
        }

    ctx.docker_exec_bash_success = _docker_exec_bash_success
    ctx.probe_isaac_runtime_state = _probe_isaac_runtime_state

    def _launch_nvblox_bridge_with_od(
        enable_od: bool,
        camera_retry_remaining: int = 2,
    ) -> dict:
        """
        Launch nvblox + ROS-HTTP bridge with explicit object detection mode.

        This always uses NOMAD's custom launch file so behavior stays consistent
        with the startup script used on Jetson.
        """
        container = "nomad_isaac_ros"

        if not hasattr(app.state, "slam_mesh_lock"):
            app.state.slam_mesh_lock = threading.Lock()
        if not hasattr(app.state, "slam_mesh_data"):
            app.state.slam_mesh_data = {}

        with app.state.slam_mesh_lock:
            previous_mesh_ts = (
                app.state.slam_mesh_data.get("received_at")
                if app.state.slam_mesh_data
                else None
            )

        # Verify container is running
        try:
            result = subprocess.run(
                [
                    "docker",
                    "ps",
                    "--filter",
                    f"name={container}",
                    "--format",
                    "{{.Names}}",
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if container not in result.stdout:
                return {"success": False, "error": "Container not running"}
        except Exception as e:
            return {"success": False, "error": str(e)}

        # Re-bind ZED camera to uvcvideo driver (may be needed after previous kill)
        # This ensures /dev/video* devices are available in container
        try:
            subprocess.run(
                [
                    "docker",
                    "exec",
                    container,
                    "bash",
                    "-c",
                    'for dev in /sys/bus/usb/devices/*/idVendor; do dir=$(dirname $dev); vid=$(cat $dev 2>/dev/null); if [ "$vid" = "2b03" ]; then for iface in $dir/*:*/bInterfaceClass; do idir=$(dirname $iface); cls=$(cat $iface 2>/dev/null); iname=$(basename $idir); if [ "$cls" = "0e" ] && [ ! -e $idir/driver ]; then echo $iname > /sys/bus/usb/drivers/uvcvideo/bind 2>/dev/null || true; fi; done; fi; done; sleep 1',
                ],
                capture_output=True,
                timeout=10,
            )
        except Exception:
            pass  # Non-fatal if rebinding fails

        od_value = "true" if enable_od else "false"
        mode_text = "enabled" if enable_od else "disabled"
        bridge_api_key = (os.environ.get("NOMAD_API_KEY") or "").strip()
        bridge_internal_token = (os.environ.get("NOMAD_INTERNAL_TOKEN") or "").strip()
        bridge_env_prefix = ""
        if bridge_api_key:
            bridge_env_prefix += f"NOMAD_API_KEY='{bridge_api_key}' "
        if bridge_internal_token:
            bridge_env_prefix += f"NOMAD_INTERNAL_TOKEN='{bridge_internal_token}' "
        # export-style for use before setsid (avoids single-quote nesting issues)
        bridge_env_exports = ""
        if bridge_api_key:
            bridge_env_exports += f"export NOMAD_API_KEY='{bridge_api_key}'\n"
        if bridge_internal_token:
            bridge_env_exports += f"export NOMAD_INTERNAL_TOKEN='{bridge_internal_token}'\n"

        # Build launch script with OD config merge
        # od_value is interpolated into the heredoc so the script knows whether to enable OD
        launch_script = f"""#!/bin/bash
set -e
source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
# ZED SDK 5.x ROS2 wrapper links against GXF Isaac libraries that live outside
# the default ROS library path.  Add all GXF lib dirs and the arch-specific
# ROS lib dir so the component_container can dlopen the camera plugin.
GXF_LIB_DIRS=$(find /opt/ros/humble/share -path '*/gxf/lib' -type d 2>/dev/null | tr '\\n' ':')
export LD_LIBRARY_PATH=/usr/local/zed/lib:/opt/ros/humble/lib/aarch64-linux-gnu:${{GXF_LIB_DIRS}}$LD_LIBRARY_PATH
# Reduce FastDDS SHM lock contention that can wedge ROS graph/topic flow
# after repeated relaunches in containerized runtime.
export RMW_FASTRTPS_TRANSPORT_SHARED_MEMORY_DISABLED=1

# Kill ALL previous nvblox/ZED/bridge processes from ANY launch path.
# Do NOT kill launcher scripts by name — this script IS launch_nvblox_bridge.sh
# and pkill would match our own process. Kill the actual ROS2 processes instead;
# wrapper scripts will exit when their children die.
pkill -f 'nomad_zed_nvblox\\.launch\\.py|zed_example\\.launch\\.py' 2>/dev/null || true
pkill -f 'component_container' 2>/dev/null || true
pkill -f 'controller_server|planner_server|smoother_server|behavior_server|bt_navigator|lifecycle_manager_navigation|waypoint_follower|velocity_smoother' 2>/dev/null || true
pkill -f 'target_localizer_node|target_localizer\\.launch\\.py' 2>/dev/null || true
pkill -f 'servo_tf_publisher\\.py|obstacle_distance_bridge\\.py|nav2_goal_bridge\\.py|robot_state_publisher|static_transform_publisher' 2>/dev/null || true
sleep 2
# Clean up stale FastRTPS/DDS shared memory locks left by killed processes.
# Without this, new ROS2 nodes fail with RTPS_TRANSPORT_SHM port lock errors.
rm -f /dev/shm/fastrtps_* 2>/dev/null || true

# Camera preflight with retry/rebind attempts.
cam_ready=false
for attempt in 1 2 3; do
    if grep -q '^2b03$' /sys/bus/usb/devices/*/idVendor 2>/dev/null && ls /dev/video* >/dev/null 2>&1; then
        cam_ready=true
        echo "ZED camera detected (attempt $attempt)"
        break
    fi

    echo "ZED camera not detected (attempt $attempt/3), rebinding USB video interfaces"
    for dev in /sys/bus/usb/devices/*/idVendor; do
        dir=$(dirname $dev)
        vid=$(cat $dev 2>/dev/null)
        if [ "$vid" = "2b03" ]; then
            for iface in $dir/*:*/bInterfaceClass; do
                idir=$(dirname $iface)
                cls=$(cat $iface 2>/dev/null)
                iname=$(basename $idir)
                if [ "$cls" = "0e" ]; then
                    echo $iname > /sys/bus/usb/drivers/uvcvideo/unbind 2>/dev/null || true
                    sleep 0.1
                    echo $iname > /sys/bus/usb/drivers/uvcvideo/bind 2>/dev/null || true
                fi
            done
        fi
    done
    sleep 2
done

if [ "$cam_ready" != "true" ]; then
    echo "ERROR: ZED camera not detected after retries"
    exit 3
fi

ENABLE_OD={od_value}
CUSTOM_OD=/workspaces/isaac_ros-dev/config/custom_circle_detection.yaml
ZED_COMMON=/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/sensors/zed_common.yaml

# Only merge custom OD config when object detection is enabled
if [ "$ENABLE_OD" = "true" ] && [ -f "$CUSTOM_OD" ] && [ -f "$ZED_COMMON" ]; then
    python3 << 'PYEOF2'
import yaml
custom_path = "/workspaces/isaac_ros-dev/config/custom_circle_detection.yaml"
common_path = "/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/sensors/zed_common.yaml"
try:
    with open(common_path, 'r') as f:
        common = yaml.safe_load(f) or {{}}
    with open(custom_path, 'r') as f:
        custom = yaml.safe_load(f) or {{}}

    # Extract the object_detection block from custom config
    od_params = None
    if '/**' in custom and 'ros__parameters' in custom.get('/**', {{}}):
        od_params = custom['/**']['ros__parameters'].get('object_detection')
    elif 'ros__parameters' in custom:
        od_params = custom['ros__parameters'].get('object_detection')

    if od_params is None:
        print("WARNING: No object_detection section found in custom config")
    else:
        # Find the existing namespace key in zed_common.yaml and inject there
        # ZED common.yaml may use '/**:', '/zed/zed_node:', or bare 'ros__parameters:'
        injected = False
        for key in common:
            if isinstance(common[key], dict) and 'ros__parameters' in common[key]:
                common[key]['ros__parameters']['object_detection'] = od_params
                print(f"Injected object_detection into '{{key}}' namespace")
                injected = True
                break

        # Fallback: if no namespaced ros__parameters found, create under /**
        if not injected:
            if '/**' not in common:
                common['/**'] = {{}}
            if 'ros__parameters' not in common['/**']:
                common['/**']['ros__parameters'] = {{}}
            common['/**']['ros__parameters']['object_detection'] = od_params
            print("Injected object_detection into new '/**' namespace")

    with open(common_path, 'w') as f:
        yaml.safe_dump(common, f, default_flow_style=False, sort_keys=False)
    print("Applied custom OD config with custom_onnx_file")
except Exception as e:
    print("Custom OD merge ERROR: " + str(e))
PYEOF2
elif [ "$ENABLE_OD" = "false" ] && [ -f "$ZED_COMMON" ]; then
    # Remove any previous object_detection config so ZED starts without OD
    python3 << 'PYEOF3'
import yaml
common_path = "/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/sensors/zed_common.yaml"
try:
    with open(common_path, 'r') as f:
        common = yaml.safe_load(f) or {{}}
    removed = False
    for key in common:
        if isinstance(common[key], dict) and 'ros__parameters' in common[key]:
            if 'object_detection' in common[key]['ros__parameters']:
                del common[key]['ros__parameters']['object_detection']
                removed = True
    if removed:
        with open(common_path, 'w') as f:
            yaml.safe_dump(common, f, default_flow_style=False, sort_keys=False)
        print("Removed object_detection config (OD disabled)")
    else:
        print("No object_detection config to remove")
except Exception as e:
    print("OD removal warning: " + str(e))
PYEOF3
fi

# Overlay NOMAD nvblox config
NOMAD_CFG=/workspaces/isaac_ros-dev/config/nvblox_performance.yaml
NVBLOX_BASE_A=$(python3 -c "from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory(\"nvblox_examples_bringup\"))" 2>/dev/null || true)/config/nvblox/nvblox_base.yaml
NVBLOX_BASE_B=/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml
NVBLOX_BASE=""
for cand in "$NVBLOX_BASE_A" "$NVBLOX_BASE_B"; do
    if [ -f "$cand" ]; then
        NVBLOX_BASE="$cand"
        break
    fi
done
if [ -f "$NOMAD_CFG" ] && [ -n "$NVBLOX_BASE" ]; then
    cp "$NOMAD_CFG" "$NVBLOX_BASE"
        echo "Applied NOMAD nvblox config to $NVBLOX_BASE"
        sha256sum "$NOMAD_CFG" "$NVBLOX_BASE" 2>/dev/null || true
else
        echo "WARNING: Could not resolve nvblox base config path"
        echo "  NOMAD_CFG=$NOMAD_CFG"
        echo "  NVBLOX_BASE_A=$NVBLOX_BASE_A"
        echo "  NVBLOX_BASE_B=$NVBLOX_BASE_B"
fi

# Configure NITROS mode for ZED SDK 5.x.
# In OD mode, target_localizer is a Python node that consumes sensor_msgs/Image;
# forcing disable_nitros=true improves compatibility/reliability of RGB/depth
# delivery to Python subscribers.
python3 << 'PYEOF_NITROS'
import yaml
import os
common_path = "/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/sensors/zed_common.yaml"
try:
    with open(common_path, 'r') as f:
        common = yaml.safe_load(f) or {{}}

    enable_od = os.environ.get("ENABLE_OD", "false").strip().lower() == "true"
    disable_nitros = True if enable_od else False

    for key in common:
        if isinstance(common[key], dict) and 'ros__parameters' in common[key]:
            params = common[key]['ros__parameters']
            if 'debug' not in params:
                params['debug'] = {{}}
            params['debug']['disable_nitros'] = disable_nitros
            params['debug']['debug_nitros'] = False
            break
    with open(common_path, 'w') as f:
        yaml.safe_dump(common, f, default_flow_style=False, sort_keys=False)
    print("Configured NITROS (ENABLE_OD=%s, disable_nitros=%s)" % (enable_od, disable_nitros))
except Exception as e:
    print("NITROS config warning: " + str(e))
PYEOF_NITROS

# Ensure ZED loop-closure memory is enabled for drift correction.
python3 << 'PYEOF_AREA_MEMORY'
import yaml

common_path = "/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/sensors/zed_common.yaml"
try:
    with open(common_path, 'r') as f:
        common = yaml.safe_load(f) or {{}}

    patched = False
    for key in common:
        if isinstance(common[key], dict) and 'ros__parameters' in common[key]:
            params = common[key]['ros__parameters']

            if isinstance(params.get('positional_tracking'), dict):
                params['positional_tracking']['area_memory'] = True
            elif isinstance(params.get('pos_tracking'), dict):
                params['pos_tracking']['area_memory'] = True
            else:
                params['positional_tracking'] = {{'area_memory': True}}

            patched = True
            break

    if patched:
        with open(common_path, 'w') as f:
            yaml.safe_dump(common, f, default_flow_style=False, sort_keys=False)
        print("Ensured ZED area_memory enabled")
    else:
        print("Area memory warning: no ros__parameters namespace found in zed_common.yaml")
except Exception as e:
    print("Area memory config warning: " + str(e))
PYEOF_AREA_MEMORY

# ZED SDK 5.2 publishes RGB on /zed/zed_node/rgb/color/rect/*, while
# older nvblox launch files still remap to legacy RGB topics. Patch remaps
# in-place so nvblox receives color frames.
python3 << 'PYEOF_RGB_REMAP'
from pathlib import Path

launch_path = Path(
    "/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/"
    "nvblox_examples_bringup/launch/zed_nvblox_split.launch.py"
)

if not launch_path.exists():
    print("WARNING: nvblox split launch not found: " + str(launch_path))
else:
    text = launch_path.read_text()
    text_new = text.replace(
        "'/zed/zed_node/rgb/image_rect_color'",
        "'/zed/zed_node/rgb/color/rect/image'",
    )
    text_new = text_new.replace(
        "'/zed/zed_node/rgb/camera_info'",
        "'/zed/zed_node/rgb/color/rect/camera_info'",
    )
    if text_new != text:
        launch_path.write_text(text_new)
        print("Patched nvblox RGB remaps to ZED SDK 5.2 topics")
    else:
        print("nvblox RGB remaps already set")
PYEOF_RGB_REMAP

# NOTE: Do NOT patch pub_downscale_factor to 1.0 (720p).
# ZED 360p (default downscale 2.0) uses ~75% less GPU memory than 720p.
# On 8GB Jetson Orin Nano, 720p depth causes cudaErrorIllegalAddress
# when nvblox allocates GPU memory for depth integration.

# Launch nvblox with NOMAD custom launch
NOMAD_LAUNCH=/workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py
if [ ! -f "$NOMAD_LAUNCH" ]; then
    echo "ERROR: NOMAD launch file not found at $NOMAD_LAUNCH"
    exit 2
fi

ros2 launch "$NOMAD_LAUNCH" enable_od:=$ENABLE_OD &
echo $! > /tmp/zed_nvblox.pid

# Wait for launch process to stabilize before starting bridge.
sleep 10
if ! kill -0 "$(cat /tmp/zed_nvblox.pid 2>/dev/null)" 2>/dev/null; then
    echo "ERROR: ZED/nvblox launch exited early"
    exit 4
fi

# Ensure nvblox-expected frame alias exists even when camera URDF chain is delayed.
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id camera_link --child-frame-id zed_camera_link \
    >/tmp/zed_camera_link_alias.log 2>&1 &

# NOTE: ros_http_bridge is managed independently via /api/isaac/bridge/start
# and is NOT restarted here so it survives nvblox relaunches.

# target_localizer is launched by nomad_zed_nvblox.launch.py above.
# Just wait here until its services are discoverable before returning.
TL_READY=0
for i in $(seq 1 45); do
    if ROS2CLI_DISABLE_DAEMON=1 ros2 service list 2>/dev/null | grep -q '/target_localizer/capture_target' \
       && ROS2CLI_DISABLE_DAEMON=1 ros2 service list 2>/dev/null | grep -q '/target_localizer/save_targets' \
       && ROS2CLI_DISABLE_DAEMON=1 ros2 service list 2>/dev/null | grep -q '/target_localizer/print_model'; then
        TL_READY=1
        break
    fi
    sleep 1
done

if [ "$TL_READY" != "1" ]; then
    echo "WARNING: target_localizer services not discoverable yet; continuing startup"
fi

    # Wait for ZED/nvblox launch to exit. The ros_http_bridge runs in its own
    # session (setsid above) and continues independently — do NOT kill it here.
    # Killing the bridge on nvblox crash would cut off VIO/telemetry from GCS.
    if [ -f /tmp/zed_nvblox.pid ]; then
        ZED_PID=$(cat /tmp/zed_nvblox.pid 2>/dev/null || true)
        if [ -n "$ZED_PID" ]; then
            wait "$ZED_PID"
            ZED_RC=$?
            echo "ZED/nvblox launch exited (rc=$ZED_RC); bridge continues running independently"
            exit "$ZED_RC"
        fi
    fi

    wait
"""

        try:
            # Write launch script into container
            subprocess.run(
                [
                    "docker",
                    "exec",
                    container,
                    "bash",
                    "-c",
                    f"cat > /tmp/launch_nvblox_bridge.sh << 'EOFSCRIPT'\n{launch_script}\nEOFSCRIPT\nchmod +x /tmp/launch_nvblox_bridge.sh",
                ],
                capture_output=True,
                text=True,
                timeout=10,
                check=True,
            )

            # Run in background
            result = subprocess.run(
                [
                    "docker",
                    "exec",
                    "-d",
                    container,
                    "bash",
                    "-c",
                    "bash /tmp/launch_nvblox_bridge.sh > /tmp/zed_nvblox.log 2>&1",
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )
            if result.returncode != 0:
                return {
                    "success": False,
                    "error": f"Launch failed: {result.stderr.strip()}",
                }

            # Validate launch stayed alive long enough to be meaningful.
            launch_ok = False
            for _ in range(12):
                time.sleep(1)
                probe = subprocess.run(
                    [
                        "docker",
                        "exec",
                        container,
                        "bash",
                        "-c",
                        "test -f /tmp/zed_nvblox.pid && kill -0 $(cat /tmp/zed_nvblox.pid) 2>/dev/null",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                if probe.returncode == 0:
                    launch_ok = True
                    break

            if not launch_ok:
                log_tail = subprocess.run(
                    ["docker", "exec", container, "tail", "-80", "/tmp/zed_nvblox.log"],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                snippet = (log_tail.stdout or log_tail.stderr or "")[-600:]
                return {
                    "success": False,
                    "error": "nvblox launch did not stay alive (likely camera or launch config issue)",
                    "logs": snippet,
                }

            cam_err = subprocess.run(
                [
                    "docker",
                    "exec",
                    container,
                    "bash",
                    "-c",
                    "grep -q 'CAMERA NOT DETECTED' /tmp/zed_nvblox.log",
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if cam_err.returncode == 0:
                return {
                    "success": False,
                    "error": "ZED camera not detected inside container",
                }

            cam_stream_err = subprocess.run(
                [
                    "docker",
                    "exec",
                    container,
                    "bash",
                    "-c",
                    "grep -Eq 'CAMERA STREAM FAILED TO START|CAMERA FAILED TO SETUP|Camera detection timeout|Error opening camera' /tmp/zed_nvblox.log",
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if cam_stream_err.returncode == 0:
                if camera_retry_remaining > 0:
                    logger.warning(
                        "ZED camera stream failed; retrying launch with extra cleanup (remaining=%s)",
                        camera_retry_remaining,
                    )
                    try:
                        subprocess.run(
                            [
                                "docker",
                                "exec",
                                container,
                                "bash",
                                "-lc",
                                'pkill -f \'component_container|zed_example\\.launch\\.py|nomad_zed_nvblox\\.launch\\.py|ros_http_bridge|target_localizer_node|servo_tf_publisher\\.py|obstacle_distance_bridge\\.py|static_transform_publisher\' 2>/dev/null || true; rm -f /dev/shm/fastrtps_* /tmp/zed_nvblox.pid /tmp/ros_bridge.pid /tmp/target_localizer.pid 2>/dev/null || true; for dev in /sys/bus/usb/devices/*/idVendor; do dir=$(dirname "$dev"); vid=$(cat "$dev" 2>/dev/null); if [ "$vid" = "2b03" ]; then for iface in "$dir"/*:*/bInterfaceClass; do idir=$(dirname "$iface"); cls=$(cat "$iface" 2>/dev/null); iname=$(basename "$idir"); if [ "$cls" = "0e" ]; then echo "$iname" > /sys/bus/usb/drivers/uvcvideo/unbind 2>/dev/null || true; sleep 0.2; echo "$iname" > /sys/bus/usb/drivers/uvcvideo/bind 2>/dev/null || true; fi; done; fi; done',
                            ],
                            capture_output=True,
                            text=True,
                            timeout=8,
                        )
                    except Exception:
                        pass
                    time.sleep(3)
                    return _launch_nvblox_bridge_with_od(
                        enable_od=enable_od,
                        camera_retry_remaining=camera_retry_remaining - 1,
                    )

                log_tail = subprocess.run(
                    [
                        "docker",
                        "exec",
                        container,
                        "tail",
                        "-120",
                        "/tmp/zed_nvblox.log",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                return {
                    "success": False,
                    "error": "ZED camera stream failed to start inside container",
                    "logs": (log_tail.stdout or log_tail.stderr or "")[-800:],
                }

            # Require at least one fresh mesh update before reporting success.
            mesh_ready = False
            for _ in range(50):
                time.sleep(0.5)
                with app.state.slam_mesh_lock:
                    current_mesh_ts = (
                        app.state.slam_mesh_data.get("received_at")
                        if app.state.slam_mesh_data
                        else None
                    )
                if current_mesh_ts and current_mesh_ts != previous_mesh_ts:
                    mesh_ready = True
                    break

            if not mesh_ready:
                if enable_od:
                    # OD startup should be accepted only when target_localizer is
                    # discoverable AND RGB/depth streams are producing messages.
                    detector_ready = False
                    service_ready = False
                    for _ in range(30):
                        time.sleep(0.5)
                        service_probe = subprocess.run(
                            [
                                "docker",
                                "exec",
                                container,
                                "bash",
                                "-lc",
                                "source /opt/ros/humble/setup.bash >/dev/null 2>&1; ROS2CLI_DISABLE_DAEMON=1 ros2 service list 2>/dev/null | grep -q '/target_localizer/capture_target'",
                            ],
                            capture_output=True,
                            text=True,
                            timeout=5,
                        )
                        topic_probe = subprocess.run(
                            [
                                "docker",
                                "exec",
                                container,
                                "bash",
                                "-lc",
                                "source /opt/ros/humble/setup.bash >/dev/null 2>&1; ROS2CLI_DISABLE_DAEMON=1 ros2 topic list 2>/dev/null | grep -q '/zed/zed_node/rgb/color/rect/image'",
                            ],
                            capture_output=True,
                            text=True,
                            timeout=5,
                        )
                        if (
                            service_probe.returncode == 0
                            and topic_probe.returncode == 0
                        ):
                            service_ready = True
                            break

                    rgb_stream_probe = subprocess.run(
                        [
                            "docker",
                            "exec",
                            container,
                            "bash",
                            "-lc",
                            "source /opt/ros/humble/setup.bash >/dev/null 2>&1; export ROS2CLI_DISABLE_DAEMON=1; export ROS2CLI_NO_DAEMON=1; timeout 8s ros2 topic echo --once /zed/zed_node/rgb/color/rect/image >/dev/null 2>&1",
                        ],
                        capture_output=True,
                        text=True,
                        timeout=12,
                    )
                    depth_stream_probe = subprocess.run(
                        [
                            "docker",
                            "exec",
                            container,
                            "bash",
                            "-lc",
                            "source /opt/ros/humble/setup.bash >/dev/null 2>&1; export ROS2CLI_DISABLE_DAEMON=1; export ROS2CLI_NO_DAEMON=1; timeout 8s ros2 topic echo --once /zed/zed_node/depth/depth_registered >/dev/null 2>&1",
                        ],
                        capture_output=True,
                        text=True,
                        timeout=12,
                    )
                    rgb_stream_ready = rgb_stream_probe.returncode == 0
                    depth_stream_ready = depth_stream_probe.returncode == 0

                    rgb_pub_probe = _docker_exec_bash_success(
                        container,
                        "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
                        "source /workspaces/isaac_ros-dev/install/setup.bash >/dev/null 2>&1; "
                        "ROS2CLI_DISABLE_DAEMON=1 ros2 topic info /zed/zed_node/rgb/color/rect/image 2>/dev/null | "
                        "grep -Eq 'Publisher count: [1-9]'",
                        timeout_s=6,
                    )
                    depth_pub_probe = _docker_exec_bash_success(
                        container,
                        "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
                        "source /workspaces/isaac_ros-dev/install/setup.bash >/dev/null 2>&1; "
                        "ROS2CLI_DISABLE_DAEMON=1 ros2 topic info /zed/zed_node/depth/depth_registered 2>/dev/null | "
                        "grep -Eq 'Publisher count: [1-9]'",
                        timeout_s=6,
                    )

                    rgb_publisher_ready = bool(rgb_pub_probe)
                    depth_publisher_ready = bool(depth_pub_probe)
                    rgb_ready = rgb_stream_ready or rgb_publisher_ready
                    depth_ready = depth_stream_ready or depth_publisher_ready
                    detector_ready = service_ready and rgb_ready and depth_ready

                    if detector_ready:
                        return {
                            "success": True,
                            "message": "Detections launch succeeded (target_localizer ready; mesh stream still warming up).",
                            "detection_enabled": enable_od,
                            "mesh_ready": False,
                        }

                    if camera_retry_remaining > 0 and (
                        not rgb_ready or not depth_ready
                    ):
                        logger.warning(
                            "OD launch missing streams; retrying with extra cleanup "
                            "(service_ready=%s, rgb=%s, depth=%s, rgb_pub=%s, depth_pub=%s, remaining=%s)",
                            service_ready,
                            rgb_stream_ready,
                            depth_stream_ready,
                            rgb_publisher_ready,
                            depth_publisher_ready,
                            camera_retry_remaining,
                        )
                        try:
                            subprocess.run(
                                [
                                    "docker",
                                    "exec",
                                    container,
                                    "bash",
                                    "-lc",
                                    'pkill -f \'component_container|zed_example\\.launch\\.py|nomad_zed_nvblox\\.launch\\.py|ros_http_bridge|target_localizer_node|servo_tf_publisher\\.py|obstacle_distance_bridge\\.py|static_transform_publisher\' 2>/dev/null || true; rm -f /dev/shm/fastrtps_* /tmp/zed_nvblox.pid /tmp/ros_bridge.pid /tmp/target_localizer.pid 2>/dev/null || true; for dev in /sys/bus/usb/devices/*/idVendor; do dir=$(dirname "$dev"); vid=$(cat "$dev" 2>/dev/null); if [ "$vid" = "2b03" ]; then for iface in "$dir"/*:*/bInterfaceClass; do idir=$(dirname "$iface"); cls=$(cat "$iface" 2>/dev/null); iname=$(basename "$idir"); if [ "$cls" = "0e" ]; then echo "$iname" > /sys/bus/usb/drivers/uvcvideo/unbind 2>/dev/null || true; sleep 0.2; echo "$iname" > /sys/bus/usb/drivers/uvcvideo/bind 2>/dev/null || true; fi; done; fi; done',
                                ],
                                capture_output=True,
                                text=True,
                                timeout=8,
                            )
                        except Exception:
                            pass
                        time.sleep(3)
                        return _launch_nvblox_bridge_with_od(
                            enable_od=enable_od,
                            camera_retry_remaining=camera_retry_remaining - 1,
                        )

                    log_tail = subprocess.run(
                        [
                            "docker",
                            "exec",
                            container,
                            "tail",
                            "-120",
                            "/tmp/zed_nvblox.log",
                        ],
                        capture_output=True,
                        text=True,
                        timeout=5,
                    )
                    return {
                        "success": False,
                        "error": (
                            "Detections launch incomplete: target_localizer or camera streams not ready "
                            f"(service_ready={service_ready}, rgb_stream_ready={rgb_stream_ready}, "
                            f"depth_stream_ready={depth_stream_ready}, "
                            f"rgb_publisher_ready={rgb_publisher_ready}, "
                            f"depth_publisher_ready={depth_publisher_ready})"
                        ),
                        "logs": (log_tail.stdout or log_tail.stderr or "")[-800:],
                    }

                return {
                    "success": False,
                    "error": "Launch completed but mesh stream did not become active.",
                }

            return {
                "success": True,
                "message": f"nvblox + ROS-HTTP bridge launching with circle detection {mode_text}. ZED init takes ~15s.",
                "detection_enabled": enable_od,
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    ctx.launch_nvblox_bridge_with_od = _launch_nvblox_bridge_with_od
    from .api_routes.system import register_system_routes
    register_system_routes(app, ctx)

    from .api_routes.task1 import register_task1_routes
    register_task1_routes(app, ctx)

    from .api_routes.task2_ops import register_task2_routes
    register_task2_routes(app, ctx)

    from .api_routes.terminal import register_terminal_routes
    register_terminal_routes(app, ctx)

    def _require_terminal_api_key() -> None:
        """
        Guard for terminal/admin routes.

        When NOMAD_API_KEY is configured, APIKeyMiddleware enforces X-API-Key.
        When NOMAD_API_KEY is not configured, this remains development mode.
        """
        return

    from .api_routes.services import register_services_routes
    register_services_routes(app, ctx)

    from .api_routes.streaming import register_streaming_routes
    register_streaming_routes(app, ctx)

    from .api_routes.isaac import register_isaac_routes
    register_isaac_routes(app, ctx)

    from .api_routes.detections import register_detection_routes
    register_detection_routes(app, ctx)

    from .api_routes.video_slam import register_video_slam_routes
    register_video_slam_routes(app, ctx)

    ctx.require_terminal_api_key = _require_terminal_api_key
    from .api_routes.calibration_admin_servo import register_calibration_admin_servo_routes
    register_calibration_admin_servo_routes(app, ctx)

    return app
