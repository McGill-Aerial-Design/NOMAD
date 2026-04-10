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
from pydantic import BaseModel
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

from .state import StateManager

# msgpack for efficient mesh deserialization (optional)
try:
    import msgpack

    MSGPACK_AVAILABLE = True
except ImportError:
    MSGPACK_AVAILABLE = False

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


# ==================== Request/Response Models ====================


class Task2HitRequest(BaseModel):
    """Request model for Task 2 target hit."""

    x: float
    y: float
    z: float


class Task1CapturesList(BaseModel):
    """Response model for list of Task 1 captures."""

    captures: list[str]
    count: int


# Whitelist of allowed terminal commands for safety
# NOTE: mediamtx and mavlink-routerd run as bare processes (started by
# scripts/run/start_nomad_full.sh), NOT as systemd services.  Only nomad
# (edge_core) has a real systemd unit.  Status checks therefore use
# pgrep and restarts use pkill + nohup.
COMMAND_WHITELIST: dict[str, str] = {
    # --- Service status (pgrep for bare processes, systemctl for systemd) ---
    "status_mediamtx": "pgrep -x mediamtx > /dev/null && echo active || echo inactive",
    "status_mavlink": "pgrep -f mavlink-routerd > /dev/null && echo active || echo inactive",
    "status_novnc": "if systemctl --user is-active --quiet novnc 2>/dev/null || systemctl is-active --quiet novnc 2>/dev/null || (pgrep -f '[w]ebsockify.*6080' >/dev/null && pgrep -f '[x]11vnc.*-rfbport 5900' >/dev/null); then echo active; else echo inactive; fi",
    "status_nomad": "systemctl is-active nomad",
    # --- Service restart ---
    "restart_video": "pkill -x mediamtx 2>/dev/null; sleep 1; nohup mediamtx ~/NOMAD/infra/mediamtx.yml > ~/nomad_logs/mediamtx.log 2>&1 & sleep 1; pgrep -x mediamtx > /dev/null && echo restarted || echo failed",
    "restart_mavlink": "pkill -9 mavlink-routerd 2>/dev/null; sleep 3; for i in {1..5}; do lsof -i :14550 2>/dev/null || break; [ $i -lt 5 ] && sleep 1; done; [ -e /dev/ttyACM0 ] && { GCS=${GCS_IP:-100.76.127.17}; nohup mavlink-routerd -e \"${GCS}:14550\" -e 127.0.0.1:14550 /dev/ttyACM0 > ~/nomad_logs/mavlink.log 2>&1 & sleep 2; pgrep -f mavlink-routerd > /dev/null && echo restarted || (cat ~/nomad_logs/mavlink.log; echo failed); } || echo 'no CubePilot'",
    "restart_edge_core": "nohup bash -c 'sleep 2 && sudo systemctl restart nomad' > /dev/null 2>&1 & echo 'restart scheduled'",
    # --- Service start / stop ---
    "start_mediamtx": "pgrep -x mediamtx > /dev/null && echo 'already running' || (nohup mediamtx ~/NOMAD/infra/mediamtx.yml > ~/nomad_logs/mediamtx.log 2>&1 & sleep 1; echo started)",
    "stop_mediamtx": "pkill -x mediamtx 2>&1 && echo stopped || echo 'not running'",
    "start_mavlink": "[ -e /dev/ttyACM0 ] && { pgrep -f mavlink-routerd > /dev/null && echo 'already running' || { GCS=${GCS_IP:-100.76.127.17}; nohup mavlink-routerd -e \"${GCS}:14550\" -e 127.0.0.1:14550 /dev/ttyACM0 > ~/nomad_logs/mavlink.log 2>&1 & sleep 2; pgrep -f mavlink-routerd > /dev/null && echo started || (cat ~/nomad_logs/mavlink.log; echo failed); }; } || echo 'no CubePilot'",
    "stop_mavlink": "pkill -f mavlink-routerd 2>&1 && echo stopped || echo 'not running'",
    "start_novnc": "if ss -ltn | grep -q ':6080 '; then echo 'already running'; else mkdir -p ~/nomad_logs; pgrep -x Xvfb >/dev/null || (screen=${NOVNC_GEOMETRY:-1920x1080}x24; nohup Xvfb :1 -screen 0 \"$screen\" -ac +extension RANDR > ~/nomad_logs/xvfb.log 2>&1 & sleep 1); pgrep -x openbox >/dev/null || (DISPLAY=:1 nohup openbox-session > ~/nomad_logs/openbox.log 2>&1 & sleep 1); command -v tint2 >/dev/null 2>&1 && (pgrep -x tint2 >/dev/null || (DISPLAY=:1 nohup tint2 > ~/nomad_logs/tint2.log 2>&1 & sleep 1)); if ! ss -ltn | grep -q ':5900 '; then x11vnc -display :1 -rfbport 5900 -localhost -forever -shared -repeat -xkb -noxdamage -noxfixes -noxrecord -bg -passwd ${NOVNC_VNC_PASSWORD:-skibidi123} -o ~/nomad_logs/x11vnc.log >/dev/null 2>&1; fi; for i in 1 2 3 4 5 6 7 8 9 10; do ss -ltn | grep -q ':5900 ' && break; sleep 1; done; if ! ss -ltn | grep -q ':5900 '; then echo failed; exit 1; fi; nohup websockify --heartbeat 30 --web /usr/share/novnc/ 6080 localhost:5900 > ~/nomad_logs/novnc.log 2>&1 & for i in 1 2 3 4 5 6 7 8 9 10; do ss -ltn | grep -q ':6080 ' && break; sleep 1; done; ss -ltn | grep -q ':6080 ' && echo started || (echo failed; exit 1); fi",
    "stop_novnc": "stopped=0; pkill -f '[w]ebsockify.*6080' 2>/dev/null && stopped=1; pkill -f '[x]11vnc.*-rfbport 5900' 2>/dev/null && stopped=1; pkill -x tint2 2>/dev/null && stopped=1; pkill -x openbox 2>/dev/null && stopped=1; pkill -f '[X]vfb :1' 2>/dev/null && stopped=1; [ $stopped -eq 1 ] && echo stopped || echo 'not running'",
    "start_nomad": "sudo systemctl start nomad 2>&1 && echo started || echo failed",
    "stop_nomad": "nohup bash -c 'sleep 2 && sudo systemctl stop nomad' > /dev/null 2>&1 & echo 'stop scheduled'",
    # --- System commands ---
    "reboot_jetson": "nohup bash -c 'sleep 2 && sudo reboot' > /dev/null 2>&1 & echo 'reboot scheduled'",
    "shutdown_jetson": "nohup bash -c 'sleep 2 && sudo shutdown -h now' > /dev/null 2>&1 & echo 'shutdown scheduled'",
    "check_disk": "df -h",
    "check_memory": "free -h",
    "check_processes": "ps aux | head -20",
    "tailscale_status": "tailscale status",
    "network_info": "ip addr show",
    "gpu_status": "tegrastats --interval 1000 --stop 2",
}


# ==================== Helper Functions ====================


class TerminalCommandRequest(BaseModel):
    """Request model for terminal command execution."""

    command_name: str  # Must be a key in COMMAND_WHITELIST
    timeout: int = 10


class TerminalExecRequest(BaseModel):
    """Request model for arbitrary terminal command execution."""

    command: str
    timeout: int = 30
    cwd: Optional[str] = None  # Working directory (persistent cd support)


class TerminalCommandResponse(BaseModel):
    """Response model for terminal command."""

    success: bool
    stdout: str
    stderr: str
    return_code: int
    command_executed: Optional[str] = None
    cwd: Optional[str] = None  # Current working directory after execution


class VIOUpdateRequest(BaseModel):
    """Request model for VIO pose update from ROS bridge."""

    timestamp: float
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    confidence: float = 1.0
    source: str = "external"
    # Raw ROS-frame pose (odom/map) for SLAM 3D visualization
    ros_x: float = 0.0
    ros_y: float = 0.0
    ros_z: float = 0.0
    # Raw ROS-frame orientation (same frame as mesh vertices)
    ros_roll: float = 0.0
    ros_pitch: float = 0.0
    ros_yaw: float = 0.0
    # Optional body attitude in ROS frame (camera attitude with gimbal pitch removed)
    body_roll: Optional[float] = None
    body_pitch: Optional[float] = None
    body_yaw: Optional[float] = None
    # Coordinate frame for ros_* pose fields: REP-103 odom frame (X-forward, Y-left, Z-up)
    frame_id: str = "ros_odom"


class VIOAreaSaveRequest(BaseModel):
    """Request model for saving a VIO relocalization area map."""

    file_path: str
    wait_for_completion: bool = True
    timeout_s: float = 30.0


class VIOAreaLoadRequest(BaseModel):
    """Request model for loading a VIO relocalization area map."""

    file_path: str


class NavVelocityRequest(BaseModel):
    """Request model for navigation velocity command from ROS nav2/nvblox."""

    timestamp: float
    vx: float  # Forward velocity (m/s)
    vy: float  # Lateral velocity (m/s)
    vz: float  # Vertical velocity (m/s)
    yaw_rate: float  # Yaw rate (rad/s)
    source: str = "nav2"


class NavPositionRequest(BaseModel):
    """Request model for navigation position target."""

    x: float  # North position (NED meters)
    y: float  # East position (NED meters)
    z: float  # Down position (NED meters)
    yaw: float  # Heading (radians)
    source: str = "nav2"


class GDriveUploadRequest(BaseModel):
    """Request model for generic Google Drive file upload."""

    local_path: str
    filename: str
    folder_id: Optional[str] = None


class GDriveUploadResponse(BaseModel):
    """Response model for Google Drive upload."""

    success: bool
    file_id: str = ""
    error: str = ""


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

    # Nav2 goal state (Jetson-side obstacle avoidance via nav2 stack)
    app.state.nav2_pending_goal = None  # Goal waiting to be picked up by bridge
    app.state.nav2_current_status = {"status": "idle"}  # Latest feedback from bridge
    app.state.nav2_last_result = None  # Last completed goal result

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
    app.state.foxglove_enabled: bool = False  # Foxglove bridge running state
    app.state.foxglove_port: int = 8765
    app.state.isaac_runtime_cache = {
        "timestamp": 0.0,
        "container_running": False,
        "nvblox_running": False,
        "bridge_running": False,
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

            # Store ROS-frame pose for SLAM 3D WebSocket (same frame as mesh vertices)
            # REP-103 odom frame: X-forward, Y-left, Z-up (standard ROS convention)
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
                # Canonical SLAM frame identifier end-to-end: "odom" (REP-103).
                # Bridge, mesh endpoint, ws_slam, and Mission Planner all agree on this.
                "frame_id": getattr(vio_request, "frame_id", "odom"),
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

    @app.on_event("startup")
    async def _startup_high_rate_zmq_listener() -> None:
        """Start high-rate ZMQ listener on API startup."""
        _start_high_rate_zmq_listener()

    @app.on_event("shutdown")
    async def _shutdown_high_rate_zmq_listener() -> None:
        """Stop high-rate ZMQ listener on API shutdown."""
        _stop_high_rate_zmq_listener()

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

    def _probe_isaac_runtime_state(force_refresh: bool = False) -> dict[str, bool]:
        """Probe Isaac ROS container/bridge process state with short cache grace."""
        now = time.time()
        cache = getattr(app.state, "isaac_runtime_cache", {}) or {}
        cache_age_s = now - float(cache.get("timestamp", 0.0))
        cache_max_stale_s = 20.0

        # Throttle probe frequency to avoid expensive docker exec churn.
        if not force_refresh and cache and cache_age_s < 1.5:
            return {
                "container_running": bool(cache.get("container_running", False)),
                "nvblox_running": bool(cache.get("nvblox_running", False)),
                "bridge_running": bool(cache.get("bridge_running", False)),
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
        if container_running:
            nvblox_probe = _docker_exec_pgrep(
                "nomad_isaac_ros",
                "component_container_mt|component_container|zed_example.launch.py|nomad_zed_nvblox.launch.py",
                timeout_s=5,
            )
            bridge_probe = _docker_exec_pgrep(
                "nomad_isaac_ros",
                "ros_http_bridge.py|ros_http_bridge",
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

        app.state.isaac_runtime_cache = {
            "timestamp": now,
            "container_running": container_running,
            "nvblox_running": nvblox_running,
            "bridge_running": bridge_running,
        }
        if container_running and nvblox_running and bridge_running:
            app.state.isaac_startup_last_initiated = 0.0

        return {
            "container_running": container_running,
            "nvblox_running": nvblox_running,
            "bridge_running": bridge_running,
        }

    def _launch_nvblox_bridge_with_od(enable_od: bool) -> dict:
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
pkill -f ros_http_bridge 2>/dev/null || true
pkill -f 'target_localizer_node|target_localizer\\.launch\\.py' 2>/dev/null || true
pkill -f 'servo_tf_publisher\.py|obstacle_distance_bridge\.py|nav2_goal_bridge\.py|robot_state_publisher|static_transform_publisher' 2>/dev/null || true
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

# Ensure NITROS is enabled for ZED SDK 5.x (zero-copy depth transport to nvblox).
# The nvblox_examples zed_common.yaml may omit disable_nitros, causing the ZED
# wrapper to default to NITROS disabled.  This patches the debug section.
python3 << 'PYEOF_NITROS'
import yaml
common_path = "/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/sensors/zed_common.yaml"
try:
    with open(common_path, 'r') as f:
        common = yaml.safe_load(f) or {{}}
    for key in common:
        if isinstance(common[key], dict) and 'ros__parameters' in common[key]:
            params = common[key]['ros__parameters']
            if 'debug' not in params:
                params['debug'] = {{}}
            params['debug']['disable_nitros'] = False
            params['debug']['debug_nitros'] = False
            break
    with open(common_path, 'w') as f:
        yaml.safe_dump(common, f, default_flow_style=False, sort_keys=False)
    print("Ensured NITROS enabled (disable_nitros: false)")
except Exception as e:
    print("NITROS config warning: " + str(e))
PYEOF_NITROS

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

NAV2_FLAG=false
if [ "${{NOMAD_ENABLE_NAV2:-false}}" = "true" ] && [ "$ENABLE_OD" != "true" ]; then
    NAV2_FLAG=true
fi

ros2 launch "$NOMAD_LAUNCH" enable_nav2:=$NAV2_FLAG enable_od:=$ENABLE_OD &
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

# Wait for topics then launch bridge
sleep 2
{bridge_env_prefix}python3 /workspaces/isaac_ros-dev/edge_core/ros_http_bridge.py --host localhost --port 8000 --rate 30 --vio-topic /zed/zed_node/odom --mag-topic /zed/zed_node/imu/mag --mesh-topic /nvblox_node/color_layer_marker --high-rate-transport http &
echo $! > /tmp/ros_bridge.pid

# Launch Task 1 target-localizer only when OD mode is enabled.
if [ "$ENABLE_OD" = "true" ]; then
    PYTHONPATH=/workspaces/isaac_ros-dev/edge_core/target_localizer:${{PYTHONPATH:-}} \
    python3 -m target_localizer.target_localizer_node \
        --ros-args \
        -p output_dir:=/home/mad/NOMAD/data/task1_captures \
        -p team_name:=MAD \
        -r /zed2i/zed_node/rgb/image_rect_color:=/zed/zed_node/rgb/color/rect/image \
        -r /zed2i/zed_node/depth/depth_registered:=/zed/zed_node/depth/depth_registered \
        -r /zed2i/zed_node/rgb/camera_info:=/zed/zed_node/rgb/color/rect/camera_info &
    echo $! > /tmp/target_localizer.pid

    sleep 2
    if ! kill -0 "$(cat /tmp/target_localizer.pid 2>/dev/null)" 2>/dev/null; then
        echo "ERROR: target_localizer process exited early"
        exit 6
    fi

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

    # ==================== Root / Health ====================

    @app.get("/", tags=["System"])
    async def root():
        """API root - returns service information."""
        return {
            "service": "NOMAD Edge Core",
            "version": "1.0.0",
            "platform": "Jetson Orin Nano",
            "description": "Connect via Mission Planner plugin for full control",
            "endpoints": {
                "health": "/health",
                "status": "/status",
                "task1": "/api/task/1/*",
                "task2": "/api/task/2/*",
                "terminal_run": "/api/terminal/run",
                "terminal_exec": "/api/terminal/exec",
                "vio": "/api/vio/*",
            },
        }

    # ==================== Health Endpoints ====================

    @app.get("/health", tags=["System"])
    async def health_check(request: Request):
        """
        Comprehensive health check endpoint.

        Returns system health including CPU/GPU temperatures,
        memory usage, VIO status, and network connectivity.
        """
        state = request.app.state.state_manager.get_state()

        # Base health response
        response = {
            "status": "ok" if state.connected else "degraded",
            "connected": state.connected,
            "gps_fix": state.gps_fix,
            "flight_mode": state.flight_mode,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }

        # Add Jetson health metrics if available
        health_monitor = request.app.state.health_monitor
        if health_monitor:
            health = health_monitor.health
            response.update(
                {
                    "cpu_temp": health.cpu_temp_c,
                    "cpu_load": health.cpu_load_pct,
                    "gpu_temp": health.gpu_temp_c,
                    "gpu_load": health.gpu_load_pct,
                    "memory_used_pct": health.memory_used_pct,
                    "disk_free_gb": health.disk_free_gb,
                    "power_draw_w": health.power_draw_w,
                    "throttled": health.throttled,
                    "thermal_zone": health.thermal_zone,
                    "tailscale_connected": health.tailscale_connected,
                    "tailscale_ip": health.tailscale_ip,
                }
            )

            # Override status based on thermal state
            if health.thermal_zone == "critical":
                response["status"] = "critical"
            elif health.thermal_zone == "warning" or health.throttled:
                response["status"] = "warning"

        # Add VIO health from external source (ros_http_bridge) if available
        external_vio = _get_vio_snapshot()["external_vio_state"]
        if external_vio:
            confidence_0_1 = external_vio.get("confidence", 0)
            response["vio"] = {
                "health": "healthy" if confidence_0_1 > 0.5 else "degraded",
                "tracking_confidence": confidence_0_1,
                "message_rate_hz": 30.0,
            }

        return response

    @app.get("/health/detailed", tags=["System"])
    async def detailed_health(request: Request):
        """Get detailed health metrics for monitoring dashboard."""
        health_monitor = request.app.state.health_monitor
        if not health_monitor:
            return {"error": "Health monitor not initialized"}

        result = health_monitor.health.to_dict()

        # Include VIO health from external source (ros_http_bridge)
        external_vio = _get_vio_snapshot()["external_vio_state"]
        if external_vio:
            confidence_0_1 = external_vio.get("confidence", 0)
            result["vio"] = {
                "health": "healthy" if confidence_0_1 > 0.5 else "degraded",
                "tracking_confidence": confidence_0_1,
                "message_rate_hz": external_vio.get("message_rate_hz", 30.0),
                "source": external_vio.get("source", "unknown"),
            }

        return result

    # ==================== Status Endpoints ====================

    @app.get("/status", tags=["System"])
    async def get_status(request: Request):
        """Get current system state including all telemetry."""
        state = request.app.state.state_manager.get_state()
        return jsonable_encoder(state)

    # =================== Network Endpoints =====================

    @app.get("/network/status", tags=["Network"])
    async def network_status(request: Request):
        """Get current network + tailscale status."""
        tailscale = None
        modem = None
        internet_reachable = False
        gcs_reachable = False

        tailscale_manager = request.app.state.tailscale_manager
        if tailscale_manager:
            info = tailscale_manager.info
            tailscale = {
                "status": info.status.value
                if getattr(info, "status", None)
                else "unknown",
                "ip": getattr(info, "ip_address", None),
                "hostname": getattr(info, "hostname", "unknown"),
                "peer_count": getattr(info, "peer_count", None),
                "latency_ms": getattr(info, "latency_ms", None),
            }

        network_monitor = request.app.state.network_monitor
        if network_monitor:
            status = network_monitor.status
            internet_reachable = bool(getattr(status, "internet_reachable", False))
            gcs_reachable = bool(getattr(status, "tailscale_reachable", False))

            if getattr(status, "modem", None):
                modem_obj = status.modem
                modem = {
                    "connected": modem_obj.connected,
                    "signal_strength_dbm": modem_obj.signal_strength_dbm,
                    "signal_quality": modem_obj.signal_quality.value,
                    "carrier": modem_obj.carrier,
                    "technology": modem_obj.technology,
                }
        return {
            "tailscale": tailscale,
            "modem": modem,
            "internet_reachable": internet_reachable,
            "gcs_reachable": gcs_reachable,
        }

    @app.post("/network/reconnect", tags=["Network"])
    async def network_reconnect(request: Request):
        """Trigger Tailscale reconnection."""
        tailscale_manager = request.app.state.tailscale_manager
        if not tailscale_manager:
            raise HTTPException(
                status_code=503, detail="Tailscale manager not initialized"
            )

        ok = await tailscale_manager.reconnect()
        return {
            "success": ok,
            "message": "Tailscale reconnection triggered"
            if ok
            else "Failed to trigger reconnection",
        }

    @app.get("/network/ping/{host}", tags=["Network"])
    async def network_ping(host: str):
        try:
            result = subprocess.run(
                ["ping", "-c", "3", host],
                capture_output=True,
                text=True,
                timeout=5,
            )

            out = result.stdout + "\n" + result.stderr

            # Extract packet counts
            sent = received = None
            m = re.search(r"(\d+)\s+packets transmitted,\s+(\d+)\s+received", out)
            if m:
                sent = int(m.group(1))
                received = int(m.group(2))

            # Extract average latency (ms)
            latency_ms = None
            m = re.search(
                r"rtt [^=]+= ([\d\.]+)/([\d\.]+)/([\d\.]+)/([\d\.]+)\s*ms",
                out,
            )
            if m:
                latency_ms = float(m.group(2))

            if result.returncode != 0 and received in (0, None):
                raise HTTPException(
                    status_code=502,
                    detail=f"Ping failed: {out.strip()[:300]}",
                )

            return {
                "host": host,
                "latency_ms": latency_ms,
                "packets_sent": sent,
                "packets_received": received,
            }

        except subprocess.TimeoutExpired:
            raise HTTPException(status_code=504, detail="Ping timed out")

    async def _validate_ws_token(websocket: WebSocket) -> bool:
        """Validate API key token on WebSocket connect. Returns True if authorised."""
        if _NOMAD_API_KEY is None:
            return True
        token = websocket.query_params.get("token", "")
        if not hmac.compare_digest(token, _NOMAD_API_KEY):
            await websocket.close(code=4003, reason="Unauthorized")
            return False
        return True

    @app.websocket("/ws/state")
    async def ws_state(websocket: WebSocket):
        """WebSocket endpoint for real-time state updates (10Hz)."""
        if not await _validate_ws_token(websocket):
            return
        await websocket.accept()
        try:
            while True:
                state = websocket.app.state.state_manager.get_state()
                data = jsonable_encoder(state)

                # Add additional real-time data
                health_monitor = websocket.app.state.health_monitor
                if health_monitor:
                    data["jetson_health"] = health_monitor.health.to_dict()
                external_vio = _get_vio_snapshot()["external_vio_state"]
                if external_vio:
                    data["vio_status"] = external_vio

                await websocket.send_json(data)
                await asyncio.sleep(0.1)  # 10Hz
        except WebSocketDisconnect:
            return

    @app.websocket("/ws/slam")
    async def ws_slam(websocket: WebSocket):
        """
        WebSocket endpoint for real-time SLAM 3D visualization (30Hz pose, mesh on change).

        Pushes two types of messages:
        - type="pose": drone position/attitude at 30Hz (~100 bytes)
        - type="mesh": mesh delta when new data arrives (only changed blocks/voxels)

        The client (SLAM3DView) connects once and receives a continuous stream.
        All poses are in REP-103 odom frame (X-forward, Y-left, Z-up).
        """
        if not await _validate_ws_token(websocket):
            return
        await websocket.accept()
        last_mesh_timestamp = None
        frame_count = 0
        has_last_good_attitude = False
        last_good_roll = 0.0
        last_good_pitch = 0.0
        last_good_yaw = 0.0
        target_interval = 1.0 / 30.0
        next_tick = asyncio.get_running_loop().time()
        try:
            while True:
                frame = {"type": "pose", "ts": frame_count}
                # Canonical SLAM frame identifier (REP-103 odom). Same string the
                # bridge uses for mesh payloads and the Mission Planner client expects.
                frame_id = "odom"
                has_position = False
                has_attitude = False

                # Check for mesh updates -- mesh-bundled pose is in the
                # same coordinate frame as the mesh vertices (ROS odom/map frame)
                has_mesh = False
                if (
                    hasattr(websocket.app.state, "slam_mesh_data")
                    and websocket.app.state.slam_mesh_data
                ):
                    stored = websocket.app.state.slam_mesh_data
                    mesh_ts = stored.get("received_at")
                    if mesh_ts and mesh_ts != last_mesh_timestamp:
                        last_mesh_timestamp = mesh_ts
                        has_mesh = True
                        frame["type"] = "mesh"
                        frame["mesh"] = stored.get("mesh")
                        # Expose the mesh's ROS-time timestamp so clients can
                        # measure mesh/pose skew and reason about staleness.
                        frame["mesh_ts"] = mesh_ts
                        # Canonical frame is "odom". If the stored mesh payload
                        # has a different frame_id the bridge is misconfigured --
                        # log once, but keep emitting the canonical identifier so
                        # downstream clients have a stable contract.
                        stored_mesh_frame = stored.get("frame_id")
                        if stored_mesh_frame and stored_mesh_frame != frame_id:
                            _mesh_mismatch_log = getattr(
                                websocket.app.state,
                                "_ws_slam_mesh_frame_mismatch_logged",
                                False,
                            )
                            if not _mesh_mismatch_log:
                                logger.warning(
                                    "ws_slam mesh frame_id mismatch: got %r, expected %r",
                                    stored_mesh_frame,
                                    frame_id,
                                )
                                websocket.app.state._ws_slam_mesh_frame_mismatch_logged = True

                        # Prefer mesh-bundled drone pose for mesh frames so mesh
                        # vertices and the drone marker originate from the same
                        # ROS-time snapshot. This matches how RViz renders mesh
                        # plus TF at a single consistent moment rather than
                        # mixing older mesh with freshest VIO. Pose-only frames
                        # continue to use the live ros_vio snapshot below.
                        mesh_position = stored.get("drone_position")
                        if isinstance(mesh_position, dict):
                            frame["x"] = float(mesh_position.get("x", 0) or 0)
                            frame["y"] = float(mesh_position.get("y", 0) or 0)
                            frame["z"] = float(mesh_position.get("z", 0) or 0)
                            has_position = True
                            frame["pose_source"] = "mesh"

                        mesh_attitude = stored.get("drone_attitude")
                        if isinstance(mesh_attitude, dict):
                            m_roll = mesh_attitude.get("roll")
                            m_pitch = mesh_attitude.get("pitch")
                            m_yaw = mesh_attitude.get("yaw")
                            if m_roll is not None and m_pitch is not None and m_yaw is not None:
                                frame["roll"] = float(m_roll)
                                frame["pitch"] = float(m_pitch)
                                frame["yaw"] = float(m_yaw)
                                frame["body_roll"] = float(m_roll)
                                frame["body_pitch"] = float(m_pitch)
                                frame["body_yaw"] = float(m_yaw)
                                frame["attitude_valid"] = True
                                last_good_roll = float(m_roll)
                                last_good_pitch = float(m_pitch)
                                last_good_yaw = float(m_yaw)
                                has_last_good_attitude = True
                                has_attitude = True

                # Fall back to ROS-frame VIO for pose (used for pose-only frames
                # and for mesh frames that only include one of position/attitude).
                # frame_id is always "odom" (REP-103) for all SLAM pose data.
                ros_vio = _get_vio_snapshot()["slam_vio_ros_frame"]
                if ros_vio:
                    # Only honor the stored frame_id if it matches the canonical
                    # identifier -- mismatched frames indicate a stale/misrouted
                    # publisher and must not silently corrupt the ws_slam contract.
                    stored_frame = ros_vio.get("frame_id")
                    if stored_frame and stored_frame != frame_id:
                        _frame_mismatch_log = getattr(
                            websocket.app.state, "_ws_slam_frame_mismatch_logged", False
                        )
                        if not _frame_mismatch_log:
                            logger.warning(
                                "ws_slam pose frame_id mismatch: got %r, expected %r",
                                stored_frame,
                                frame_id,
                            )
                            websocket.app.state._ws_slam_frame_mismatch_logged = True
                    if not has_position:
                        frame["x"] = ros_vio.get("x", 0)
                        frame["y"] = ros_vio.get("y", 0)
                        frame["z"] = ros_vio.get("z", 0)
                        has_position = True
                        frame.setdefault("pose_source", "vio")

                    # For pose-only frames, pull attitude from the freshest VIO
                    # snapshot. For mesh frames we already populated attitude
                    # from the mesh-bundled drone pose (same ROS-time as the
                    # mesh vertices); leave that intact to keep mesh + drone
                    # marker temporally consistent.
                    body_roll = ros_vio.get("body_roll")
                    body_pitch = ros_vio.get("body_pitch")
                    body_yaw = ros_vio.get("body_yaw")
                    body_attitude_available = (
                        body_roll is not None and body_pitch is not None and body_yaw is not None
                    )

                    if not has_attitude:
                        roll = float(
                            (
                                body_roll
                                if body_attitude_available
                                else ros_vio.get("roll", 0)
                            )
                            or 0
                        )
                        pitch = float(
                            (
                                body_pitch
                                if body_attitude_available
                                else ros_vio.get("pitch", 0)
                            )
                            or 0
                        )
                        yaw = float(
                            (
                                body_yaw
                                if body_attitude_available
                                else ros_vio.get("yaw", 0)
                            )
                            or 0
                        )
                        if all(abs(v) <= 1e-4 for v in (roll, pitch, yaw)):
                            if has_last_good_attitude:
                                frame["roll"] = last_good_roll
                                frame["pitch"] = last_good_pitch
                                frame["yaw"] = last_good_yaw
                                frame["attitude_valid"] = False
                                has_attitude = True
                        else:
                            frame["roll"] = roll
                            frame["pitch"] = pitch
                            frame["yaw"] = yaw
                            if body_attitude_available:
                                frame["body_roll"] = float(body_roll)
                                frame["body_pitch"] = float(body_pitch)
                                frame["body_yaw"] = float(body_yaw)
                            frame["attitude_valid"] = True
                            last_good_roll = roll
                            last_good_pitch = pitch
                            last_good_yaw = yaw
                            has_last_good_attitude = True
                            has_attitude = True

                if not has_attitude and has_last_good_attitude:
                    frame["roll"] = last_good_roll
                    frame["pitch"] = last_good_pitch
                    frame["yaw"] = last_good_yaw
                    frame["attitude_valid"] = False
                    has_attitude = True

                # Always include velocity from external VIO state when available
                external_vio = _get_vio_snapshot()["external_vio_state"]
                if external_vio:
                    frame["vx"] = external_vio.get("vx", 0)
                    frame["vy"] = external_vio.get("vy", 0)
                    frame["vz"] = external_vio.get("vz", 0)

                has_pose = has_position or has_attitude
                frame["frame_id"] = frame_id

                # Skip frames when no pose data available at all
                if not has_pose and not has_mesh:
                    frame_count += 1
                    next_tick += target_interval
                    now = asyncio.get_running_loop().time()
                    sleep_for = next_tick - now
                    if sleep_for > 0:
                        await asyncio.sleep(sleep_for)
                    else:
                        next_tick = now
                    continue

                # Include detection markers every 30th frame (~1Hz) to keep
                # SLAM pose/mesh stream responsive under heavy load.
                if frame_count % 30 == 0:
                    with websocket.app.state.detection_state_lock:
                        det_history = list(websocket.app.state.detection_history)
                    if det_history:
                        # Cap to 50 most recent detections to limit payload size
                        capped = (
                            det_history[-50:] if len(det_history) > 50 else det_history
                        )
                        frame["detections"] = [
                            {
                                "label": d.get("label", ""),
                                "x": d.get("x", 0),
                                "y": d.get("y", 0),
                                "z": d.get("z", 0),
                                "confidence": d.get("confidence", 0),
                                "seen_count": d.get("seen_count", 1),
                            }
                            for d in capped
                            if d.get("x") is not None
                        ]
                    else:
                        # Explicitly signal empty detections so client can clear markers
                        frame["detections"] = []

                await websocket.send_json(frame)
                frame_count += 1
                next_tick += target_interval
                now = asyncio.get_running_loop().time()
                sleep_for = next_tick - now
                if sleep_for > 0:
                    await asyncio.sleep(sleep_for)
                else:
                    # If we fall behind, resync to avoid accumulating drift.
                    next_tick = now
        except WebSocketDisconnect:
            logger.debug("SLAM WebSocket client disconnected")
            return
        except Exception as e:
            logger.warning(f"SLAM WebSocket error: {e}")

    # ==================== Task 1: Recon (Outdoor) ====================

    @app.post("/api/task/1/capture", tags=["Task 1"])
    async def task1_capture(request: Request):
        """
        Trigger target detection for Task 1 recon mission.

        Delegates to the target_localizer ROS 2 node running in the Isaac
        ROS container. The node captures the current ZED frame, runs HSV
        circle detection, back-projects to 3D, and generates a description.
        """
        try:
            output = _call_ros2_service_in_isaac_container_or_raise(
                service_name="/target_localizer/capture_target",
                service_type="std_srvs/srv/Trigger",
                request_payload={},
                timeout_s=45.0,
            )
            return {
                "success": True,
                "output": output,
                "mode": "target_localizer",
            }
        except HTTPException as exc:
            # Fallback to the live detections cache so Task 1 capture can still
            # produce useful output when the target_localizer service is unavailable.
            detail = str(exc.detail)
            with request.app.state.detection_state_lock:
                current = list(request.app.state.detected_objects)
                history = list(request.app.state.detection_history)
                last_update = request.app.state.detection_last_update

            fallback_candidates = current if current else history[-10:]
            age_seconds = time.time() - last_update if last_update > 0 else None

            if fallback_candidates:
                best = max(
                    fallback_candidates,
                    key=lambda d: float(d.get("confidence", 0.0) or 0.0),
                )
                label = best.get("label") or best.get("class_name") or "target"
                confidence = float(best.get("confidence", 0.0) or 0.0)
                x = best.get("x")
                y = best.get("y")
                z = best.get("z")
                pos = ""
                if x is not None and y is not None and z is not None:
                    pos = f" at ({float(x):.2f}, {float(y):.2f}, {float(z):.2f})"

                output = (
                    f"Fallback capture: top detection '{label}' "
                    f"(confidence {confidence:.2f}){pos}."
                )
                return {
                    "success": True,
                    "output": output,
                    "mode": "detections_fallback",
                    "detail": detail,
                    "current_count": len(current),
                    "history_count": len(history),
                    "age_seconds": age_seconds,
                }

            return {
                "success": False,
                "output": "Capture fallback has no detections available.",
                "mode": "detections_fallback",
                "detail": detail,
                "current_count": len(current),
                "history_count": len(history),
                "age_seconds": age_seconds,
            }

    @app.get("/api/task/1/images/{filename}", tags=["Task 1"])
    async def task1_get_image(filename: str):
        """
        Retrieve a saved Task 1 image (legacy endpoint for backward compatibility).

        Returns the image file captured during a Task 1 recon mission.
        For new folder-based structure, use /api/task/1/images/{folder}/{filename}
        """
        image_dir = "./data/task1_captures"
        image_path = os.path.join(image_dir, filename)

        # Validate filename to prevent directory traversal
        if ".." in filename or "/" in filename or "\\" in filename:
            raise HTTPException(status_code=400, detail="Invalid filename")

        # Check if file exists
        if not os.path.exists(image_path):
            # Backward-compatible fallback for folder-based captures.
            if os.path.isdir(image_dir):
                for folder in sorted(os.listdir(image_dir), reverse=True):
                    candidate_path = os.path.join(image_dir, folder, filename)
                    if os.path.isfile(candidate_path):
                        image_path = candidate_path
                        break
                else:
                    raise HTTPException(status_code=404, detail="Image not found")
            else:
                raise HTTPException(status_code=404, detail="Image not found")

        return FileResponse(image_path, media_type="image/jpeg")

    @app.get("/api/task/1/captures", tags=["Task 1"], response_model=Task1CapturesList)
    async def list_task1_captures():
        """
        List all Task 1 capture folders.

        Returns: List of folder names (timestamps) sorted by date descending.
        Example: ["20260202_120000", "20260202_115500", ...]
        """
        base_dir = "./data/task1_captures"

        # Create directory if it doesn't exist
        if not os.path.exists(base_dir):
            os.makedirs(base_dir, exist_ok=True)
            return Task1CapturesList(captures=[], count=0)

        try:
            # List all directories in task1_captures
            entries = os.listdir(base_dir)
            folders = [
                entry
                for entry in entries
                if os.path.isdir(os.path.join(base_dir, entry))
            ]

            # Filter to only timestamp-pattern folders (YYYYMMDD_HHMMSS)
            timestamp_pattern = re.compile(r"^\d{8}_\d{6}$")
            valid_folders = [
                folder for folder in folders if timestamp_pattern.match(folder)
            ]

            # Sort by timestamp descending (newest first)
            valid_folders.sort(reverse=True)

            return Task1CapturesList(captures=valid_folders, count=len(valid_folders))

        except Exception as e:
            logger.error(f"Failed to list Task 1 captures: {e}")
            raise HTTPException(
                status_code=500, detail=f"Failed to list captures: {str(e)}"
            )

    @app.get("/api/task/1/images/{folder}/{filename}", tags=["Task 1"])
    async def get_task1_image_with_folder(folder: str, filename: str):
        """
        Download specific file from Task 1 capture folder.

        Args:
            folder: Folder name (e.g., "20260202_120000")
            filename: File name (e.g., "photo.jpg", "metadata.json")

        Returns: File content with appropriate content type
        """
        # Security: Validate folder name matches timestamp pattern
        timestamp_pattern = re.compile(r"^\d{8}_\d{6}$")
        if not timestamp_pattern.match(folder):
            raise HTTPException(status_code=400, detail="Invalid folder name format")

        # Security: Validate filename - whitelist allowed files
        allowed_files = [
            "photo.jpg",
            "metadata.json",
            "description.txt",
            "detections_overlay.jpg",
            "building_3d_snapshot.jpg",
        ]
        if filename not in allowed_files:
            raise HTTPException(
                status_code=400,
                detail=f"Invalid filename. Allowed: {', '.join(allowed_files)}",
            )

        # Security: Prevent path traversal
        if ".." in folder or "/" in folder or "\\" in folder:
            raise HTTPException(status_code=400, detail="Invalid folder name")
        if ".." in filename or "/" in filename or "\\" in filename:
            raise HTTPException(status_code=400, detail="Invalid filename")

        # Build file path
        base_dir = "./data/task1_captures"
        file_path = os.path.join(base_dir, folder, filename)

        # Normalize path and ensure it's within base_dir (additional security)
        base_dir_abs = os.path.abspath(base_dir)
        file_path_abs = os.path.abspath(file_path)
        if not file_path_abs.startswith(base_dir_abs):
            raise HTTPException(status_code=400, detail="Invalid file path")

        # Check if file exists
        if not os.path.exists(file_path):
            raise HTTPException(
                status_code=404, detail=f"File not found: {folder}/{filename}"
            )

        # Determine media type
        media_type = "application/octet-stream"
        if filename.endswith(".jpg") or filename.endswith(".jpeg"):
            media_type = "image/jpeg"
        elif filename.endswith(".json"):
            media_type = "application/json"
        elif filename.endswith(".txt"):
            media_type = "text/plain"

        return FileResponse(file_path, media_type=media_type)

    # ==================== Task 1: Target Localizer (ROS 2) ====================

    @app.post("/api/task/1/target/capture", tags=["Task 1"])
    async def task1_capture_target():
        """
        Trigger the target_localizer ROS 2 node to detect and describe targets.

        Calls the ~/capture_target service which runs HSV circle detection on
        the current ZED frame, back-projects to 3D, determines the building face,
        and generates a ConOps-compliant description.
        """
        output = _call_ros2_service_in_isaac_container_or_raise(
            service_name="/target_localizer/capture_target",
            service_type="std_srvs/srv/Trigger",
            request_payload={},
            timeout_s=45.0,
        )
        return {"success": True, "output": output}

    @app.post("/api/task/1/target/save", tags=["Task 1"])
    async def task1_save_targets():
        """
        Save all captured targets to the competition .txt file.

        Calls the ~/save_targets service which writes Task_1_MAD_targets.txt
        and a debug log to the configured output directory.
        """
        output = _call_ros2_service_in_isaac_container_or_raise(
            service_name="/target_localizer/save_targets",
            service_type="std_srvs/srv/Trigger",
            request_payload={},
            timeout_s=10.0,
        )
        return {"success": True, "output": output}

    @app.get("/api/task/1/target/model", tags=["Task 1"])
    async def task1_print_building_model():
        """
        Print the current building model summary (landmark counts, face info).

        Calls the ~/print_model service on the target_localizer node.
        """
        output = _call_ros2_service_in_isaac_container_or_raise(
            service_name="/target_localizer/print_model",
            service_type="std_srvs/srv/Trigger",
            request_payload={},
            timeout_s=10.0,
        )
        return {"success": True, "output": output}

    # ==================== Google Drive Upload ====================

    @app.get("/api/gdrive/status", tags=["Google Drive"])
    async def gdrive_status():
        """
        Check Google Drive upload capability status.

        Returns whether credentials are configured and valid.
        """
        try:
            from .gdrive_upload import _get_credentials, _get_token_path

            token_path = _get_token_path()
            has_token = os.path.isfile(token_path)

            if not has_token:
                return {
                    "available": False,
                    "message": "No Google Drive token found. Run setup.",
                    "token_path": token_path,
                }

            creds = _get_credentials()
            if creds is None:
                return {
                    "available": False,
                    "message": "Token exists but credentials invalid/expired.",
                    "token_path": token_path,
                }

            folder_id = os.environ.get("GDRIVE_FOLDER_ID", "")

            return {
                "available": True,
                "message": "Google Drive ready",
                "token_path": token_path,
                "folder_configured": bool(folder_id),
            }
        except ImportError:
            return {
                "available": False,
                "message": "Google Drive dependencies not installed",
            }
        except Exception as e:
            return {
                "available": False,
                "message": f"Error checking status: {e}",
            }

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
            return {
                "health": "healthy" if confidence_0_1 > 0.5 else "degraded",
                "tracking_confidence": confidence_0_1,  # 0-1 scale
                "position_valid": True,
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

    def _normalize_vio_area_file_path_or_raise(file_path: str) -> str:
        """Validate and normalize area-map file paths from API requests."""
        normalized = (file_path or "").strip()
        if not normalized:
            raise HTTPException(status_code=400, detail="file_path is required")
        return normalized

    def _resolve_nvblox_empty_map_path(file_path: Optional[str] = None) -> str:
        """Resolve baseline map path used for non-restart nvblox clear workflow."""
        if file_path and str(file_path).strip():
            return _normalize_vio_area_file_path_or_raise(str(file_path))

        env_path = (os.getenv("NVBLOX_EMPTY_MAP_PATH") or "").strip()
        if env_path:
            return env_path

        return "/home/mad/NOMAD/data/area_maps/nvblox_empty_map"

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
            return (
                f"{base}. Action: verify the map path exists on Jetson and retry."
            )
        if "timed out waiting for area map export" in lower:
            return (
                f"{base}. Action: increase timeout_s or retry after camera motion/feature-rich view."
            )
        if "export state api is unavailable" in lower:
            return (
                f"{base}. Action: use wait_for_completion=false for this runtime or upgrade ZED SDK/runtime."
            )
        if "failed to load area map for relocalization" in lower:
            return (
                f"{base}. Action: confirm area file was created on this unit and is not corrupted."
            )
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

    def _call_ros2_service_in_isaac_container_or_raise(
        service_name: str,
        service_type: str,
        request_payload: dict[str, Any],
        timeout_s: float = 30.0,
    ) -> str:
        """Call a ROS2 service inside the active Isaac container stack."""
        runtime = _probe_isaac_runtime_state(force_refresh=True)
        if not runtime.get("container_running", False):
            raise HTTPException(
                status_code=503,
                detail="Isaac ROS container stack is not running",
            )
        if not runtime.get("nvblox_running", False):
            raise HTTPException(
                status_code=503,
                detail="Isaac ROS nvblox stack is not running",
            )

        _ensure_file_path_parent_dir_in_isaac_container_or_raise(
            service_type=service_type,
            request_payload=request_payload,
        )

        request_yaml = json.dumps(request_payload)
        base_env_cmd = (
            "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
            "source /workspaces/isaac_ros-dev/install/setup.bash >/dev/null 2>&1; "
            "export ROS2CLI_DISABLE_DAEMON=1; "
            "export ROS2CLI_NO_DAEMON=1; "
        )

        type_check_cmd = (
            base_env_cmd
            + f"ros2 service type {shlex.quote(service_name)}"
        )
        try:
            type_result = subprocess.run(
                ["docker", "exec", "nomad_isaac_ros", "bash", "-lc", type_check_cmd],
                capture_output=True,
                text=True,
                timeout=10,
            )
        except subprocess.TimeoutExpired:
            raise HTTPException(
                status_code=504,
                detail=f"ROS2 service probe timed out: {service_name}",
            )
        except Exception as e:
            logger.error(f"ROS2 service probe failed ({service_name}): {e}")
            raise HTTPException(
                status_code=503,
                detail=f"Failed to probe ROS2 service: {service_name}",
            )

        type_output = "\n".join(
            part
            for part in [
                (type_result.stdout or "").strip(),
                (type_result.stderr or "").strip(),
            ]
            if part
        ).strip()
        type_output = _sanitize_ros2_service_output(type_output)
        if type_result.returncode != 0 or not type_output:
            raise HTTPException(
                status_code=503,
                detail=type_output or f"ROS2 service not available: {service_name}",
            )

        advertised_type = type_output.splitlines()[-1].strip()
        if advertised_type != service_type:
            raise HTTPException(
                status_code=503,
                detail=(
                    f"ROS2 service type mismatch for {service_name}: "
                    f"expected {service_type}, got {advertised_type}"
                ),
            )

        shell_cmd = base_env_cmd + f"ros2 service call {service_name} {service_type} {shlex.quote(request_yaml)}"

        try:
            result = subprocess.run(
                ["docker", "exec", "nomad_isaac_ros", "bash", "-lc", shell_cmd],
                capture_output=True,
                text=True,
                timeout=max(5, int(timeout_s) + 5),
            )
        except subprocess.TimeoutExpired:
            raise HTTPException(
                status_code=504,
                detail=f"ROS2 service call timed out: {service_name}",
            )
        except Exception as e:
            logger.error(f"ROS2 service exec failed ({service_name}): {e}")
            raise HTTPException(
                status_code=503,
                detail=f"Failed to execute ROS2 service call: {service_name}",
            )

        output = "\n".join(
            part for part in [result.stdout.strip(), result.stderr.strip()] if part
        ).strip()
        output = _sanitize_ros2_service_output(output)
        if result.returncode != 0:
            raise HTTPException(
                status_code=502,
                detail=output or f"ROS2 service call failed: {service_name}",
            )

        success_match = re.search(
            r"\bsuccess\s*[:=]\s*(true|false)\b",
            output,
            flags=re.IGNORECASE,
        )
        success = (
            True if not success_match else (success_match.group(1).lower() == "true")
        )

        message_match = re.search(
            r"message\s*[:=]\s*['\"]?([^\n'\"]+)",
            output,
            flags=re.IGNORECASE,
        )
        if message_match:
            message = message_match.group(1).strip()
        elif service_type == "nvblox_msgs/srv/FilePath":
            message = (
                f"{service_name} returned success={'true' if success else 'false'}"
            )
        else:
            message = output

        if not success:
            status = _status_for_vio_area_failure(message or output, default_status=502)
            raise HTTPException(
                status_code=status,
                detail=message
                or output
                or f"ROS2 service reported failure: {service_name}",
            )

        return message or f"ROS2 service call succeeded: {service_name}"

    @app.post("/api/vio/area/save", tags=["VIO"])
    async def vio_area_save(area_request: VIOAreaSaveRequest, request: Request):
        """Save relocalization map via direct ZED area map backend or nvblox map service."""
        file_path = _normalize_vio_area_file_path_or_raise(area_request.file_path)
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
        file_path = _normalize_vio_area_file_path_or_raise(area_request.file_path)
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
        file_path = _normalize_vio_area_file_path_or_raise(area_request.file_path)
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
        # Clear trajectory on reset
        with request.app.state.vio_state_lock:
            request.app.state.vio_trajectory = []

        camera_service = request.app.state.camera_service
        if camera_service and hasattr(camera_service, "reset_tracking"):
            backend = "direct_zed_area_map"
            try:
                reset_ok = bool(camera_service.reset_tracking())
                return {
                    "success": reset_ok,
                    "reset_counter": 0,
                    "backend": backend,
                    "message": (
                        "Tracking origin reset via direct ZED backend; trajectory cleared"
                        if reset_ok
                        else "Failed to reset tracking via direct ZED backend; trajectory cleared"
                    ),
                }
            except Exception as e:
                logger.error(f"Direct camera reset_tracking failed: {e}")
                return {
                    "success": False,
                    "reset_counter": 0,
                    "backend": backend,
                    "message": f"Direct ZED reset_tracking failed; trajectory cleared: {e}",
                }

        # Keep existing fallback behavior for ros_http_bridge-managed VIO.
        return {
            "success": True,
            "reset_counter": 0,
            "backend": "nvblox_map_service",
            "message": "Trajectory cleared (VIO reset managed via ROS bridge fallback)",
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

    # ==================== Nav2 Obstacle Avoidance (Jetson-side) ====================
    # ArduPlane has no onboard obstacle avoidance. Nav2 runs on the Jetson with
    # nvblox costmap and generates obstacle-avoiding /cmd_vel. The nav2_goal_bridge
    # (ROS2 node inside the container) polls these endpoints to receive goals and
    # report feedback/results back.

    async def _parse_request_json_object(request: Request) -> dict[str, Any]:
        """Parse request JSON and enforce object payloads with controlled 400s."""
        try:
            body = await request.json()
        except (json.JSONDecodeError, UnicodeDecodeError, ValueError) as exc:
            raise HTTPException(status_code=400, detail="Invalid JSON body") from exc

        if not isinstance(body, dict):
            raise HTTPException(status_code=400, detail="JSON body must be an object")

        return body

    @app.get("/api/nav2/status", tags=["Nav2"])
    async def nav2_status(request: Request):
        """Get current Nav2 navigation status and feedback."""
        return {
            "status": request.app.state.nav2_current_status,
            "pending_goal": request.app.state.nav2_pending_goal is not None,
            "last_result": request.app.state.nav2_last_result,
        }

    @app.post("/api/nav2/goal", tags=["Nav2"])
    async def nav2_send_goal(request: Request):
        """
        Send a navigation goal to Nav2 for obstacle-avoiding autonomous flight.

        Goal types:
        - navigate_to_pose: Single pose {x, y, z, yaw} in odom frame
        - navigate_through_poses: List of poses to follow as one path
        - follow_waypoints: List of waypoints (stops at each)
        - cancel: Cancel current navigation

        Example (navigate_to_pose):
            {"type": "navigate_to_pose", "pose": {"x": 2.0, "y": 1.0, "z": 0.0, "yaw": 0.0}}

        Example (follow_waypoints):
            {"type": "follow_waypoints", "waypoints": [
                {"x": 1.0, "y": 0.0, "yaw": 0.0},
                {"x": 2.0, "y": 1.0, "yaw": 1.57}
            ]}
        """
        body = await _parse_request_json_object(request)
        goal_type = body.get("type", "navigate_to_pose")

        if goal_type == "cancel":
            request.app.state.nav2_pending_goal = {
                "type": "cancel",
                "id": f"cancel_{datetime.now(timezone.utc).timestamp():.0f}",
            }
            return {"success": True, "message": "Cancel requested"}

        import uuid

        goal_id = str(uuid.uuid4())[:8]

        goal = {"id": goal_id, "type": goal_type}
        if goal_type == "navigate_to_pose":
            goal["pose"] = body.get("pose", {})
        elif goal_type == "navigate_through_poses":
            goal["poses"] = body.get("poses", [])
        elif goal_type == "follow_waypoints":
            goal["waypoints"] = body.get("waypoints", [])
        else:
            raise HTTPException(
                status_code=400, detail=f"Unknown goal type: {goal_type}"
            )

        request.app.state.nav2_pending_goal = goal
        request.app.state.nav2_current_status = {
            "status": "pending",
            "goal_id": goal_id,
        }
        return {"success": True, "goal_id": goal_id, "type": goal_type}

    @app.get("/api/nav2/pending", tags=["Nav2"])
    async def nav2_pending(request: Request):
        """
        Poll for pending navigation goal (called by nav2_goal_bridge inside container).
        Returns the goal and clears it so it's only dispatched once.
        """
        goal = request.app.state.nav2_pending_goal
        if goal:
            request.app.state.nav2_pending_goal = None
            return {"goal": goal}
        return {"goal": None}

    @app.post("/api/nav2/feedback", tags=["Nav2"])
    async def nav2_feedback(request: Request):
        """Receive navigation feedback from nav2_goal_bridge."""
        body = await _parse_request_json_object(request)
        request.app.state.nav2_current_status = body
        return {"success": True}

    @app.post("/api/nav2/result", tags=["Nav2"])
    async def nav2_result(request: Request):
        """Receive navigation result from nav2_goal_bridge."""
        body = await _parse_request_json_object(request)
        request.app.state.nav2_last_result = body
        request.app.state.nav2_current_status = {
            "status": body.get("status", "unknown"),
            "goal_id": body.get("goal_id"),
            "message": body.get("message"),
        }
        return {"success": True}

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
        Trigger autonomous spray sequence on a target (SP-001).

        Requires target_id, x, y, z coordinates. Drone must be > 2m
        from target in the plane parallel to the target.

        The sequence runs fully autonomously (SP-002):
        APPROACH -> AIM -> SPRAY -> VERIFY -> UPLOAD -> COMPLETE
        """
        spray_ctrl = getattr(request.app.state, "spray_controller", None)
        if not spray_ctrl:
            raise HTTPException(
                status_code=503, detail="Spray controller not initialized"
            )

        body = await _parse_request_json_object(request)
        from .spray_controller import SprayTarget

        target = SprayTarget(
            target_id=body.get("target_id", 0),
            x=body.get("x", 0.0),
            y=body.get("y", 0.0),
            z=body.get("z", 0.0),
            label=body.get("label", ""),
            confidence=body.get("confidence", 0.0),
            is_ground=body.get("is_ground", False),
        )

        result = spray_ctrl.trigger(target)
        if not result["success"]:
            raise HTTPException(status_code=400, detail=result.get("error"))
        return result

    @app.post("/api/spray/abort", tags=["Spray"])
    async def abort_spray(request: Request):
        """Abort the current spray sequence."""
        spray_ctrl = getattr(request.app.state, "spray_controller", None)
        if not spray_ctrl:
            raise HTTPException(
                status_code=503, detail="Spray controller not initialized"
            )
        return spray_ctrl.abort()

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

        Coordinates servo, VIO source, nvblox config, and obstacle avoidance.
        Drone must be hovering for modes that require nvblox restart.

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

        success = mavlink_svc.send_obstacle_distance(
            distances=distances,
            increment=body.get("increment", 5),
            min_distance=body.get("min_distance", 20),
            max_distance=body.get("max_distance", 2000),
            angle_offset=body.get("angle_offset", 0),
            frame=body.get("frame", 0),
        )
        return {"success": success}

    def _require_terminal_api_key() -> None:
        """Disable terminal command execution unless API key auth is configured."""
        if not _NOMAD_API_KEY:
            raise HTTPException(
                status_code=503,
                detail="Terminal command execution is disabled until NOMAD_API_KEY is configured",
            )

    # ==================== Terminal Endpoints ======================================

    @app.post(
        "/api/terminal/run", tags=["Terminal"], response_model=TerminalCommandResponse
    )
    async def execute_terminal_command(request: TerminalCommandRequest):
        """
        Execute a whitelisted shell command on the Jetson.

        Only commands in the whitelist are allowed. To see available commands,
        use GET /api/terminal/commands.

        Common uses:
        - System diagnostics
        - Network troubleshooting
        - Service management
        """
        _require_terminal_api_key()

        # Validate command_name is in whitelist
        if request.command_name not in COMMAND_WHITELIST:
            available = list(COMMAND_WHITELIST.keys())
            raise HTTPException(
                status_code=400,
                detail=f"Command '{request.command_name}' not allowed. Available: {available}",
            )

        command_str = COMMAND_WHITELIST[request.command_name]

        try:
            # For commands with pipes or redirects, use shell=True
            # (safe because the command itself is whitelisted)
            if "|" in command_str or ">" in command_str or "<" in command_str:
                result = subprocess.run(
                    command_str,
                    shell=True,
                    capture_output=True,
                    text=True,
                    timeout=request.timeout,
                )
            else:
                # For simple commands, use shell=False with list
                cmd_parts = shlex.split(command_str)
                result = subprocess.run(
                    cmd_parts,
                    shell=False,
                    capture_output=True,
                    text=True,
                    timeout=request.timeout,
                )

            return TerminalCommandResponse(
                success=result.returncode == 0,
                stdout=result.stdout,
                stderr=result.stderr,
                return_code=result.returncode,
                command_executed=command_str,
            )

        except subprocess.TimeoutExpired:
            return TerminalCommandResponse(
                success=False,
                stdout="",
                stderr=f"Command timed out after {request.timeout}s",
                return_code=-1,
                command_executed=command_str,
            )
        except Exception as e:
            return TerminalCommandResponse(
                success=False,
                stdout="",
                stderr=str(e),
                return_code=-1,
                command_executed=command_str,
            )

    @app.post(
        "/api/terminal/exec", tags=["Terminal"], response_model=TerminalCommandResponse
    )
    async def exec_terminal_command(request: TerminalExecRequest):
        """
        Execute an arbitrary shell command on the Jetson.

        Intended for the Mission Planner built-in terminal.
        Commands are executed via ``bash -c`` so pipes, redirects, and
        compound statements work as expected.

        Supports persistent working directory via the ``cwd`` field.
        The response includes the resolved ``cwd`` after execution so
        the client can track directory changes across commands.
        """
        _require_terminal_api_key()

        import os

        command_str = request.command.strip()
        if not command_str:
            raise HTTPException(status_code=400, detail="Empty command")

        # Resolve working directory
        work_dir = request.cwd if request.cwd else os.path.expanduser("~")
        if not os.path.isdir(work_dir):
            work_dir = os.path.expanduser("~")

        try:
            # Append pwd to capture the cwd after execution
            # This handles cd commands naturally since bash runs them in sequence
            wrapped_cmd = f'{command_str}\necho "__NOMAD_CWD__$(pwd)"'

            result = subprocess.run(
                ["bash", "-c", wrapped_cmd],
                capture_output=True,
                text=True,
                timeout=request.timeout,
                cwd=work_dir,
            )

            # Extract cwd from stdout
            stdout_lines = result.stdout.split("\n")
            new_cwd = work_dir
            clean_stdout_lines = []
            for line in stdout_lines:
                if line.startswith("__NOMAD_CWD__"):
                    new_cwd = line[len("__NOMAD_CWD__") :]
                else:
                    clean_stdout_lines.append(line)
            clean_stdout = "\n".join(clean_stdout_lines)

            return TerminalCommandResponse(
                success=result.returncode == 0,
                stdout=clean_stdout,
                stderr=result.stderr,
                return_code=result.returncode,
                command_executed=command_str,
                cwd=new_cwd,
            )

        except subprocess.TimeoutExpired:
            return TerminalCommandResponse(
                success=False,
                stdout="",
                stderr=f"Command timed out after {request.timeout}s",
                return_code=-1,
                command_executed=command_str,
                cwd=work_dir,
            )
        except Exception as e:
            return TerminalCommandResponse(
                success=False,
                stdout="",
                stderr=str(e),
                return_code=-1,
                command_executed=command_str,
                cwd=work_dir,
            )

    @app.get("/api/terminal/commands", tags=["Terminal"])
    async def list_terminal_commands():
        """
        List all available whitelisted terminal commands.

        Returns a dictionary mapping command names to their actual shell commands.
        """
        return {"commands": COMMAND_WHITELIST}

    @app.get("/api/terminal/logs", tags=["Terminal"])
    async def get_service_logs(
        service: str = Query("edge_core", description="Service name"),
        lines: int = Query(50, description="Number of lines"),
    ):
        """Get recent logs for a service."""
        _require_terminal_api_key()

        try:
            result = subprocess.run(
                ["journalctl", "-u", service, "-n", str(lines), "--no-pager"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            return {
                "service": service,
                "logs": result.stdout,
                "lines": lines,
            }
        except Exception as e:
            return {"error": str(e)}

    # ==================== Services Status Endpoint ====================

    @app.get("/api/services/status", tags=["System"])
    async def services_status(request: Request):
        """
        Get status of all NOMAD services.

        Returns status of:
        - mavlink-router: MAVLink routing to CubePilot
        - mediamtx: RTSP video server
        - novnc: Browser-based remote desktop service
        - edge_core: This API service (always running if you see this)
        - isaac_ros: Isaac ROS bridge status
        - vio: VIO pipeline status
        """
        services = {}

        # Check mavlink-router
        try:
            systemd_status = "inactive"
            systemd_running = False
            try:
                result = subprocess.run(
                    ["systemctl", "is-active", "mavlink-router"],
                    capture_output=True,
                    text=True,
                    timeout=2,
                )
                systemd_status = result.stdout.strip() or "inactive"
                systemd_running = result.returncode == 0
            except Exception:
                pass

            process_running = False
            try:
                process_result = subprocess.run(
                    ["pgrep", "-f", "mavlink-routerd"],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                process_running = process_result.returncode == 0
            except Exception:
                process_running = False

            cubepilot_present = os.path.exists("/dev/ttyACM0")
            if not cubepilot_present:
                mavlink_status = "no_cubepilot"
                mavlink_running = False
            else:
                mavlink_running = systemd_running or process_running
                mavlink_status = "active" if mavlink_running else systemd_status

            services["mavlink_router"] = {
                "status": mavlink_status,
                "running": mavlink_running,
                "cubepilot_present": cubepilot_present,
            }
        except Exception as e:
            services["mavlink_router"] = {"status": "error", "error": str(e)}

        # Check mediamtx
        try:
            result = subprocess.run(
                ["systemctl", "is-active", "mediamtx"],
                capture_output=True,
                text=True,
                timeout=2,
            )
            systemd_status = result.stdout.strip() or "inactive"
            systemd_running = result.returncode == 0

            process_running = False
            try:
                process_result = subprocess.run(
                    ["pgrep", "-f", "mediamtx"],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                process_running = process_result.returncode == 0
            except Exception:
                process_running = False

            mediamtx_running = systemd_running or process_running
            services["mediamtx"] = {
                "status": "active" if mediamtx_running else systemd_status,
                "running": mediamtx_running,
            }
        except Exception as e:
            services["mediamtx"] = {"status": "error", "error": str(e)}

        # Check noVNC (remote desktop web bridge)
        try:
            user_status = "inactive"
            user_running = False
            try:
                user_result = subprocess.run(
                    ["systemctl", "--user", "is-active", "novnc"],
                    capture_output=True,
                    text=True,
                    timeout=2,
                )
                user_status = user_result.stdout.strip() or "inactive"
                user_running = user_result.returncode == 0
            except Exception:
                pass

            system_status = "inactive"
            system_running = False
            try:
                system_result = subprocess.run(
                    ["systemctl", "is-active", "novnc"],
                    capture_output=True,
                    text=True,
                    timeout=2,
                )
                system_status = system_result.stdout.strip() or "inactive"
                system_running = system_result.returncode == 0
            except Exception:
                pass

            websockify_running = False
            x11vnc_running = False
            try:
                websockify_result = subprocess.run(
                    ["pgrep", "-f", "[w]ebsockify.*6080"],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                websockify_running = websockify_result.returncode == 0

                x11vnc_result = subprocess.run(
                    ["pgrep", "-f", "[x]11vnc.*-rfbport 5900"],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                x11vnc_running = x11vnc_result.returncode == 0
            except Exception:
                websockify_running = False
                x11vnc_running = False

            novnc_running = user_running or system_running or (
                websockify_running and x11vnc_running
            )
            novnc_status = (
                "active"
                if novnc_running
                else (
                    user_status
                    if user_status != "inactive"
                    else system_status
                )
            )

            services["novnc"] = {
                "status": novnc_status,
                "running": novnc_running,
                "url": "http://localhost:6080/vnc.html",
            }
        except Exception as e:
            services["novnc"] = {"status": "error", "error": str(e)}

        # Edge Core is always running (we're responding)
        services["edge_core"] = {
            "status": "active",
            "running": True,
        }

        runtime_state = _probe_isaac_runtime_state(force_refresh=True)
        container_running = runtime_state["container_running"]
        nvblox_running = runtime_state["nvblox_running"]
        bridge_running = runtime_state["bridge_running"]

        # Isaac ROS status
        isaac_bridge = request.app.state.isaac_bridge
        if isaac_bridge:
            services["isaac_ros"] = {
                "status": "active",
                "running": True,
                "container_running": container_running,
                "nvblox_running": nvblox_running,
                "bridge_running": bridge_running,
                **isaac_bridge.get_status(),
            }
        else:
            external_bridge_active = container_running and bridge_running
            services["isaac_ros"] = {
                "status": "active" if external_bridge_active else "not_initialized",
                "running": external_bridge_active,
                "container_running": container_running,
                "nvblox_running": nvblox_running,
                "bridge_running": bridge_running,
                "message": "Active via external ROS-HTTP bridge"
                if external_bridge_active
                else "Isaac ROS bridge not enabled",
            }

        # VIO status
        vio_snapshot = _get_vio_snapshot(include_trajectory=True)
        external_vio_state = vio_snapshot["external_vio_state"]
        vio_trajectory = vio_snapshot["vio_trajectory"] or []

        if external_vio_state:
            services["vio"] = {
                "status": "active",
                "running": True,
                "source": external_vio_state.get("source", "external"),
                "confidence": external_vio_state.get("confidence", 0),
                "trajectory_points": len(vio_trajectory),
            }
        else:
            services["vio"] = {
                "status": "not_initialized",
                "running": False,
                "trajectory_points": len(vio_trajectory),
            }

        # Target localization status (Task 1 target_localizer ROS2 node)
        with request.app.state.detection_state_lock:
            detection_enabled = bool(
                getattr(request.app.state, "detection_enabled", True)
            )
            detection_last_update = request.app.state.detection_last_update
            detection_current_count = len(request.app.state.detected_objects)
            detection_history_count = len(request.app.state.detection_history)
        age_seconds = (
            time.time() - detection_last_update if detection_last_update > 0 else None
        )
        detection_fresh = age_seconds is not None and age_seconds <= 3.0
        services["detections"] = {
            "status": "active" if detection_enabled else "inactive",
            "running": detection_enabled,
            "detection_enabled": detection_enabled,
            "fresh_stream": detection_fresh,
            "age_seconds": age_seconds,
            "current_count": detection_current_count,
            "history_count": detection_history_count,
            "message": ("Running" if detection_enabled else "Stopped")
            + (
                f" ({detection_current_count} current, {detection_history_count} history)"
                if detection_enabled
                else ""
            ),
        }

        # Isaac ROS container summary (from shared probe)
        services["isaac_ros_container"] = {
            "status": "running" if container_running else "not_running",
            "running": container_running,
        }

        return services

    # ==================== Streaming Endpoints ====================

    @app.get("/api/stream/info", tags=["Streaming"])
    async def stream_info():
        """Get RTSP stream information."""
        rtsp_base = os.environ.get("MEDIA_SERVER_URL", "rtsp://localhost:8554")

        return {
            "primary_stream": f"{rtsp_base}/zed",
            "secondary_stream": f"{rtsp_base}/gimbal",
            "format": "H.264/RTSP",
            "recommended_player": "VLC or FFplay",
        }

    # ==================== Isaac ROS Bridge Endpoints ====================

    @app.get("/api/isaac/status", tags=["Isaac ROS"])
    async def isaac_status(request: Request):
        """
        Get Isaac ROS bridge status.

        Returns information about the perception backend,
        VIO state, and exclusion map status.
        """
        runtime_state = _probe_isaac_runtime_state(force_refresh=True)
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

    @app.post("/api/isaac/start", tags=["Isaac ROS"])
    async def isaac_start():
        """
        Start Isaac ROS container and services.

        This runs the start_isaac_ros_auto.sh script which:
        1. Starts the Docker container
        2. Installs dependencies
        3. Launches ZED + Nvblox
        4. Starts the ROS-HTTP bridge
        """
        script_path = os.path.expanduser("~/NOMAD/scripts/run/start_isaac_ros_auto.sh")

        if not os.path.exists(script_path):
            return {
                "success": False,
                "error": f"Script not found: {script_path}",
            }

        startup_guard_window_s = 90.0
        last_start_ts = float(
            getattr(app.state, "isaac_startup_last_initiated", 0.0) or 0.0
        )
        if (
            last_start_ts > 0.0
            and (time.time() - last_start_ts) < startup_guard_window_s
        ):
            try:
                runtime_state = _probe_isaac_runtime_state(force_refresh=True)
                if (
                    runtime_state["container_running"]
                    and runtime_state["nvblox_running"]
                    and runtime_state["bridge_running"]
                ):
                    app.state.isaac_startup_last_initiated = 0.0
                else:
                    return {
                        "success": True,
                        "message": "Isaac ROS startup already in progress. Check status in 30-60 seconds.",
                        "startup_in_progress": True,
                    }
            except Exception:
                return {
                    "success": True,
                    "message": "Isaac ROS startup already in progress. Check status in 30-60 seconds.",
                    "startup_in_progress": True,
                }

        # Avoid duplicate startup attempts while stack is already running.
        try:
            container_running = False
            nvblox_running = False
            bridge_running = False

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
            container_running = bool(result.stdout.strip())

            if container_running:
                result = subprocess.run(
                    [
                        "docker",
                        "exec",
                        "nomad_isaac_ros",
                        "bash",
                        "-c",
                        "ps aux | grep -v grep | grep -c component_container 2>/dev/null || echo 0",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                nvblox_running = int(result.stdout.strip() or "0") > 0

                result = subprocess.run(
                    [
                        "docker",
                        "exec",
                        "nomad_isaac_ros",
                        "bash",
                        "-c",
                        "ps aux | grep -v grep | grep -c ros_http_bridge 2>/dev/null || echo 0",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                bridge_running = int(result.stdout.strip() or "0") > 0

            if container_running and nvblox_running and bridge_running:
                app.state.isaac_startup_last_initiated = 0.0
                return {
                    "success": True,
                    "message": "Isaac ROS stack already running. Skipping duplicate start.",
                    "already_running": True,
                }
        except Exception:
            # Fall through to normal startup path on probe failures.
            pass

        try:
            # Run in background
            process = subprocess.Popen(
                ["bash", script_path, "start"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            app.state.isaac_startup_last_initiated = time.time()

            # Don't wait for completion - it takes a while
            return {
                "success": True,
                "message": "Isaac ROS startup initiated. Check status in 30-60 seconds.",
                "pid": process.pid,
            }
        except Exception as e:
            return {
                "success": False,
                "error": str(e),
            }

    @app.post("/api/isaac/stop", tags=["Isaac ROS"])
    async def isaac_stop():
        """Stop Isaac ROS container and services."""
        script_path = os.path.expanduser("~/NOMAD/scripts/run/start_isaac_ros_auto.sh")

        try:
            result = subprocess.run(
                ["bash", script_path, "stop"],
                capture_output=True,
                text=True,
                timeout=30,
            )

            return {
                "success": result.returncode == 0,
                "stdout": result.stdout,
                "stderr": result.stderr,
            }
        except Exception as e:
            return {
                "success": False,
                "error": str(e),
            }

    @app.post("/api/isaac/launch-nvblox", tags=["Isaac ROS"])
    async def isaac_launch_nvblox(
        request: Request,
        enable_od: bool = Query(
            default=False,
            description="Enable ZED object detection (less stable on current stack)",
        ),
    ):
        """
        Launch nvblox + ROS-HTTP bridge inside a running container.

        Lightweight alternative to /api/isaac/start: does NOT install deps
        or rebuild packages.  Assumes the container is already running and
        packages are already built.  Kills any existing nvblox / bridge
        processes first, applies NOMAD config overlay, then launches both.
        """
        result = _launch_nvblox_bridge_with_od(enable_od=enable_od)
        if result.get("success"):
            request.app.state.detection_enabled = enable_od
            request.app.state.detection_last_update = 0.0
        return result

    @app.post("/api/isaac/stop-nvblox", tags=["Isaac ROS"])
    async def isaac_stop_nvblox():
        """
        Stop nvblox and ROS-HTTP bridge without stopping the container.
        """
        container = "nomad_isaac_ros"
        try:
            for proc in [
                "launch_nvblox_bridge\\.sh|launch_zed_nvblox\\.sh",
                "nomad_zed_nvblox\\.launch\\.py|zed_example\\.launch\\.py",
                "component_container",
                "ros_http_bridge",
                "target_localizer_node|target_localizer\\.launch\\.py",
            ]:
                subprocess.run(
                    ["docker", "exec", container, "pkill", "-f", proc],
                    capture_output=True,
                    timeout=5,
                )
            # Clean stale FastRTPS SHM locks so next launch succeeds
            subprocess.run(
                [
                    "docker",
                    "exec",
                    container,
                    "bash",
                    "-c",
                    "rm -f /dev/shm/fastrtps_* 2>/dev/null",
                ],
                capture_output=True,
                timeout=5,
            )
            return {"success": True, "message": "nvblox and bridge stopped"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    # ==================== Foxglove Bridge ====================
    # The foxglove_bridge ROS2 package exposes all ROS2 topics via WebSocket
    # so Foxglove Studio can visualize them. Default port 8765.
    # This runs independently of nvblox — start/stop without relaunching.

    def _strip_host_port(host_value: str) -> str:
        host_value = (host_value or "").strip()
        if not host_value:
            return ""
        if host_value.startswith("["):
            end = host_value.find("]")
            if end > 1:
                return host_value[1:end]
        if host_value.count(":") == 1:
            return host_value.rsplit(":", 1)[0]
        return host_value

    def _resolve_foxglove_host(request: Request) -> str:
        forwarded_host = (request.headers.get("x-forwarded-host") or "").split(",", 1)[
            0
        ]
        host = _strip_host_port(forwarded_host)
        if host:
            return host

        host_header = (request.headers.get("host") or "").split(",", 1)[0]
        host = _strip_host_port(host_header)
        if host:
            return host

        if request.url.hostname:
            return request.url.hostname

        for env_name in ("FOXGLOVE_WS_HOST", "TAILSCALE_IP", "NOMAD_API_HOST"):
            env_host = (os.environ.get(env_name) or "").strip()
            if env_host:
                return env_host

        return "127.0.0.1"

    def _foxglove_ws_url(request: Request, port: int) -> str:
        return f"ws://{_resolve_foxglove_host(request)}:{int(port)}"

    @app.post("/api/isaac/foxglove/start", tags=["Isaac ROS"])
    async def foxglove_start(
        request: Request,
        port: int = Query(
            default=8765, description="WebSocket port for Foxglove Studio"
        ),
    ):
        """
        Start Foxglove bridge inside the Isaac ROS container.

        Opens a WebSocket server that Foxglove Studio can connect to for
        visualizing ROS2 topics (pointclouds, meshes, TF, images, etc.).

        Connect Foxglove Studio to: ws://<jetson-ip>:<port>
        """
        container = "nomad_isaac_ros"
        try:
            # Check if already running
            probe = subprocess.run(
                ["docker", "exec", container, "bash", "-c", "pgrep -f foxglove_bridge"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if probe.returncode == 0:
                active_port = int(getattr(request.app.state, "foxglove_port", port))
                request.app.state.foxglove_port = active_port
                return {
                    "success": True,
                    "message": f"Foxglove bridge already running on port {active_port}",
                    "already_running": True,
                    "url": _foxglove_ws_url(request, active_port),
                }

            # Launch foxglove_bridge in background
            # Try ros2 launch first (installed package), fall back to ros2 run
            launch_cmd = f"""#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null

# Check if foxglove_bridge package is available
if ! ros2 pkg list 2>/dev/null | grep -q foxglove_bridge; then
    echo "Installing foxglove_bridge..."
    apt-get update -qq && apt-get install -y -qq ros-humble-foxglove-bridge 2>&1
fi

ros2 run foxglove_bridge foxglove_bridge --ros-args \\
    -p port:={port} \\
    -p address:=0.0.0.0 \\
    -p capabilities:='["clientPublish","connectionGraph","assets"]' \\
    -p send_buffer_limit:=10000000
"""
            subprocess.run(
                [
                    "docker",
                    "exec",
                    container,
                    "bash",
                    "-c",
                    f"cat > /tmp/start_foxglove.sh << 'EOFSCRIPT'\n{launch_cmd}\nEOFSCRIPT\nchmod +x /tmp/start_foxglove.sh",
                ],
                capture_output=True,
                text=True,
                timeout=10,
                check=True,
            )

            subprocess.run(
                [
                    "docker",
                    "exec",
                    "-d",
                    container,
                    "bash",
                    "-c",
                    "bash /tmp/start_foxglove.sh > /tmp/foxglove_bridge.log 2>&1",
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )

            # Wait briefly for startup
            time.sleep(3)
            verify = subprocess.run(
                ["docker", "exec", container, "bash", "-c", "pgrep -f foxglove_bridge"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if verify.returncode == 0:
                request.app.state.foxglove_enabled = True
                request.app.state.foxglove_port = int(port)
                return {
                    "success": True,
                    "message": f"Foxglove bridge started on port {port}",
                    "url": _foxglove_ws_url(request, int(port)),
                }
            else:
                log_tail = subprocess.run(
                    [
                        "docker",
                        "exec",
                        container,
                        "tail",
                        "-20",
                        "/tmp/foxglove_bridge.log",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                return {
                    "success": False,
                    "error": "Foxglove bridge failed to start",
                    "logs": (log_tail.stdout or log_tail.stderr or "")[-400:],
                }
        except Exception as e:
            return {"success": False, "error": str(e)}

    @app.post("/api/isaac/foxglove/stop", tags=["Isaac ROS"])
    async def foxglove_stop(request: Request):
        """Stop Foxglove bridge."""
        container = "nomad_isaac_ros"
        try:
            subprocess.run(
                ["docker", "exec", container, "pkill", "-f", "foxglove_bridge"],
                capture_output=True,
                timeout=5,
            )
            request.app.state.foxglove_enabled = False
            return {"success": True, "message": "Foxglove bridge stopped"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    @app.get("/api/isaac/foxglove/status", tags=["Isaac ROS"])
    async def foxglove_status(request: Request):
        """Check if Foxglove bridge is running."""
        container = "nomad_isaac_ros"
        try:
            probe = subprocess.run(
                ["docker", "exec", container, "bash", "-c", "pgrep -f foxglove_bridge"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            running = probe.returncode == 0
            port = int(getattr(request.app.state, "foxglove_port", 8765))
            return {
                "running": running,
                "url": _foxglove_ws_url(request, port) if running else None,
            }
        except Exception as e:
            return {"running": False, "error": str(e)}

    @app.get("/api/isaac/logs", tags=["Isaac ROS"])
    async def isaac_logs(
        log_type: str = Query(
            default="all", description="Log type: all, zed, bridge, foxglove"
        ),
    ):
        """Get Isaac ROS container logs."""
        try:
            if log_type == "zed":
                result = subprocess.run(
                    [
                        "docker",
                        "exec",
                        "nomad_isaac_ros",
                        "tail",
                        "-50",
                        "/tmp/zed_nvblox.log",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
            elif log_type == "bridge":
                result = subprocess.run(
                    [
                        "docker",
                        "exec",
                        "nomad_isaac_ros",
                        "tail",
                        "-50",
                        "/tmp/ros_bridge.log",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
            elif log_type == "foxglove":
                result = subprocess.run(
                    [
                        "docker",
                        "exec",
                        "nomad_isaac_ros",
                        "tail",
                        "-50",
                        "/tmp/foxglove_bridge.log",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
            else:
                zed_result = subprocess.run(
                    [
                        "docker",
                        "exec",
                        "nomad_isaac_ros",
                        "tail",
                        "-25",
                        "/tmp/zed_nvblox.log",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                bridge_result = subprocess.run(
                    [
                        "docker",
                        "exec",
                        "nomad_isaac_ros",
                        "tail",
                        "-25",
                        "/tmp/ros_bridge.log",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
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

    # ==================== Object Detection Endpoints ====================
    # ROS2 target detections via ZED custom OD pipeline.
    # Detections are received from ros_http_bridge and served to Mission Planner.

    @app.post("/api/detections/start", tags=["Detections"])
    async def start_detections(request: Request):
        """
        Start ROS2 target detection by relaunching nvblox with OD enabled.

        This keeps launch behavior consistent with NOMAD's custom launch file.
        """
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

    # ==================== Video Streaming Endpoints ====================
    # Isaac ROS H.264 video streaming with dynamic topic switching

    from .video_stream_manager import get_video_stream_manager

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

        topics = mgr.list_topics()
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

            # Canonical SLAM frame identifier is "odom" (REP-103). Anything else
            # from the bridge is a contract violation -- normalize and log once
            # so silent drift cannot corrupt Mission Planner visualization.
            incoming_frame = mesh_data.get("frame_id", "odom")
            if incoming_frame != "odom":
                if not getattr(request.app.state, "_mesh_ingest_frame_mismatch_logged", False):
                    logger.warning(
                        "mesh/update frame_id mismatch: got %r, expected 'odom'",
                        incoming_frame,
                    )
                    request.app.state._mesh_ingest_frame_mismatch_logged = True

            request.app.state.slam_mesh_data = {
                "mesh": mesh_data,
                "received_at": datetime.now(timezone.utc).isoformat(),
                "block_count": item_count,
                "total_blocks": total_items,
                "mode": mode,
                "frame_id": "odom",
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
    ):
        """
        Clear the current SLAM mesh and reset the map without a full relaunch when possible.

        Strategy:
        1) Try native nvblox clear service.
        2) Fallback to loading baseline map snapshot.
        3) Fallback to ZED tracking reset services.
        4) Relaunch only when explicitly requested via relaunch_if_needed=true.
        """
        nvblox_cleared = False
        nvblox_message = ""
        nvblox_error_status = 503
        clear_strategy = "none"
        clear_warnings: list[str] = []

        # Preferred path: native nvblox clear service (no restart).
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

        # Fallback path: load baseline map snapshot (no restart).
        if not nvblox_cleared:
            try:
                baseline_path = _resolve_nvblox_empty_map_path(empty_map_path)
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
                nvblox_cleared = True
                clear_strategy = "load_empty_map"
                nvblox_message = "; ".join(
                    part for part in [load_msg, reset_msg] if part
                ) or f"Loaded baseline map: {baseline_path}"
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
            return JSONResponse(status_code=nvblox_error_status, content=response_payload)
        return response_payload

    # ==================== Sensor Calibration Endpoints ====================

    @app.post("/api/calibration/zed/sensor-viewer/start", tags=["Calibration"])
    async def start_zed_sensor_viewer(request: Request):
        """
        Launch ZED Sensor Viewer on the Jetson desktop.

        This is intended for manual magnetometer calibration via noVNC.
        """
        viewer_bin = shutil.which("ZED_Sensor_Viewer") or shutil.which("zed_sensor_viewer")
        fallback_bin = shutil.which("ZED_Calibration")
        if not viewer_bin and not fallback_bin:
            raise HTTPException(
                status_code=500,
                detail="Neither ZED_Sensor_Viewer nor ZED_Calibration was found in PATH",
            )

        display = os.environ.get("NOMAD_CAL_DISPLAY") or os.environ.get("DISPLAY") or ":1"
        request_host = request.url.hostname or ""
        novnc_host = request_host
        if not novnc_host or novnc_host in {"localhost", "127.0.0.1"}:
            novnc_host = (
                os.environ.get("TAILSCALE_IP")
                or os.environ.get("JETSON_IP")
                or "localhost"
            )
        novnc_url = f"http://{novnc_host}:6080/vnc.html?autoconnect=0&reconnect=0&resize=scale"

        deps_lib_dir = os.path.expanduser(
            "~/NOMAD/.deps/zed_viewer/root/usr/lib/aarch64-linux-gnu"
        )
        run_env = os.environ.copy()
        run_env["DISPLAY"] = display
        if os.path.isdir(deps_lib_dir):
            existing_ld = run_env.get("LD_LIBRARY_PATH", "")
            run_env["LD_LIBRARY_PATH"] = (
                f"{deps_lib_dir}:{existing_ld}" if existing_ld else deps_lib_dir
            )

        def _missing_shared_libs(binary_path: str) -> list[str]:
            try:
                result = subprocess.run(
                    ["ldd", binary_path],
                    capture_output=True,
                    text=True,
                    timeout=5,
                    env=run_env,
                )
                output = "\n".join(
                    part for part in [result.stdout, result.stderr] if part
                )
                missing = []
                for line in output.splitlines():
                    if "=> not found" not in line:
                        continue
                    lib_name = line.split("=>", 1)[0].strip()
                    if lib_name:
                        missing.append(lib_name)
                return missing
            except Exception:
                return []

        def _launch_and_verify(binary_path: str, process_name: str) -> int:
            proc = subprocess.Popen(
                [binary_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=run_env,
                start_new_session=True,
            )
            time.sleep(0.8)
            if proc.poll() is not None:
                raise RuntimeError(f"{process_name} process exited immediately")
            return proc.pid

        try:
            if viewer_bin:
                running_probe = subprocess.run(
                    ["pgrep", "-f", "ZED_Sensor_Viewer|zed_sensor_viewer"],
                    capture_output=True,
                    text=True,
                    timeout=3,
                )
                if running_probe.returncode == 0:
                    return {
                        "success": True,
                        "message": "ZED Sensor Viewer already running",
                        "display": display,
                        "tool": os.path.basename(viewer_bin),
                        "novnc_url": novnc_url,
                    }

            if fallback_bin:
                fallback_running = subprocess.run(
                    ["pgrep", "-f", "ZED_Calibration"],
                    capture_output=True,
                    text=True,
                    timeout=3,
                )
                if fallback_running.returncode == 0:
                    return {
                        "success": True,
                        "message": "ZED_Calibration already running",
                        "display": display,
                        "tool": os.path.basename(fallback_bin),
                        "fallback": True,
                        "novnc_url": novnc_url,
                    }
        except Exception:
            # Continue with launch attempt even if probe fails.
            pass

        viewer_missing_deps: list[str] = []
        if viewer_bin:
            viewer_missing_deps = _missing_shared_libs(viewer_bin)
            if not viewer_missing_deps:
                try:
                    pid = _launch_and_verify(viewer_bin, "ZED Sensor Viewer")
                    return {
                        "success": True,
                        "message": "ZED Sensor Viewer launched",
                        "display": display,
                        "tool": os.path.basename(viewer_bin),
                        "pid": pid,
                        "novnc_url": novnc_url,
                    }
                except Exception as e:
                    logger.warning(f"Sensor Viewer launch failed, trying fallback: {e}")
            else:
                logger.warning(
                    "Sensor Viewer missing shared libs: "
                    + ", ".join(viewer_missing_deps)
                )

        if fallback_bin:
            fallback_missing = _missing_shared_libs(fallback_bin)
            if fallback_missing:
                raise HTTPException(
                    status_code=500,
                    detail=(
                        "ZED_Calibration is missing shared libraries: "
                        + ", ".join(fallback_missing)
                    ),
                )

            try:
                pid = _launch_and_verify(fallback_bin, "ZED_Calibration")
                message = "ZED_Calibration launched"
                if viewer_missing_deps:
                    message = (
                        "ZED Sensor Viewer dependencies missing; "
                        "launched ZED_Calibration fallback"
                    )
                return {
                    "success": True,
                    "message": message,
                    "display": display,
                    "tool": os.path.basename(fallback_bin),
                    "fallback": True,
                    "missing_dependencies": viewer_missing_deps,
                    "pid": pid,
                    "novnc_url": novnc_url,
                }
            except Exception as e:
                logger.error(f"Failed to launch ZED_Calibration fallback: {e}")
                raise HTTPException(
                    status_code=500,
                    detail=(
                        "Failed to launch ZED calibration tools. "
                        + (f"Sensor Viewer missing deps: {', '.join(viewer_missing_deps)}. " if viewer_missing_deps else "")
                        + f"Fallback error: {e}"
                    ),
                )

        raise HTTPException(
            status_code=500,
            detail=(
                "ZED Sensor Viewer could not be launched"
                + (
                    f" (missing dependencies: {', '.join(viewer_missing_deps)})"
                    if viewer_missing_deps
                    else ""
                )
            ),
        )

    @app.post("/api/tools/rviz2/start", tags=["Tools"])
    async def start_rviz2(request: Request):
        """
        Launch RViz2 inside the Isaac ROS container and show it via noVNC.
        """
        display = os.environ.get("NOMAD_CAL_DISPLAY") or os.environ.get("DISPLAY") or ":1"
        rviz_config_path = os.environ.get(
            "NOMAD_RVIZ_CONFIG_PATH",
            "/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/visualization/zed_example.rviz",
        )
        request_host = request.url.hostname or ""
        novnc_host = request_host
        if not novnc_host or novnc_host in {"localhost", "127.0.0.1"}:
            novnc_host = (
                os.environ.get("TAILSCALE_IP")
                or os.environ.get("JETSON_IP")
                or "localhost"
            )
        novnc_url = f"http://{novnc_host}:6080/vnc.html?autoconnect=0&reconnect=0&resize=scale"

        run_env = os.environ.copy()
        run_env["DISPLAY"] = display
        run_env.setdefault("QT_X11_NO_MITSHM", "1")
        run_env.setdefault("XDG_RUNTIME_DIR", "/tmp")

        start_novnc_cmd = COMMAND_WHITELIST.get("start_novnc")
        if start_novnc_cmd:
            novnc_result = subprocess.run(
                ["bash", "-lc", start_novnc_cmd],
                capture_output=True,
                text=True,
                timeout=25,
                env=run_env,
            )
            if novnc_result.returncode != 0:
                novnc_output = "\n".join(
                    part for part in [novnc_result.stdout, novnc_result.stderr] if part
                ).strip()
                raise HTTPException(
                    status_code=500,
                    detail=(
                        "Failed to start noVNC desktop stack before launching RViz2. "
                        + (novnc_output or "No additional output available")
                    ),
                )

        try:
            running_probe = subprocess.run(
                ["pgrep", "-x", "rviz2"],
                capture_output=True,
                text=True,
                timeout=3,
            )
            if running_probe.returncode == 0:
                return {
                    "success": True,
                    "message": "RViz2 already running",
                    "display": display,
                    "tool": "rviz2",
                    "novnc_url": novnc_url,
                }
        except Exception:
            # Continue with launch attempt even if process probe fails.
            pass

        runtime_state = _probe_isaac_runtime_state(force_refresh=True)
        if not runtime_state.get("container_running", False):
            raise HTTPException(
                status_code=503,
                detail=(
                    "Isaac ROS container is not running. Start Task 2 / Isaac ROS "
                    "before launching RViz2."
                ),
            )

        container = "nomad_isaac_ros"

        config_probe = subprocess.run(
            [
                "docker",
                "exec",
                container,
                "bash",
                "-lc",
                f"test -f {shlex.quote(rviz_config_path)}",
            ],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if config_probe.returncode != 0:
            raise HTTPException(
                status_code=500,
                detail=(
                    "RViz2 config file not found in Isaac ROS container: "
                    f"{rviz_config_path}"
                ),
            )

        # Compatibility fix for Humble images where PointCloudBox requests
        # rviz/glsl150/box.geom, which may be missing in the container runtime.
        patch_cmd = """
set -e
src=/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_rviz_plugin/src/nvblox_plugin_visual.cpp
if [ ! -f "$src" ]; then
  echo __NVBLOX_PLUGIN_SRC_MISSING__
  exit 0
fi
if grep -q '"rviz/PointCloudBox"' "$src"; then
  sed -i 's/"rviz\\/PointCloudBox"/"rviz\\/PointCloudPoint"/g' "$src"
  source /opt/ros/humble/setup.bash
  cd /workspaces/isaac_ros-dev
  colcon build --packages-select nvblox_rviz_plugin > /tmp/nomad_rviz_patch_build.log 2>&1
  echo __NVBLOX_PATCH_APPLIED__
else
  echo __NVBLOX_PATCH_ALREADY_PRESENT__
fi
"""
        patch_result = subprocess.run(
            ["docker", "exec", "-u", "root", container, "bash", "-lc", patch_cmd],
            capture_output=True,
            text=True,
            timeout=90,
        )
        patch_output = "\n".join(
            part for part in [patch_result.stdout, patch_result.stderr] if part
        ).strip()
        if patch_result.returncode != 0:
            raise HTTPException(
                status_code=500,
                detail=(
                    "Failed to apply RViz2 nvblox compatibility patch before launch. "
                    + (patch_output or "No patch output available")
                ),
            )

        try:
            running_probe = subprocess.run(
                ["docker", "exec", container, "pgrep", "-x", "rviz2"],
                capture_output=True,
                text=True,
                timeout=3,
            )
            if running_probe.returncode == 0:
                return {
                    "success": True,
                    "message": "RViz2 already running",
                    "display": display,
                    "tool": "rviz2",
                    "novnc_url": novnc_url,
                }
        except Exception:
            # Continue with launch attempt even if process probe fails.
            pass

        display_arg = shlex.quote(display)
        rviz_config_arg = shlex.quote(rviz_config_path)
        launch_cmd = f"""
export DISPLAY={display_arg}
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null || true
if ! command -v rviz2 >/dev/null 2>&1; then
    echo __RVIZ2_NOT_FOUND__
    exit 127
fi
if [ ! -f {rviz_config_arg} ]; then
        echo __RVIZ_CONFIG_NOT_FOUND__
        exit 66
fi
nohup rviz2 -d {rviz_config_arg} > /tmp/rviz2.log 2>&1 &
echo $!
"""

        try:
            launch_result = subprocess.run(
                ["docker", "exec", container, "bash", "-lc", launch_cmd],
                capture_output=True,
                text=True,
                timeout=20,
            )

            launch_output = "\n".join(
                part for part in [launch_result.stdout, launch_result.stderr] if part
            ).strip()

            if launch_result.returncode != 0:
                if "__RVIZ2_NOT_FOUND__" in launch_output or launch_result.returncode == 127:
                    raise HTTPException(
                        status_code=500,
                        detail=(
                            "rviz2 is not available in the current ROS environment. "
                            "Install rviz2 on the Jetson or source the correct ROS setup."
                        ),
                    )
                if "__RVIZ_CONFIG_NOT_FOUND__" in launch_output:
                    raise HTTPException(
                        status_code=500,
                        detail=(
                            "RViz2 config file not found in Isaac ROS container: "
                            f"{rviz_config_path}"
                        ),
                    )
                raise HTTPException(
                    status_code=500,
                    detail=(
                        "Failed to start RViz2 via ROS environment. "
                        + (launch_output or "No launch output available")
                    ),
                )

            pid = None
            for line in reversed((launch_result.stdout or "").splitlines()):
                token = line.strip()
                if token.isdigit():
                    pid = int(token)
                    break

            if pid is None:
                raise HTTPException(
                    status_code=500,
                    detail=(
                        "RViz2 launch command did not return a process id. "
                        + (launch_output or "No launch output available")
                    ),
                )

            time.sleep(1.0)
            alive_check = subprocess.run(
                ["docker", "exec", container, "bash", "-lc", f"kill -0 {pid} 2>/dev/null"],
                capture_output=True,
                text=True,
                timeout=3,
            )
            if alive_check.returncode != 0:
                log_tail = subprocess.run(
                    ["docker", "exec", container, "bash", "-lc", "tail -40 /tmp/rviz2.log 2>/dev/null || true"],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                tail_text = (log_tail.stdout or "").strip()
                raise HTTPException(
                    status_code=500,
                    detail=(
                        "RViz2 exited immediately after launch. "
                        + (
                            f"Recent log output:\n{tail_text}"
                            if tail_text
                            else "No RViz2 log output found"
                        )
                    ),
                )

            return {
                "success": True,
                "message": "RViz2 launched",
                "display": display,
                "tool": "rviz2",
                "pid": pid,
                "novnc_url": novnc_url,
            }
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=(
                    "Failed to launch RViz2. "
                    f"Error: {e}"
                ),
            )

    @app.post("/api/tools/rviz2/stop", tags=["Tools"])
    async def stop_rviz2():
        """
        Stop RViz2 processes to reduce CPU/GPU load.

        Attempts to stop RViz2 inside the Isaac ROS container and on the host.
        """
        container = "nomad_isaac_ros"
        runtime_state = _probe_isaac_runtime_state(force_refresh=True)

        container_status = "not_running"
        container_output = ""
        if runtime_state.get("container_running", False):
            try:
                container_stop = subprocess.run(
                    [
                        "docker",
                        "exec",
                        "-u",
                        "root",
                        container,
                        "bash",
                        "-lc",
                        "pkill -f '[r]viz2' >/dev/null 2>&1 || true; "
                        "if pgrep -f '[r]viz2' >/dev/null 2>&1; then echo running; else echo stopped; fi",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=8,
                )
                container_output = "\n".join(
                    part
                    for part in [container_stop.stdout, container_stop.stderr]
                    if part
                ).strip()
                container_status = (
                    "stopped"
                    if "stopped" in (container_output or "")
                    else "running"
                )
            except Exception as e:
                container_status = "error"
                container_output = str(e)

        host_stop = subprocess.run(
            [
                "bash",
                "-lc",
                "pkill -f '[r]viz2' >/dev/null 2>&1 || true; "
                "if pgrep -f '[r]viz2' >/dev/null 2>&1; then echo running; else echo stopped; fi",
            ],
            capture_output=True,
            text=True,
            timeout=8,
        )
        host_output = "\n".join(
            part for part in [host_stop.stdout, host_stop.stderr] if part
        ).strip()
        host_status = "stopped" if "stopped" in (host_output or "") else "running"

        success = host_status == "stopped" and container_status in {
            "stopped",
            "not_running",
        }

        return {
            "success": success,
            "message": (
                "RViz2 stopped"
                if success
                else "RViz2 stop requested but one or more processes are still running"
            ),
            "host_status": host_status,
            "container_status": container_status,
            "container_output": container_output,
        }

    # ==================== Admin Endpoints ====================

    @app.post("/api/admin/git-update", tags=["Admin"])
    async def git_update():
        """
        Update the NOMAD codebase from Git.

        Performs:
        1. git stash (save any local changes)
        2. git pull origin main
        3. chmod +x on all .sh scripts (recursive)

        Returns the output of each command.
        """
        _require_terminal_api_key()

        results = {
            "success": True,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "steps": [],
        }
        errors: list[str] = []

        nomad_dir = os.path.expanduser("~/NOMAD")

        try:
            # Step 1: git stash
            stash_result = subprocess.run(
                ["git", "stash"],
                cwd=nomad_dir,
                capture_output=True,
                text=True,
                timeout=30,
            )
            stash_output_text = "\n".join(
                part for part in [stash_result.stdout, stash_result.stderr] if part
            ).lower()
            stash_created = (
                stash_result.returncode == 0
                and "no local changes to save" not in stash_output_text
            )
            results["steps"].append(
                {
                    "step": "git stash",
                    "success": stash_result.returncode == 0,
                    "output": stash_result.stdout.strip(),
                    "error": stash_result.stderr.strip()
                    if stash_result.returncode != 0
                    else None,
                }
            )

            # Step 2: git pull
            pull_result = subprocess.run(
                ["git", "pull", "origin", "main"],
                cwd=nomad_dir,
                capture_output=True,
                text=True,
                timeout=60,
            )
            results["steps"].append(
                {
                    "step": "git pull origin main",
                    "success": pull_result.returncode == 0,
                    "output": pull_result.stdout.strip(),
                    "error": pull_result.stderr.strip()
                    if pull_result.returncode != 0
                    else None,
                }
            )

            if pull_result.returncode != 0:
                errors.append("Git pull failed")
            else:
                # Step 3: chmod +x scripts
                chmod_result = subprocess.run(
                    ["bash", "-c", "find scripts -name '*.sh' -exec chmod +x {} +"],
                    cwd=nomad_dir,
                    capture_output=True,
                    text=True,
                    timeout=10,
                )
                results["steps"].append(
                    {
                        "step": "chmod +x scripts/**/*.sh",
                        "success": chmod_result.returncode == 0,
                        "output": "Scripts made executable"
                        if chmod_result.returncode == 0
                        else chmod_result.stdout.strip(),
                        "error": chmod_result.stderr.strip()
                        if chmod_result.returncode != 0
                        else None,
                    }
                )
                if chmod_result.returncode != 0:
                    chmod_error = (
                        chmod_result.stderr.strip()
                        or chmod_result.stdout.strip()
                        or "unknown error"
                    )
                    errors.append(f"chmod +x scripts/**/*.sh failed: {chmod_error}")

            if stash_created:
                stash_pop_result = subprocess.run(
                    ["git", "stash", "pop"],
                    cwd=nomad_dir,
                    capture_output=True,
                    text=True,
                    timeout=30,
                )
                results["steps"].append(
                    {
                        "step": "git stash pop",
                        "success": stash_pop_result.returncode == 0,
                        "output": stash_pop_result.stdout.strip(),
                        "error": stash_pop_result.stderr.strip()
                        if stash_pop_result.returncode != 0
                        else None,
                    }
                )
                if stash_pop_result.returncode != 0:
                    stash_error = (
                        stash_pop_result.stderr.strip()
                        or stash_pop_result.stdout.strip()
                        or "unknown error"
                    )
                    errors.append(f"Git stash pop failed: {stash_error}")

            if errors:
                results["success"] = False
                results["error"] = "; ".join(errors)

            return results

        except subprocess.TimeoutExpired:
            results["success"] = False
            results["error"] = "Command timed out"
            return results
        except Exception as e:
            results["success"] = False
            results["error"] = str(e)
            return results

    @app.get("/api/admin/git-status", tags=["Admin"])
    async def git_status():
        """
        Get current Git status and branch info.

        Returns current branch, commit hash, and any uncommitted changes.
        """
        _require_terminal_api_key()

        nomad_dir = os.path.expanduser("~/NOMAD")

        try:
            # Get current branch
            branch_result = subprocess.run(
                ["git", "branch", "--show-current"],
                cwd=nomad_dir,
                capture_output=True,
                text=True,
                timeout=5,
            )

            # Get current commit
            commit_result = subprocess.run(
                ["git", "log", "--oneline", "-1"],
                cwd=nomad_dir,
                capture_output=True,
                text=True,
                timeout=5,
            )

            # Get status
            status_result = subprocess.run(
                ["git", "status", "--porcelain"],
                cwd=nomad_dir,
                capture_output=True,
                text=True,
                timeout=5,
            )

            return {
                "branch": branch_result.stdout.strip(),
                "commit": commit_result.stdout.strip(),
                "has_changes": len(status_result.stdout.strip()) > 0,
                "changes": status_result.stdout.strip().split("\n")
                if status_result.stdout.strip()
                else [],
                "timestamp": datetime.now(timezone.utc).isoformat(),
            }

        except Exception as e:
            return {
                "error": str(e),
                "branch": "unknown",
                "commit": "unknown",
                "has_changes": False,
            }

    @app.post("/api/admin/upload-gdrive-token", tags=["Admin"])
    async def upload_gdrive_token(request: Request):
        """
        Receive Google Drive OAuth2 token JSON from Mission Planner
        and save to ~/.nomad/gdrive_token.json on the Jetson.

        The token is generated via the one-time OAuth2 setup flow
        (python edge_core/gdrive_upload.py --setup <client_secret.json>)
        and contains a refresh_token for headless use.
        """
        _require_terminal_api_key()

        body = await request.body()
        if not body:
            raise HTTPException(status_code=400, detail="Empty request body")

        try:
            token_data = json.loads(body)
        except json.JSONDecodeError:
            raise HTTPException(status_code=400, detail="Invalid JSON")

        if "token" not in token_data and "refresh_token" not in token_data:
            raise HTTPException(
                status_code=400,
                detail="Missing required token fields (token or refresh_token)",
            )

        token_dir = os.path.expanduser("~/.nomad")
        os.makedirs(token_dir, exist_ok=True)
        token_path = os.path.join(token_dir, "gdrive_token.json")

        try:
            with open(token_path, "w") as f:
                json.dump(token_data, f, indent=2)
            os.chmod(token_path, 0o600)
            logger.info(f"Google Drive token saved to {token_path}")
            return {
                "success": True,
                "path": token_path,
            }
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to save token: {e}")

    # ==================== Servo Control Endpoints ====================
    # Control camera tilt servo and water shooter via PWM

    @app.get("/api/servo/status", tags=["Servo"])
    async def get_servo_status():
        """
        Get status of all servos.

        Returns current angle and enabled state for each servo.
        """
        try:
            from .servo_controller import get_servo_controller

            controller = get_servo_controller()
            if not controller or not controller.is_available():
                return {
                    "available": False,
                    "error": "Servo controller not initialized",
                    "servos": {},
                }

            status = controller.get_status()
            status["available"] = True
            return status

        except ImportError:
            return {
                "available": False,
                "error": "Servo controller module not available",
                "servos": {},
            }
        except Exception as e:
            logger.error(f"Servo status error: {e}")
            return {"available": False, "error": str(e), "servos": {}}

    @app.post("/api/servo/camera/tilt", tags=["Servo"])
    async def set_camera_tilt(
        angle: float = Query(..., ge=0, le=180, description="Tilt angle 0-180 degrees"),
    ):
        """
        Set camera tilt servo angle.

        Args:
            angle: Target angle in degrees
                - 0 = Looking down
                - 90 = Level (straight ahead)
                - 180 = Looking up

        Returns:
            Success status and new angle
        """
        try:
            from .servo_controller import get_servo_controller

            controller = get_servo_controller()
            if not controller or not controller.is_available():
                raise HTTPException(
                    status_code=503, detail="Servo controller not available"
                )

            success = controller.set_camera_tilt(angle)

            if success:
                return {
                    "status": "ok",
                    "angle": angle,
                    "message": f"Camera tilt set to {angle} degrees",
                }
            else:
                raise HTTPException(status_code=500, detail="Failed to set camera tilt")

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Camera tilt error: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @app.get("/api/servo/camera/tilt", tags=["Servo"])
    async def get_camera_tilt():
        """
        Get current camera tilt angle.

        Returns:
            Current angle in degrees
        """
        try:
            from .servo_controller import get_servo_controller

            controller = get_servo_controller()
            if not controller or not controller.is_available():
                return {"angle": None, "available": False}

            angle = controller.get_camera_tilt()
            return {"angle": angle, "available": angle is not None}

        except Exception as e:
            logger.error(f"Get camera tilt error: {e}")
            return {"angle": None, "available": False, "error": str(e)}

    @app.post("/api/servo/shooter/trigger", tags=["Servo"])
    async def trigger_water_shooter(
        duration_ms: int = Query(
            200, ge=50, le=2000, description="Trigger duration in milliseconds"
        ),
    ):
        """
        Trigger water shooter servo.

        Args:
            duration_ms: How long to activate shooter (50-2000ms)

        Returns:
            Success status
        """
        try:
            from .servo_controller import get_servo_controller

            controller = get_servo_controller()
            if not controller or not controller.is_available():
                raise HTTPException(
                    status_code=503, detail="Servo controller not available"
                )

            success = controller.trigger_water_shooter(duration_ms)

            if success:
                return {
                    "status": "ok",
                    "duration_ms": duration_ms,
                    "message": f"Water shooter triggered for {duration_ms}ms",
                }
            else:
                raise HTTPException(
                    status_code=500, detail="Failed to trigger water shooter"
                )

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Water shooter error: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @app.post("/api/servo/enable", tags=["Servo"])
    async def enable_servos():
        """Enable all servo PWM outputs."""
        try:
            from .servo_controller import get_servo_controller

            controller = get_servo_controller()
            if not controller:
                raise HTTPException(
                    status_code=503, detail="Servo controller not available"
                )

            controller.enable_all()
            return {"status": "ok", "message": "All servos enabled"}

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Enable servos error: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @app.post("/api/servo/disable", tags=["Servo"])
    async def disable_servos():
        """Disable all servo PWM outputs (safety)."""
        try:
            from .servo_controller import get_servo_controller

            controller = get_servo_controller()
            if not controller:
                raise HTTPException(
                    status_code=503, detail="Servo controller not available"
                )

            controller.disable_all()
            return {"status": "ok", "message": "All servos disabled"}

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Disable servos error: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    # ── RC Servo Bridge ───────────────────────────────────────────────

    @app.get("/api/servo/rc/status", tags=["Servo"])
    async def get_rc_servo_status():
        """
        Get RC-to-servo bridge status.

        Shows which RC channel is mapped to the nozzle servo,
        the last received RC value, and the last commanded angle.
        """
        try:
            from .rc_servo_bridge import get_rc_servo_bridge

            bridge = get_rc_servo_bridge()
            if bridge is None:
                return {"active": False, "error": "RC servo bridge not initialized"}

            return bridge.get_status()

        except ImportError:
            return {"active": False, "error": "RC servo bridge module not available"}
        except Exception as e:
            logger.error(f"RC servo status error: {e}")
            return {"active": False, "error": str(e)}

    @app.post("/api/servo/rc/channel", tags=["Servo"])
    async def set_rc_servo_channel(
        channel: int = Query(..., ge=1, le=18, description="RC channel number (1-18)"),
    ):
        """
        Change which RC channel controls the servo (runtime).

        Args:
            channel: RC channel number (1-18). Common choices:
                - Channel 6: Knob/potentiometer
                - Channel 7: 3-position switch
                - Channel 8: Slider
        """
        try:
            from .rc_servo_bridge import get_rc_servo_bridge

            bridge = get_rc_servo_bridge()
            if bridge is None:
                raise HTTPException(
                    status_code=503, detail="RC servo bridge not initialized"
                )

            bridge.set_channel(channel)
            return {
                "status": "ok",
                "rc_channel": channel,
                "message": f"RC servo bridge now using channel {channel}",
            }

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Set RC channel error: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    return app
