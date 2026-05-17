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
from .api_runtime import EdgeApiRuntime

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

    # API key authentication middleware.
    # If NOMAD_API_KEY is set, require X-API-Key on non-exempt endpoints.
    # If NOMAD_API_KEY is not set, allow loopback-only development traffic and
    # reject remote clients unless NOMAD_ALLOW_INSECURE_REMOTE=true is explicit.
    _NOMAD_API_KEY = (os.environ.get("NOMAD_API_KEY") or "").strip() or None
    _ALLOW_INSECURE_REMOTE = (
        (os.environ.get("NOMAD_ALLOW_INSECURE_REMOTE") or "").strip().lower()
        in {"1", "true", "yes", "on"}
    )
    _AUTH_EXEMPT_PATHS = {"/", "/health", "/docs", "/redoc", "/openapi.json"}
    _INTERNAL_BRIDGE_TOKEN_HEADER = "X-NOMAD-Internal-Token"
    _INTERNAL_BRIDGE_TOKEN = (
        os.environ.get("NOMAD_INTERNAL_TOKEN") or ""
    ).strip() or None
    _INTERNAL_BRIDGE_ALLOWED_ROUTES: set[tuple[str, str]] = {
        ("POST", "/api/vio/update"),
        ("POST", "/api/task/2/slam/mesh/update"),
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

    if _NOMAD_API_KEY is None:
        logger.warning(
            "NOMAD_API_KEY is not configured; only loopback clients are allowed "
            "unless NOMAD_ALLOW_INSECURE_REMOTE=true"
        )
    if _NOMAD_API_KEY is not None and _INTERNAL_BRIDGE_TOKEN is None:
        logger.warning(
            "NOMAD_INTERNAL_TOKEN is not configured; internal bridge bypass disabled"
        )

    def _require_admin_api_key() -> None:
        """Require a configured API key for high-risk terminal/admin routes."""
        if _NOMAD_API_KEY is None:
            raise HTTPException(
                status_code=403,
                detail="Admin/terminal routes require NOMAD_API_KEY to be configured",
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
                if _ALLOW_INSECURE_REMOTE or _is_loopback_client(request):
                    return await call_next(request)
                return JSONResponse(
                    status_code=401,
                    content={
                        "detail": (
                            "NOMAD_API_KEY is not configured; remote API access is disabled"
                        )
                    },
                )
            request_path = request.url.path.rstrip("/") or "/"
            if request_path in _AUTH_EXEMPT_PATHS:
                return await call_next(request)
            if _is_internal_bridge_request(request, request_path):
                return await call_next(request)
            provided_key = request.headers.get("X-API-Key")
            if not provided_key or not hmac.compare_digest(provided_key, _NOMAD_API_KEY):
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
    app.state.vio_trajectory: list[dict] = []  # List of {x, y, z, timestamp} points
    app.state.vio_trajectory_max_points: int = 1000  # Keep last N points
    app.state.vio_state_lock = threading.Lock()
    # Dedicated lock for SLAM mesh state (large 30MB+ updates) so writers don't
    # race on slam_mesh_version / slam_mesh_data when multiple POSTs land.
    app.state.slam_mesh_lock = threading.Lock()
    app.state.slam_mesh_delta_history: list[dict] = []
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
    app.state.detection_enabled: bool = os.environ.get(
        "NOMAD_DETECTIONS_AUTO_START", "false"
    ).strip().lower() in ("1", "true", "yes", "on")

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
    runtime = EdgeApiRuntime(app, state_manager, logger)

    ctx = ApiRouteContext(
        logger=logger,
        apply_vio_update_from_request=runtime.apply_vio_update_from_request,
        get_vio_snapshot=runtime.get_vio_snapshot,
        dispatch_nav_velocity=runtime.dispatch_nav_velocity,
        apply_detections_update=runtime.apply_detections_update,
        nomad_api_key=_NOMAD_API_KEY,
        allow_insecure_remote=_ALLOW_INSECURE_REMOTE,
        require_terminal_api_key=_require_admin_api_key,
    )

    @app.on_event("startup")
    async def _startup_high_rate_zmq_listener() -> None:
        """Start high-rate ZMQ listener on API startup."""
        runtime.start_high_rate_zmq_listener()

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
        runtime.stop_high_rate_zmq_listener()

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
                    gcs_extra_ips = os.environ.get("GCS_EXTRA_IPS", "")
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
                        endpoint_ips: list[str] = []
                        for ip in [gcs_ip, *gcs_extra_ips.replace(",", " ").split()]:
                            ip = ip.strip()
                            if ip and ip not in endpoint_ips:
                                endpoint_ips.append(ip)
                        endpoints: list[str] = []
                        for ip in endpoint_ips:
                            endpoints.extend(["-e", f"{ip}:{gcs_port_lte}"])
                        endpoints.extend(["-e", f"127.0.0.1:{gcs_port_local}"])

                        with open(log_path, "a") as lf:
                            subprocess.Popen(
                                [
                                    "mavlink-routerd",
                                    *endpoints,
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
            nvblox_node_probe = _docker_exec_bash_success(
                "nomad_isaac_ros",
                "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
                "source /workspaces/isaac_ros-dev/install/setup.bash >/dev/null 2>&1; "
                "ROS2CLI_DISABLE_DAEMON=1 ros2 node list 2>/dev/null | "
                "grep -Eq '(^|/)nvblox(_node)?$|(^|/)nvblox_node$'",
                timeout_s=6,
            )
            nvblox_marker_pub_probe = _docker_exec_bash_success(
                "nomad_isaac_ros",
                "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
                "source /workspaces/isaac_ros-dev/install/setup.bash >/dev/null 2>&1; "
                "ROS2CLI_DISABLE_DAEMON=1 ros2 topic info /nvblox_node/color_layer_marker 2>/dev/null | "
                "grep -Eq 'Publisher count: [1-9]'",
                timeout_s=6,
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

            if (
                nvblox_node_probe is None
                and nvblox_marker_pub_probe is None
                and cache_age_s < cache_max_stale_s
            ):
                nvblox_running = bool(cache.get("nvblox_running", False))
            else:
                nvblox_running = bool(nvblox_node_probe) or bool(nvblox_marker_pub_probe)

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

    from .api_routes.system import register_system_routes
    register_system_routes(app, ctx)

    from .api_routes.task1 import register_task1_routes
    register_task1_routes(app, ctx)

    from .api_routes.task2_ops import register_task2_routes
    register_task2_routes(app, ctx)

    from .api_routes.terminal import register_terminal_routes
    register_terminal_routes(app, ctx)

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

    from .api_routes.calibration_admin_servo import register_calibration_admin_servo_routes
    register_calibration_admin_servo_routes(app, ctx)

    return app
