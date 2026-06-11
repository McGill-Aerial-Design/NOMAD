# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Edge Core - REST API

Provides HTTP endpoints for system status, health monitoring,
payload/actuation control, and video streaming.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

import asyncio
import hmac
import ipaddress
import logging
import os
import subprocess
import time
from typing import TYPE_CHECKING, Any

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

from .api_context import ApiRouteContext
from .services.state import StateManager

if TYPE_CHECKING:
    from .modules.slam.isaac import IsaacROSBridge
    from .services.health_monitor import JetsonHealthMonitor

logger = logging.getLogger("edge_core.api")


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
        description="Drone-side companion-computer API for NOMAD",
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
    _ALLOW_INSECURE_REMOTE = (os.environ.get("NOMAD_ALLOW_INSECURE_REMOTE") or "").strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    _AUTH_EXEMPT_PATHS = {"/", "/health", "/docs", "/redoc", "/openapi.json"}
    # Command paths can actuate hardware (payload servos/relays, spray config).
    # They require real authentication even from loopback, and every request to
    # them is audit-logged (SR-SEC-03). Future command routes (velocity, mode)
    # must be added here.
    _COMMAND_PATH_PREFIXES = ("/api/servo/", "/api/spray/")
    _INTERNAL_BRIDGE_TOKEN_HEADER = "X-NOMAD-Internal-Token"
    _INTERNAL_BRIDGE_TOKEN = (os.environ.get("NOMAD_INTERNAL_TOKEN") or "").strip() or None
    _INTERNAL_BRIDGE_ALLOWED_ROUTES: set[tuple[str, str]] = {
        ("POST", "/api/vio/update"),
        ("POST", "/api/slam/mesh/update"),
        ("POST", "/api/servo/camera/tilt"),
        ("GET", "/api/servo/camera/tilt"),
    }
    _INTERNAL_BRIDGE_MIN_TOKEN_LEN = 32

    if _INTERNAL_BRIDGE_TOKEN is not None and len(_INTERNAL_BRIDGE_TOKEN) < _INTERNAL_BRIDGE_MIN_TOKEN_LEN:
        logger.warning(
            "NOMAD_INTERNAL_TOKEN is shorter than %d chars; disabling internal bridge bypass",
            _INTERNAL_BRIDGE_MIN_TOKEN_LEN,
        )
        _INTERNAL_BRIDGE_TOKEN = None

    if _NOMAD_API_KEY is None:
        logger.warning(
            "NOMAD_API_KEY is not configured; only loopback clients are allowed unless NOMAD_ALLOW_INSECURE_REMOTE=true"
        )
    if _NOMAD_API_KEY is not None and _INTERNAL_BRIDGE_TOKEN is None:
        logger.warning("NOMAD_INTERNAL_TOKEN is not configured; internal bridge bypass disabled")

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

    def _is_command_path(request_path: str) -> bool:
        return request_path.startswith(_COMMAND_PATH_PREFIXES)

    def _client_host(request: Request) -> str:
        if request.client is not None and request.client.host is not None:
            return request.client.host
        return "unknown"

    def _audit_command(request: Request, request_path: str, auth_mode: str) -> None:
        """Post-flight audit trail for actuation commands (SR-SEC-03)."""
        logger.info(
            "SC command audit: %s %s from %s auth=%s",
            request.method,
            request_path,
            _client_host(request),
            auth_mode,
        )

    class APIKeyMiddleware(BaseHTTPMiddleware):
        async def dispatch(self, request: Request, call_next):
            request_path = request.url.path.rstrip("/") or "/"
            if _NOMAD_API_KEY is None:
                if _is_command_path(request_path):
                    # Command endpoints never ride the unauthenticated loopback
                    # / insecure-remote fallbacks (SR-SEC-03).
                    if _is_internal_bridge_request(request, request_path):
                        _audit_command(request, request_path, "internal-token")
                        return await call_next(request)
                    logger.warning(
                        "SC command refused (no API key configured): %s %s from %s",
                        request.method,
                        request_path,
                        _client_host(request),
                    )
                    return JSONResponse(
                        status_code=403,
                        content={"detail": "Command endpoints require NOMAD_API_KEY to be configured"},
                    )
                if _ALLOW_INSECURE_REMOTE or _is_loopback_client(request):
                    return await call_next(request)
                return JSONResponse(
                    status_code=401,
                    content={"detail": ("NOMAD_API_KEY is not configured; remote API access is disabled")},
                )
            if request_path in _AUTH_EXEMPT_PATHS:
                return await call_next(request)
            if _is_internal_bridge_request(request, request_path):
                if _is_command_path(request_path):
                    _audit_command(request, request_path, "internal-token")
                return await call_next(request)
            provided_key = request.headers.get("X-API-Key")
            if not provided_key or not hmac.compare_digest(provided_key, _NOMAD_API_KEY):
                if _is_command_path(request_path):
                    logger.warning(
                        "SC command refused (bad/missing key): %s %s from %s",
                        request.method,
                        request_path,
                        _client_host(request),
                    )
                return JSONResponse(
                    status_code=401,
                    content={"detail": "Invalid or missing API key"},
                )
            if _is_command_path(request_path):
                _audit_command(request, request_path, "api-key")
            return await call_next(request)

    app.add_middleware(APIKeyMiddleware)

    # Initialize app.state with all service references (dependency injection)
    app.state.state_manager = state_manager
    app.state.health_monitor = None
    app.state.isaac_bridge = None
    app.state.tailscale_manager = None
    app.state.network_monitor = None
    app.state.camera_service = None
    app.state.mavlink_service = None
    app.state.mode_manager = None
    app.state.isaac_runtime_cache = {
        "timestamp": 0.0,
        "container_running": False,
    }

    ctx = ApiRouteContext(
        logger=logger,
        nomad_api_key=_NOMAD_API_KEY,
        allow_insecure_remote=_ALLOW_INSECURE_REMOTE,
        require_terminal_api_key=_require_admin_api_key,
    )

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
                            capture_output=True,
                            text=True,
                            timeout=3,
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
                    gcs_ip = os.environ.get("GCS_IP", "")
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
                            capture_output=True,
                            timeout=5,
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
                            capture_output=True,
                            text=True,
                            timeout=3,
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

    ctx.docker_exec_bash_success = _docker_exec_pgrep

    from .api_routes.system import register_system_routes

    register_system_routes(app, ctx)

    from .api_routes.terminal import register_terminal_routes

    register_terminal_routes(app, ctx)

    from .api_routes.services import register_services_routes

    register_services_routes(app, ctx)

    from .api_routes.streaming import register_streaming_routes

    register_streaming_routes(app, ctx)

    return app
