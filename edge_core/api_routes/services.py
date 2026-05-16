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

def register_services_routes(app, ctx) -> None:
    _probe_isaac_runtime_state = ctx.probe_isaac_runtime_state
    _get_vio_snapshot = ctx.get_vio_snapshot

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
        # ----------------------------------------------------------------
        # All subprocess.run() calls are blocking and must NOT run on the
        # FastAPI event loop — offload them to the default thread-pool so
        # other requests (telemetry, health, video) stay responsive.
        # ----------------------------------------------------------------
        def _blocking_proc_checks() -> tuple:
            """Collect service states that require blocking subprocesses."""
            svc: dict = {}

            # --- mavlink-router ---
            try:
                systemd_status = "inactive"
                systemd_running = False
                try:
                    r = subprocess.run(
                        ["systemctl", "is-active", "mavlink-router"],
                        capture_output=True, text=True, timeout=2,
                    )
                    systemd_status = r.stdout.strip() or "inactive"
                    systemd_running = r.returncode == 0
                except Exception:
                    pass

                process_running = False
                try:
                    pr = subprocess.run(
                        ["pgrep", "-f", "mavlink-routerd"],
                        capture_output=True, text=True, timeout=3,
                    )
                    process_running = pr.returncode == 0
                except Exception:
                    pass

                cubepilot_present = os.path.exists("/dev/ttyACM0")
                if not cubepilot_present:
                    svc["mavlink_router"] = {"status": "no_cubepilot", "running": False, "cubepilot_present": False}
                else:
                    mavlink_running = systemd_running or process_running
                    svc["mavlink_router"] = {
                        "status": "active" if mavlink_running else systemd_status,
                        "running": mavlink_running,
                        "cubepilot_present": True,
                    }
            except Exception as e:
                svc["mavlink_router"] = {"status": "error", "error": str(e)}

            # --- mediamtx ---
            try:
                systemd_running = False
                systemd_status = "inactive"
                try:
                    r = subprocess.run(
                        ["systemctl", "is-active", "mediamtx"],
                        capture_output=True, text=True, timeout=2,
                    )
                    systemd_status = r.stdout.strip() or "inactive"
                    systemd_running = r.returncode == 0
                except Exception:
                    pass

                process_running = False
                try:
                    pr = subprocess.run(
                        ["pgrep", "-f", "mediamtx"],
                        capture_output=True, text=True, timeout=3,
                    )
                    process_running = pr.returncode == 0
                except Exception:
                    pass

                mediamtx_running = systemd_running or process_running
                svc["mediamtx"] = {
                    "status": "active" if mediamtx_running else systemd_status,
                    "running": mediamtx_running,
                }
            except Exception as e:
                svc["mediamtx"] = {"status": "error", "error": str(e)}

            # --- noVNC ---
            try:
                user_running = False
                user_status = "inactive"
                try:
                    r = subprocess.run(
                        ["systemctl", "--user", "is-active", "novnc"],
                        capture_output=True, text=True, timeout=2,
                    )
                    user_status = r.stdout.strip() or "inactive"
                    user_running = r.returncode == 0
                except Exception:
                    pass

                system_running = False
                system_status = "inactive"
                try:
                    r = subprocess.run(
                        ["systemctl", "is-active", "novnc"],
                        capture_output=True, text=True, timeout=2,
                    )
                    system_status = r.stdout.strip() or "inactive"
                    system_running = r.returncode == 0
                except Exception:
                    pass

                websockify_running = x11vnc_running = False
                try:
                    websockify_running = subprocess.run(
                        ["pgrep", "-f", "[w]ebsockify.*6080"],
                        capture_output=True, text=True, timeout=3,
                    ).returncode == 0
                    x11vnc_running = subprocess.run(
                        ["pgrep", "-f", "[x]11vnc.*-rfbport 5900"],
                        capture_output=True, text=True, timeout=3,
                    ).returncode == 0
                except Exception:
                    pass

                novnc_running = user_running or system_running or (websockify_running and x11vnc_running)
                svc["novnc"] = {
                    "status": "active" if novnc_running else (user_status if user_status != "inactive" else system_status),
                    "running": novnc_running,
                    "url": "http://localhost:6080/vnc.html",
                }
            except Exception as e:
                svc["novnc"] = {"status": "error", "error": str(e)}

            # --- Isaac ROS runtime (docker ps + optional docker exec) ---
            # Honor the 8s probe cache: Mission Planner polls this endpoint
            # every ~3s and a force-refresh here spawns 4 docker subprocesses
            # per poll, flooding logs and adding latency.
            runtime_state = _probe_isaac_runtime_state(force_refresh=False)
            return svc, runtime_state

        # Run blocking work in thread pool — event loop stays free
        loop = asyncio.get_running_loop()
        proc_services, runtime_state = await loop.run_in_executor(None, _blocking_proc_checks)

        services = proc_services
        container_running = runtime_state["container_running"]
        nvblox_running = runtime_state["nvblox_running"]
        bridge_running = runtime_state["bridge_running"]

        # Edge Core is always running (we're responding)
        services["edge_core"] = {"status": "active", "running": True}

        # Isaac ROS status (fast — reads app state, no subprocess)
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

        # VIO status (fast — reads in-memory lock)
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

        # Target localization status (fast — reads detection state lock)
        with request.app.state.detection_state_lock:
            detection_enabled = bool(getattr(request.app.state, "detection_enabled", True))
            detection_last_update = request.app.state.detection_last_update
            detection_current_count = len(request.app.state.detected_objects)
            detection_history_count = len(request.app.state.detection_history)
        age_seconds = time.time() - detection_last_update if detection_last_update > 0 else None
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
            + (f" ({detection_current_count} current, {detection_history_count} history)" if detection_enabled else ""),
        }

        services["isaac_ros_container"] = {
            "status": "running" if container_running else "not_running",
            "running": container_running,
        }

        return services

