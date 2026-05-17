import asyncio
import hmac
import ipaddress
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

def register_system_routes(app, ctx) -> None:
    logger = ctx.logger
    _get_vio_snapshot = ctx.get_vio_snapshot
    _NOMAD_API_KEY = ctx.nomad_api_key
    _ALLOW_INSECURE_REMOTE = bool(getattr(ctx, "allow_insecure_remote", False))

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

        # Target localizer (Task 1 detection) readiness
        enable_od = os.environ.get("ENABLE_OD", "false").strip().lower() == "true"
        try:
            isaac_cache = getattr(request.app.state, "isaac_runtime_cache", {}) or {}
            target_localizer_running = bool(isaac_cache.get("target_localizer_running", False))
        except Exception:
            target_localizer_running = False
        response["target_localizer"] = {
            "enable_od": enable_od,
            "running": target_localizer_running,
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
                # ModemStatus.to_dict() already includes every field the
                # GCS-side dashboard needs (interface, ip_address, NM
                # connection name/state, APN, model, IMEI). Use it directly
                # rather than hand-picking a subset.
                modem = status.modem.to_dict()
        return {
            "tailscale": tailscale,
            "modem": modem,
            "internet_reachable": internet_reachable,
            "gcs_reachable": gcs_reachable,
        }

    # Allow IPv4 dotted quads and DNS hostnames (RFC 1123 labels). Rejects any
    # leading '-' so the value cannot be interpreted as a ping(8) flag, plus
    # whitespace, shell metacharacters, and anything else outside [A-Za-z0-9.-].
    _PING_HOST_RE = re.compile(
        r"^(?=.{1,253}$)(?!-)[A-Za-z0-9](?:[A-Za-z0-9-]{0,62}[A-Za-z0-9])?"
        r"(?:\.(?!-)[A-Za-z0-9](?:[A-Za-z0-9-]{0,62}[A-Za-z0-9])?)*$"
    )

    @app.get("/network/ping/{host}", tags=["Network"])
    async def network_ping(host: str):
        if not _PING_HOST_RE.match(host):
            raise HTTPException(status_code=400, detail="Invalid host")
        try:
            result = await asyncio.to_thread(
                subprocess.run,
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
            if _ALLOW_INSECURE_REMOTE:
                return True
            client_host = ""
            if websocket.client is not None and websocket.client.host is not None:
                client_host = websocket.client.host.strip().lower()
            if client_host.startswith("::ffff:"):
                client_host = client_host.split("::ffff:", 1)[1]
            try:
                if ipaddress.ip_address(client_host).is_loopback:
                    return True
            except ValueError:
                pass
            await websocket.close(code=4003, reason="NOMAD_API_KEY is required for remote WebSocket access")
            return False
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
                # Canonical SLAM frame identifier: "map". nvblox global_frame=map;
                # bridge TF-looks up map->camera_link for pose to match mesh vertices.
                frame_id = "map"
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
                        # Canonical frame is "map". If the stored mesh payload
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

                # Fall back to ROS-frame VIO for pose (used for pose-only frames
                # and for mesh frames that only include one of position/attitude).
                # frame_id is always "map" for all SLAM pose data.
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

                    # Always pull attitude from the freshest VIO snapshot.
                    # Mesh frames use live VIO pose too — using mesh-bundled
                    # (stale) pose caused visible back-and-forward twitching.
                    body_roll = ros_vio.get("body_roll")
                    body_pitch = ros_vio.get("body_pitch")
                    body_yaw = ros_vio.get("body_yaw")
                    body_attitude_available = (
                        body_roll is not None
                        and body_pitch is not None
                        and body_yaw is not None
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

