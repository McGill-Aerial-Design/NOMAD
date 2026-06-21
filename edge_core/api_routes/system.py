# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
import asyncio
import hmac
import ipaddress
import re
import subprocess
import time
from datetime import datetime, timezone

from fastapi import HTTPException, Request, WebSocket
from fastapi.encoders import jsonable_encoder
from fastapi.websockets import WebSocketDisconnect

try:
    import msgpack
except ImportError:  # pragma: no cover - optional Jetson dependency
    msgpack = None


def _vio_rate_hz(trajectory: list[dict]) -> float:
    if len(trajectory) < 2:
        return 0.0

    recent = trajectory[-20:]
    first = float(recent[0].get("timestamp", 0.0))
    last = float(recent[-1].get("timestamp", 0.0))
    duration = last - first
    if duration <= 0:
        return 0.0
    return (len(recent) - 1) / duration


def _external_vio_summary(app_state) -> dict:
    vio = getattr(app_state, "external_vio_state", None)
    if not vio:
        return {
            "health": "unknown",
            "tracking_confidence": 0,
            "position_valid": False,
            "message_rate_hz": 0,
            "source": "none",
        }

    confidence = float(vio.get("confidence", 0.0))
    timestamp = float(vio.get("timestamp", 0.0))
    age_s = max(0.0, time.time() - timestamp) if timestamp > 0 else None
    # A missing/zero timestamp means we have never seen a stamped update, so the
    # estimate cannot be treated as fresh.
    fresh = age_s is not None and age_s < 5.0
    trajectory = getattr(app_state, "vio_trajectory", [])

    return {
        "health": "healthy" if confidence > 0.5 and fresh else "degraded",
        "tracking_confidence": confidence,
        # Consumers gate navigation on this, so a stale fix must read as invalid
        # rather than always reporting valid.
        "position_valid": fresh,
        "message_rate_hz": _vio_rate_hz(trajectory),
        "source": vio.get("source", "external"),
        "timestamp": timestamp,
        "age_s": age_s,
        "x": vio.get("x", 0),
        "y": vio.get("y", 0),
        "z": vio.get("z", 0),
        "roll": vio.get("roll", 0),
        "pitch": vio.get("pitch", 0),
        "yaw": vio.get("yaw", 0),
    }


def register_system_routes(app, ctx) -> None:
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
                "terminal_run": "/api/terminal/run",
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

        # VIO health from external source (ros_http_bridge) if available
        # is provided by modules that register the VIO state.
        response["vio"] = _external_vio_summary(request.app.state)

        return response

    @app.get("/health/detailed", tags=["System"])
    async def detailed_health(request: Request):
        """Get detailed health metrics for monitoring dashboard."""
        health_monitor = request.app.state.health_monitor
        if not health_monitor:
            return {"error": "Health monitor not initialized"}

        response = health_monitor.health.to_dict()
        response["vio"] = _external_vio_summary(request.app.state)
        return response

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

        tailscale_manager = getattr(request.app.state, "tailscale_manager", None)
        if tailscale_manager:
            info = tailscale_manager.info
            tailscale = {
                "status": info.status.value if getattr(info, "status", None) else "unknown",
                "ip": getattr(info, "ip_address", None),
                "hostname": getattr(info, "hostname", "unknown"),
                "peer_count": getattr(info, "peer_count", None),
                "latency_ms": getattr(info, "latency_ms", None),
            }

        network_monitor = getattr(request.app.state, "network_monitor", None)
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
        except HTTPException:
            raise
        except FileNotFoundError:
            raise HTTPException(status_code=503, detail="ping utility not available")
        except OSError as e:
            raise HTTPException(status_code=502, detail=f"Ping error: {e}")

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
        token = websocket.headers.get("X-API-Key") or websocket.query_params.get("token", "")
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
                # VIO state is provided by modules that register VIO data
                # on app.state.external_vio_state
                external_vio = getattr(websocket.app.state, "external_vio_state", None)
                if external_vio:
                    data["external_vio_state"] = external_vio
                else:
                    data["vio_status"] = {
                        "health": "unknown",
                        "tracking_confidence": 0,
                        "position_valid": False,
                        "message_rate_hz": 0,
                        "reset_counter": 0,
                        "source": "none",
                    }

                obstacle_snapshot = getattr(websocket.app.state, "obstacle_distance_last", None)
                if obstacle_snapshot:
                    obstacle_age_s = time.time() - obstacle_snapshot.get("timestamp", 0.0)
                    data["obstacle_distance"] = {
                        "valid": obstacle_age_s < 5.0,
                        "age_seconds": obstacle_age_s,
                        "timestamp": obstacle_snapshot.get("timestamp"),
                        "increment_deg": obstacle_snapshot.get("increment_deg"),
                        "min_distance_cm": obstacle_snapshot.get("min_distance_cm"),
                        "max_distance_cm": obstacle_snapshot.get("max_distance_cm"),
                        "nearest_sector": obstacle_snapshot.get("nearest_sector"),
                        "nearest_distance_cm": obstacle_snapshot.get("nearest_distance_cm"),
                        "nearest_bearing_deg": obstacle_snapshot.get("nearest_bearing_deg"),
                        "distances": obstacle_snapshot.get("distances", []),
                    }

                await websocket.send_json(data)
                await asyncio.sleep(0.1)  # 10Hz
        except WebSocketDisconnect:
            return
