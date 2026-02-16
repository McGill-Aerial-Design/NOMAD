"""
NOMAD Edge Core - REST API

Provides HTTP endpoints for system status, health monitoring,
Task 1/Task 2 operations, and video streaming.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

import asyncio
import json
import logging
import os
import re
import subprocess
from datetime import datetime, timezone
from typing import TYPE_CHECKING, Any, Optional

import cv2
import piexif

from fastapi import FastAPI, HTTPException, Request, WebSocket, Query
from fastapi.encoders import jsonable_encoder
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.websockets import WebSocketDisconnect
from pydantic import BaseModel
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

from .state import StateManager

if TYPE_CHECKING:
    from .health_monitor import JetsonHealthMonitor
    from .isaac_ros_bridge import IsaacROSBridge
    from .nav_controller import NavController

logger = logging.getLogger("edge_core.api")


# ==================== Request/Response Models ====================

class Task1CaptureRequest(BaseModel):
    """Request model for Task 1 capture."""
    heading_deg: Optional[float] = None
    gimbal_pitch_deg: Optional[float] = None
    lidar_distance_m: Optional[float] = None


class Task1CaptureResponse(BaseModel):
    """Response model for Task 1 capture."""
    success: bool
    timestamp: str
    target_text: Optional[str] = None
    position: Optional[dict] = None
    heading_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    roll_deg: Optional[float] = None
    gimbal_pitch_deg: Optional[float] = None
    gimbal_yaw_deg: Optional[float] = None
    capture_folder: Optional[str] = None
    image_name: Optional[str] = None
    metadata_file: Optional[str] = None
    building_location: Optional[str] = None
    error: Optional[str] = None


class Task2HitRequest(BaseModel):
    """Request model for Task 2 target hit."""
    x: float
    y: float
    z: float


class Task1CapturesList(BaseModel):
    """Response model for list of Task 1 captures."""
    captures: list[str]
    count: int


class Task1UploadDescriptionRequest(BaseModel):
    """Request model for uploading AI-generated description."""
    folder: str
    description: str
    provider: str  # 'gemini' or 'ollama'
    model: str


class Task1UploadDescriptionResponse(BaseModel):
    """Response model for description upload."""
    success: bool
    folder: str
    message: str


# Whitelist of allowed terminal commands for safety
# NOTE: mediamtx and mavlink-routerd run as bare processes (started by
# scripts/run/start_nomad_full.sh), NOT as systemd services.  Only nomad
# (edge_core) has a real systemd unit.  Status checks therefore use
# pgrep and restarts use pkill + nohup.
COMMAND_WHITELIST: dict[str, str] = {
    # --- Service status (pgrep for bare processes, systemctl for systemd) ---
    "status_mediamtx": "pgrep -x mediamtx > /dev/null && echo active || echo inactive",
    "status_mavlink": "pgrep -f mavlink-routerd > /dev/null && echo active || echo inactive",
    "status_nomad": "systemctl is-active nomad",
    # --- Service restart ---
    "restart_video": "pkill -x mediamtx 2>/dev/null; sleep 1; nohup mediamtx ~/NOMAD/infra/mediamtx.yml > ~/nomad_logs/mediamtx.log 2>&1 & sleep 1; pgrep -x mediamtx > /dev/null && echo restarted || echo failed",
    "restart_mavlink": "pkill -f mavlink-routerd 2>/dev/null; sleep 1; [ -e /dev/ttyACM0 ] && { GCS=$(tailscale status 2>/dev/null | grep -v \"$(hostname)\" | grep -oP '\\d+\\.\\d+\\.\\d+\\.\\d+' | head -1); nohup mavlink-routerd -e \"${GCS:-192.168.1.255}:14550\" -e 127.0.0.1:14550 /dev/ttyACM0 > ~/nomad_logs/mavlink.log 2>&1 & sleep 2; pgrep -f mavlink-routerd > /dev/null && echo restarted || echo failed; } || echo 'no CubePilot'",
    "restart_edge_core": "sudo systemctl restart nomad",
    # --- Service start / stop ---
    "start_mediamtx": "pgrep -x mediamtx > /dev/null && echo 'already running' || (nohup mediamtx ~/NOMAD/infra/mediamtx.yml > ~/nomad_logs/mediamtx.log 2>&1 & sleep 1; echo started)",
    "stop_mediamtx": "pkill -x mediamtx 2>&1 && echo stopped || echo 'not running'",
    "start_mavlink": "[ -e /dev/ttyACM0 ] && { pgrep -f mavlink-routerd > /dev/null && echo 'already running' || { GCS=$(tailscale status 2>/dev/null | grep -v \"$(hostname)\" | grep -oP '\\d+\\.\\d+\\.\\d+\\.\\d+' | head -1); nohup mavlink-routerd -e \"${GCS:-192.168.1.255}:14550\" -e 127.0.0.1:14550 /dev/ttyACM0 > ~/nomad_logs/mavlink.log 2>&1 & sleep 2; echo started; }; } || echo 'no CubePilot'",
    "stop_mavlink": "pkill -f mavlink-routerd 2>&1 && echo stopped || echo 'not running'",
    "start_nomad": "sudo systemctl start nomad 2>&1 && echo started || echo failed",
    "stop_nomad": "sudo systemctl stop nomad 2>&1 && echo stopped || echo failed",
    # --- System commands ---
    "reboot_jetson": "sudo reboot",
    "shutdown_jetson": "sudo shutdown -h now",
    "check_disk": "df -h",
    "check_memory": "free -h",
    "check_processes": "ps aux | head -20",
    "tailscale_status": "tailscale status",
    "network_info": "ip addr show",
    "gpu_status": "tegrastats --interval 1000 --stop 2",
}


# ==================== Helper Functions ====================

def _gps_to_exif(lat: float, lon: float, alt: Optional[float] = None) -> dict:
    """
    Convert GPS coordinates to EXIF GPS format.
    
    Args:
        lat: Latitude in decimal degrees
        lon: Longitude in decimal degrees
        alt: Altitude in meters (optional)
    
    Returns:
        Dictionary with EXIF GPS tags
    """
    def _decimal_to_dms(decimal: float) -> tuple[tuple[int, int], tuple[int, int], tuple[int, int]]:
        """Convert decimal degrees to degrees, minutes, seconds."""
        decimal = abs(decimal)
        degrees = int(decimal)
        minutes = int((decimal - degrees) * 60)
        seconds = int(((decimal - degrees) * 60 - minutes) * 60 * 100)
        return ((degrees, 1), (minutes, 1), (seconds, 100))
    
    gps_dict = {}
    
    # Latitude
    gps_dict[piexif.GPSIFD.GPSLatitude] = _decimal_to_dms(lat)
    gps_dict[piexif.GPSIFD.GPSLatitudeRef] = b'N' if lat >= 0 else b'S'
    
    # Longitude
    gps_dict[piexif.GPSIFD.GPSLongitude] = _decimal_to_dms(lon)
    gps_dict[piexif.GPSIFD.GPSLongitudeRef] = b'E' if lon >= 0 else b'W'
    
    # Altitude (if provided)
    if alt is not None:
        gps_dict[piexif.GPSIFD.GPSAltitude] = (int(abs(alt) * 100), 100)
        gps_dict[piexif.GPSIFD.GPSAltitudeRef] = 0 if alt >= 0 else 1
    
    return gps_dict


class TerminalCommandRequest(BaseModel):
    """Request model for terminal command execution."""
    command_name: str  # Must be a key in COMMAND_WHITELIST
    timeout: int = 10


class TerminalExecRequest(BaseModel):
    """Request model for arbitrary terminal command execution."""
    command: str
    timeout: int = 30


class TerminalCommandResponse(BaseModel):
    """Response model for terminal command."""
    success: bool
    stdout: str
    stderr: str
    return_code: int
    command_executed: Optional[str] = None


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


class NavVelocityRequest(BaseModel):
    """Request model for navigation velocity command from ROS nav2/nvblox."""
    timestamp: float
    vx: float       # Forward velocity (m/s)
    vy: float       # Lateral velocity (m/s)
    vz: float       # Vertical velocity (m/s)
    yaw_rate: float # Yaw rate (rad/s)
    source: str = "nav2"


class NavPositionRequest(BaseModel):
    """Request model for navigation position target."""
    x: float        # North position (NED meters)
    y: float        # East position (NED meters)
    z: float        # Down position (NED meters)
    yaw: float      # Heading (radians)
    source: str = "nav2"


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
    """Register ZED camera service with API via app.state."""
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
    app.add_middleware(
        CORSMiddleware,
        allow_origins=allowed_origins,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    
    # API key authentication middleware
    # If NOMAD_API_KEY is set, require X-API-Key header on non-exempt endpoints.
    # If NOMAD_API_KEY is not set, skip authentication (development mode).
    _NOMAD_API_KEY = os.environ.get("NOMAD_API_KEY")
    _AUTH_EXEMPT_PATHS = {"/", "/health", "/docs", "/redoc", "/openapi.json"}

    class APIKeyMiddleware(BaseHTTPMiddleware):
        async def dispatch(self, request: Request, call_next):
            if _NOMAD_API_KEY is None:
                # Development mode - no authentication
                return await call_next(request)
            if request.url.path in _AUTH_EXEMPT_PATHS:
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
    
    # VIO state from external sources (ROS bridge)
    app.state.external_vio_state: Optional[dict] = None
    app.state.vio_trajectory: list[dict] = []  # List of {x, y, z, timestamp} points
    app.state.vio_trajectory_max_points: int = 1000  # Keep last N points
    app.state.exclusion_map: list[dict] = []

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
            }
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
            response.update({
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
            })
            
            # Override status based on thermal state
            if health.thermal_zone == "critical":
                response["status"] = "critical"
            elif health.thermal_zone == "warning" or health.throttled:
                response["status"] = "warning"
        
        # Add VIO health from external source (ros_http_bridge) if available
        external_vio = request.app.state.external_vio_state
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
        external_vio = request.app.state.external_vio_state
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
                "status": info.status.value if getattr(info, "status", None) else "unknown",
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
            raise HTTPException(status_code=503, detail="Tailscale manager not initialized")
        
        ok = await tailscale_manager.reconnect()
        return {
            "success": ok, 
            "message": "Tailscale reconnection triggered" if ok else "Failed to trigger reconnection"}
    
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


    @app.websocket("/ws/state")
    async def ws_state(websocket: WebSocket):
        """WebSocket endpoint for real-time state updates (10Hz)."""
        await websocket.accept()
        try:
            while True:
                state = websocket.app.state.state_manager.get_state()
                data = jsonable_encoder(state)
                
                # Add additional real-time data
                health_monitor = websocket.app.state.health_monitor
                if health_monitor:
                    data["jetson_health"] = health_monitor.health.to_dict()
                external_vio = websocket.app.state.external_vio_state
                if external_vio:
                    data["vio_status"] = external_vio
                
                await websocket.send_json(data)
                await asyncio.sleep(0.1)  # 10Hz
        except WebSocketDisconnect:
            return

    # ==================== Task 1: Recon (Outdoor) ====================

    @app.post("/api/task/1/capture", tags=["Task 1"], response_model=Task1CaptureResponse)
    async def task1_capture(task_request: Task1CaptureRequest = None, request: Request = None):
        """
        Capture snapshot for Task 1 recon mission.
        
        Captures current position, heading, camera image with EXIF metadata,
        and comprehensive metadata in JSON format.
        Used for outdoor GPS-based reconnaissance.
        """
        state = request.app.state.state_manager.get_state()
        
        # Get values from request or current state
        heading = task_request.heading_deg if task_request and task_request.heading_deg else state.heading_deg
        gimbal_pitch = task_request.gimbal_pitch_deg if task_request and task_request.gimbal_pitch_deg else state.gimbal_pitch_deg
        gimbal_yaw = state.gimbal_yaw_deg
        pitch = state.pitch_deg
        roll = state.roll_deg
        
        # Get building location from environment
        building_name = os.environ.get("TASK1_BUILDING_NAME", "Unknown Building")
        building_lat = os.environ.get("TASK1_BUILDING_LAT")
        building_lon = os.environ.get("TASK1_BUILDING_LON")
        
        building_location = {
            "name": building_name,
            "lat": float(building_lat) if building_lat else None,
            "lon": float(building_lon) if building_lon else None,
        }
        
        if not state.gps_fix:
            logger.warning("Task 1 capture: No GPS fix - position data will be unavailable")

        # Create capture record with timestamp
        timestamp = datetime.now(timezone.utc)
        timestamp_str = timestamp.strftime('%Y%m%d_%H%M%S')
        
        # Create timestamped folder structure: data/task1_captures/YYYYMMDD_HHMMSS/
        base_dir = "./data/task1_captures"
        capture_folder = os.path.join(base_dir, timestamp_str)
        os.makedirs(capture_folder, exist_ok=True)
        
        # Prepare metadata
        metadata = {
            "timestamp": timestamp.isoformat(),
            "gps": {
                "lat": state.gps_lat,
                "lon": state.gps_lon,
                "alt": state.gps_alt,
            },
            "ahrs": {
                "heading_deg": heading,
                "pitch_deg": pitch,
                "roll_deg": roll,
            },
            "gimbal": {
                "pitch_deg": gimbal_pitch,
                "yaw_deg": gimbal_yaw,
            },
            "building_location": building_location,
            "photo_path": None,  # Will be set after photo capture
        }
        
        image_filename = "photo.jpg"
        metadata_filename = "metadata.json"
        image_path = os.path.join(capture_folder, image_filename)
        metadata_path = os.path.join(capture_folder, metadata_filename)
        
        # Capture image from ZED camera if available
        camera_service = request.app.state.camera_service
        if camera_service:
            try:
                frame = camera_service.get_frame()
                if frame and frame.left_image is not None:
                    # Convert from BGRA to BGR if needed (ZED often returns BGRA)
                    if frame.left_image.shape[2] == 4:
                        image_bgr = cv2.cvtColor(frame.left_image, cv2.COLOR_BGRA2BGR)
                    else:
                        image_bgr = frame.left_image
                    
                    # Save image temporarily to embed EXIF
                    temp_path = image_path + ".temp"
                    cv2.imwrite(temp_path, image_bgr)
                    
                    # Prepare EXIF data
                    exif_dict = {
                        "0th": {},
                        "Exif": {},
                        "GPS": {},
                    }
                    
                    # Embed GPS coordinates in EXIF
                    if state.gps_lat is not None and state.gps_lon is not None:
                        exif_dict["GPS"] = _gps_to_exif(state.gps_lat, state.gps_lon, state.gps_alt)
                    
                    # Embed timestamp in EXIF DateTime
                    exif_dict["0th"][piexif.ImageIFD.DateTime] = timestamp.strftime("%Y:%m:%d %H:%M:%S").encode()
                    
                    # Embed heading and AHRS data in ImageDescription
                    heading_str = f"{heading:.1f}" if heading is not None else "N/A"
                    pitch_str = f"{pitch:.1f}" if pitch is not None else "N/A"
                    roll_str = f"{roll:.1f}" if roll is not None else "N/A"
                    description = f"Heading: {heading_str}deg, Pitch: {pitch_str}deg, Roll: {roll_str}deg"
                    exif_dict["0th"][piexif.ImageIFD.ImageDescription] = description.encode()
                    
                    # Dump EXIF data and save final image
                    exif_bytes = piexif.dump(exif_dict)
                    piexif.insert(exif_bytes, temp_path, image_path)
                    
                    # Remove temporary file
                    os.remove(temp_path)
                    
                    logger.info(f"Task 1 image with EXIF saved: {image_path}")
                    
                    # Update metadata with photo path
                    metadata["photo_path"] = image_path
                else:
                    logger.warning("Task 1 capture: Camera frame not available")
            except Exception as e:
                logger.error(f"Task 1 image capture failed: {e}")
                # Continue to save metadata even if image capture fails
        else:
            logger.warning("Task 1 capture: Camera service not available")
        
        # Save metadata.json
        try:
            with open(metadata_path, "w") as f:
                json.dump(metadata, f, indent=2)
            
            logger.info(f"Task 1 metadata saved: {metadata_path}")
            
            # Also save to mission log for backward compatibility
            log_dir = os.environ.get("NOMAD_LOG_DIR", "./data/mission_logs")
            os.makedirs(log_dir, exist_ok=True)
            log_file = os.path.join(log_dir, f"task1_{timestamp_str}.json")
            with open(log_file, "w") as f:
                json.dump(metadata, f, indent=2)
            
            return Task1CaptureResponse(
                success=True,
                timestamp=timestamp.isoformat(),
                target_text=(
                    f"Captured at {state.gps_lat:.6f}, {state.gps_lon:.6f}"
                    if state.gps_lat is not None and state.gps_lon is not None
                    else "Captured (GPS coordinates pending)"
                ),
                position=metadata["gps"],
                heading_deg=heading,
                pitch_deg=pitch,
                roll_deg=roll,
                gimbal_pitch_deg=gimbal_pitch,
                gimbal_yaw_deg=gimbal_yaw,
                capture_folder=capture_folder,
                image_name=image_filename,
                metadata_file=metadata_filename,
                building_location=building_name,
            )
            
        except Exception as e:
            logger.error(f"Task 1 capture failed: {e}")
            return Task1CaptureResponse(
                success=False,
                timestamp=timestamp.isoformat(),
                error=str(e)
            )

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
                entry for entry in entries
                if os.path.isdir(os.path.join(base_dir, entry))
            ]
            
            # Filter to only timestamp-pattern folders (YYYYMMDD_HHMMSS)
            timestamp_pattern = re.compile(r'^\d{8}_\d{6}$')
            valid_folders = [
                folder for folder in folders
                if timestamp_pattern.match(folder)
            ]
            
            # Sort by timestamp descending (newest first)
            valid_folders.sort(reverse=True)
            
            return Task1CapturesList(captures=valid_folders, count=len(valid_folders))
            
        except Exception as e:
            logger.error(f"Failed to list Task 1 captures: {e}")
            raise HTTPException(status_code=500, detail=f"Failed to list captures: {str(e)}")

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
        timestamp_pattern = re.compile(r'^\d{8}_\d{6}$')
        if not timestamp_pattern.match(folder):
            raise HTTPException(status_code=400, detail="Invalid folder name format")
        
        # Security: Validate filename - whitelist allowed files
        allowed_files = ['photo.jpg', 'metadata.json', 'description.txt']
        if filename not in allowed_files:
            raise HTTPException(
                status_code=400,
                detail=f"Invalid filename. Allowed: {', '.join(allowed_files)}"
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
                status_code=404,
                detail=f"File not found: {folder}/{filename}"
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

    @app.post("/api/task/1/upload_description", tags=["Task 1"], response_model=Task1UploadDescriptionResponse)
    async def upload_task1_description(request: Task1UploadDescriptionRequest):
        """
        Upload AI-generated description for a capture.
        
        Request body:
        {
            "folder": "20260202_120000",
            "description": "Scene description text...",
            "provider": "gemini" or "ollama",
            "model": "gemini-1.5-flash" or "llava:13b"
        }
        
        Saves description.txt to data/task1_captures/{folder}/description.txt
        Also adds AI metadata to metadata.json (provider, model, timestamp)
        """
        # Security: Validate folder name matches timestamp pattern
        timestamp_pattern = re.compile(r'^\d{8}_\d{6}$')
        if not timestamp_pattern.match(request.folder):
            raise HTTPException(status_code=400, detail="Invalid folder name format")
        
        # Security: Prevent path traversal
        if ".." in request.folder or "/" in request.folder or "\\" in request.folder:
            raise HTTPException(status_code=400, detail="Invalid folder name")
        
        # Security: Limit description size (10KB)
        max_description_size = 10 * 1024  # 10KB
        if len(request.description.encode('utf-8')) > max_description_size:
            raise HTTPException(
                status_code=413,
                detail=f"Description too large. Maximum size: {max_description_size} bytes"
            )
        
        # Validate provider
        allowed_providers = ['gemini', 'ollama', 'openrouter']
        if request.provider not in allowed_providers:
            raise HTTPException(
                status_code=400,
                detail=f"Invalid provider. Allowed: {', '.join(allowed_providers)}"
            )
        
        # Build folder path
        base_dir = "./data/task1_captures"
        folder_path = os.path.join(base_dir, request.folder)
        
        # Normalize path and ensure it's within base_dir (additional security)
        base_dir_abs = os.path.abspath(base_dir)
        folder_path_abs = os.path.abspath(folder_path)
        if not folder_path_abs.startswith(base_dir_abs):
            raise HTTPException(status_code=400, detail="Invalid folder path")
        
        # Check if folder exists
        if not os.path.exists(folder_path):
            raise HTTPException(
                status_code=404,
                detail=f"Capture folder not found: {request.folder}"
            )
        
        try:
            # Save description.txt
            description_path = os.path.join(folder_path, "description.txt")
            with open(description_path, "w", encoding="utf-8") as f:
                f.write(request.description)
            
            logger.info(f"Saved description to {description_path}")
            
            # Update metadata.json with AI info
            metadata_path = os.path.join(folder_path, "metadata.json")
            if os.path.exists(metadata_path):
                try:
                    with open(metadata_path, "r", encoding="utf-8") as f:
                        metadata = json.load(f)
                    
                    # Add AI processing metadata
                    metadata["ai_processing"] = {
                        "provider": request.provider,
                        "model": request.model,
                        "processed_at": datetime.now(timezone.utc).isoformat(),
                        "description_file": "description.txt",
                    }
                    
                    # Save updated metadata
                    with open(metadata_path, "w", encoding="utf-8") as f:
                        json.dump(metadata, f, indent=2)
                    
                    logger.info(f"Updated metadata with AI info: {metadata_path}")
                    
                except Exception as e:
                    logger.warning(f"Failed to update metadata.json: {e}")
                    # Continue - description was saved successfully
            
            return Task1UploadDescriptionResponse(
                success=True,
                folder=request.folder,
                message=f"Description saved successfully using {request.provider}/{request.model}"
            )
            
        except Exception as e:
            logger.error(f"Failed to save description: {e}")
            raise HTTPException(
                status_code=500,
                detail=f"Failed to save description: {str(e)}"
            )

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
        logger.info(f"Task 2 target hit registered: ({hit_request.x}, {hit_request.y}, {hit_request.z})")
        
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
        external_vio_state = request.app.state.external_vio_state
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
        # Store latest state
        request.app.state.external_vio_state = {
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
            "source": vio_request.source,
        }
        
        # Add to trajectory
        request.app.state.vio_trajectory.append({
            "x": vio_request.x,
            "y": vio_request.y,
            "z": vio_request.z,
            "timestamp": vio_request.timestamp,
        })
        
        # Trim trajectory if too long
        if len(request.app.state.vio_trajectory) > request.app.state.vio_trajectory_max_points:
            request.app.state.vio_trajectory = request.app.state.vio_trajectory[-request.app.state.vio_trajectory_max_points:]
        
        return {"success": True, "trajectory_points": len(request.app.state.vio_trajectory)}

    @app.get("/api/vio/pose", tags=["VIO"])
    async def vio_pose(request: Request):
        """Get current VIO pose (position and orientation)."""
        external_vio_state = request.app.state.external_vio_state
        if external_vio_state:
            return external_vio_state
        
        isaac_bridge = request.app.state.isaac_bridge
        if isaac_bridge and isaac_bridge.vio_state:
            vio = isaac_bridge.vio_state
            return {
                "timestamp": vio.timestamp,
                "x": vio.x,
                "y": vio.y,
                "z": vio.z,
                "roll": vio.roll,
                "pitch": vio.pitch,
                "yaw": vio.yaw,
                "vx": vio.vx,
                "vy": vio.vy,
                "vz": vio.vz,
                "confidence": vio.confidence,
                "source": "isaac_ros",
            }
        
        return {"valid": False, "message": "No VIO data available"}

    @app.get("/api/vio/trajectory", tags=["VIO"])
    async def vio_trajectory(request: Request, max_points: int = Query(default=100, le=1000)):
        """
        Get VIO trajectory for visualization.
        
        Returns a list of (x, y, z) points representing the drone's path.
        Use max_points to limit the response size.
        """
        trajectory = request.app.state.vio_trajectory
        points = trajectory[-max_points:] if trajectory else []
        return {
            "total_points": len(trajectory),
            "returned_points": len(points),
            "trajectory": points,
        }

    @app.delete("/api/vio/trajectory", tags=["VIO"])
    async def vio_clear_trajectory(request: Request):
        """Clear the VIO trajectory history."""
        count = len(request.app.state.vio_trajectory)
        request.app.state.vio_trajectory = []
        return {"success": True, "cleared_points": count}

    @app.post("/api/vio/reset_origin", tags=["VIO"])
    async def vio_reset_origin(request: Request):
        """Reset VIO tracking origin to current position."""
        # Clear trajectory on reset
        request.app.state.vio_trajectory = []
        
        vio_pipeline = None  # Deprecated: VIO handled via ros_http_bridge
        if not vio_pipeline:
            # Just clear trajectory if no VIO pipeline
            return {
                "success": True,
                "reset_counter": 0,
                "message": "Trajectory cleared (VIO managed by ros_http_bridge)",
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
        nav_controller = request.app.state.nav_controller
        if not nav_controller:
            raise HTTPException(status_code=503, detail="Navigation controller not initialized")
        
        success = nav_controller.send_velocity(
            vx=nav_request.vx,
            vy=nav_request.vy,
            vz=nav_request.vz,
            yaw_rate=nav_request.yaw_rate,
            source=nav_request.source,
        )
        
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
            raise HTTPException(status_code=503, detail="Navigation controller not initialized")
        
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
            raise HTTPException(status_code=503, detail="Navigation controller not initialized")
        
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
            raise HTTPException(status_code=503, detail="Navigation controller not initialized")
        
        success = nav_controller.enable_guided_mode()
        return {
            "success": success,
            "message": "GUIDED mode requested" if success else "Failed to request GUIDED mode",
        }

    # ==================== Camera Endpoints ====================

    @app.get("/api/camera/status", tags=["Camera"])
    async def camera_status(request: Request):
        """Get ZED camera status."""
        camera_service = request.app.state.camera_service
        if not camera_service:
            return {
                "initialized": False,
                "tracking": False,
                "fps": 0,
            }
        
        return {
            "initialized": camera_service.is_initialized,
            "tracking": camera_service.is_tracking,
            "fps": camera_service.current_fps,
        }

    @app.post("/api/camera/reset_tracking", tags=["Camera"])
    async def camera_reset_tracking(request: Request):
        """Reset camera positional tracking."""
        camera_service = request.app.state.camera_service
        if not camera_service:
            raise HTTPException(status_code=503, detail="Camera not initialized")
        
        success = camera_service.reset_tracking()
        return {"success": success}

    # ==================== Terminal Endpoints ====================

    @app.post("/api/terminal/run", tags=["Terminal"], response_model=TerminalCommandResponse)
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
                cmd_parts = command_str.split()
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

    @app.post("/api/terminal/exec", tags=["Terminal"], response_model=TerminalCommandResponse)
    async def exec_terminal_command(request: TerminalExecRequest):
        """
        Execute an arbitrary shell command on the Jetson.

        Intended for the Mission Planner built-in terminal.
        Commands are executed via ``bash -c`` so pipes, redirects, and
        compound statements work as expected.
        """
        command_str = request.command.strip()
        if not command_str:
            raise HTTPException(status_code=400, detail="Empty command")

        try:
            result = subprocess.run(
                ["bash", "-c", command_str],
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
        - edge_core: This API service (always running if you see this)
        - isaac_ros: Isaac ROS bridge status
        - vio: VIO pipeline status
        """
        services = {}
        
        # Check mavlink-router
        try:
            result = subprocess.run(
                ["systemctl", "is-active", "mavlink-router"],
                capture_output=True,
                text=True,
                timeout=2,
            )
            services["mavlink_router"] = {
                "status": result.stdout.strip(),
                "running": result.returncode == 0,
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
            services["mediamtx"] = {
                "status": result.stdout.strip(),
                "running": result.returncode == 0,
            }
        except Exception as e:
            services["mediamtx"] = {"status": "error", "error": str(e)}
        
        # Edge Core is always running (we're responding)
        services["edge_core"] = {
            "status": "active",
            "running": True,
        }
        
        # Isaac ROS status
        isaac_bridge = request.app.state.isaac_bridge
        if isaac_bridge:
            services["isaac_ros"] = {
                "status": "active",
                "running": True,
                **isaac_bridge.get_status(),
            }
        else:
            services["isaac_ros"] = {
                "status": "not_initialized",
                "running": False,
                "message": "Isaac ROS bridge not enabled",
            }
        
        # VIO status
        external_vio_state = request.app.state.external_vio_state
        vio_trajectory = request.app.state.vio_trajectory
        
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
        
        # Check for Isaac ROS Docker container
        try:
            result = subprocess.run(
                ["docker", "ps", "--filter", "name=nomad_isaac_ros", "--format", "{{.Status}}"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            container_status = result.stdout.strip()
            services["isaac_ros_container"] = {
                "status": container_status if container_status else "not_running",
                "running": bool(container_status),
            }
        except Exception:
            services["isaac_ros_container"] = {
                "status": "docker_unavailable",
                "running": False,
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
        # Check container status first
        container_running = False
        nvblox_running = False
        bridge_running = False
        try:
            result = subprocess.run(
                ["docker", "ps", "--filter", "name=nomad_isaac_ros", "--format", "{{.Status}}"],
                capture_output=True, text=True, timeout=5,
            )
            container_running = bool(result.stdout.strip())
        except Exception:
            pass

        if container_running:
            try:
                result = subprocess.run(
                    ["docker", "exec", "nomad_isaac_ros", "bash", "-c",
                     "ps aux | grep -v grep | grep -c component_container 2>/dev/null || echo 0"],
                    capture_output=True, text=True, timeout=5,
                )
                nvblox_running = int(result.stdout.strip()) > 0
            except Exception:
                pass
            try:
                result = subprocess.run(
                    ["docker", "exec", "nomad_isaac_ros", "bash", "-c",
                     "ps aux | grep -v grep | grep -c ros_http_bridge 2>/dev/null || echo 0"],
                    capture_output=True, text=True, timeout=5,
                )
                bridge_running = int(result.stdout.strip()) > 0
            except Exception:
                pass

        isaac_bridge = request.app.state.isaac_bridge
        if not isaac_bridge:
            # No Python-side bridge, but the external ROS-HTTP bridge may be active
            external_bridge_active = container_running and nvblox_running and bridge_running
            return {
                "available": external_bridge_active,
                "backend": "ros_http_bridge" if external_bridge_active else "not_initialized",
                "container_running": container_running,
                "nvblox_running": nvblox_running,
                "bridge_running": bridge_running,
                "message": "Active via external ROS-HTTP bridge" if external_bridge_active
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
        
        try:
            # Run in background
            process = subprocess.Popen(
                ["bash", script_path, "start"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
            )
            
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
    async def isaac_launch_nvblox():
        """
        Launch nvblox + ROS-HTTP bridge inside a running container.

        Lightweight alternative to /api/isaac/start: does NOT install deps
        or rebuild packages.  Assumes the container is already running and
        packages are already built.  Kills any existing nvblox / bridge
        processes first, applies NOMAD config overlay, then launches both.
        """
        container = "nomad_isaac_ros"

        # Verify container is running
        try:
            result = subprocess.run(
                ["docker", "ps", "--filter", f"name={container}", "--format", "{{.Names}}"],
                capture_output=True, text=True, timeout=5,
            )
            if container not in result.stdout:
                return {"success": False, "error": "Container not running"}
        except Exception as e:
            return {"success": False, "error": str(e)}

        # Bridge script is available via volume mount at /workspaces/isaac_ros-dev/edge_core/

        # Build inline launch script
        launch_script = r"""#!/bin/bash
set -e
source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH=/usr/local/zed/lib:$LD_LIBRARY_PATH

# Kill previous instances
pkill -f 'ros2 launch.*nvblox' 2>/dev/null || true
pkill -f component_container 2>/dev/null || true
pkill -f ros_http_bridge 2>/dev/null || true
sleep 2

# Overlay NOMAD nvblox config
NOMAD_CFG=/workspaces/isaac_ros-dev/config/nvblox_performance.yaml
NVBLOX_BASE=$(python3 -c "from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory('nvblox_examples_bringup'))" 2>/dev/null)/config/nvblox/nvblox_base.yaml
if [ -f "$NOMAD_CFG" ] && [ -f "$NVBLOX_BASE" ]; then
    cp "$NOMAD_CFG" "$NVBLOX_BASE"
    echo "Applied NOMAD nvblox config"
fi

# Patch ZED publish resolution
sed -i 's/pub_downscale_factor: 2\.0/pub_downscale_factor: 1.0/' \
    /workspaces/isaac_ros-dev/install/zed_wrapper/share/zed_wrapper/config/common.yaml 2>/dev/null

# Launch nvblox
ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2 &
echo $! > /tmp/zed_nvblox.pid

# Wait for topics then launch bridge
sleep 10
python3 /workspaces/isaac_ros-dev/edge_core/ros_http_bridge.py --host localhost --port 8000 --rate 30 --vio-topic /zed/zed_node/odom &
echo $! > /tmp/ros_bridge.pid

wait
"""
        try:
            # Write launch script into container
            subprocess.run(
                ["docker", "exec", container, "bash", "-c",
                 f"cat > /tmp/launch_nvblox_bridge.sh << 'EOFSCRIPT'\n{launch_script}\nEOFSCRIPT\nchmod +x /tmp/launch_nvblox_bridge.sh"],
                capture_output=True, timeout=10, check=True,
            )
            # Run in background
            subprocess.run(
                ["docker", "exec", "-d", container, "bash", "-c",
                 "bash /tmp/launch_nvblox_bridge.sh > /tmp/zed_nvblox.log 2>&1"],
                capture_output=True, timeout=10,
            )
            return {
                "success": True,
                "message": "nvblox + ROS-HTTP bridge launching. ZED init takes ~15s.",
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    @app.post("/api/isaac/stop-nvblox", tags=["Isaac ROS"])
    async def isaac_stop_nvblox():
        """
        Stop nvblox and ROS-HTTP bridge without stopping the container.
        """
        container = "nomad_isaac_ros"
        try:
            for proc in ["ros_http_bridge", "component_container", "ros2 launch.*nvblox"]:
                subprocess.run(
                    ["docker", "exec", container, "pkill", "-f", proc],
                    capture_output=True, timeout=5,
                )
            return {"success": True, "message": "nvblox and bridge stopped"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    @app.get("/api/isaac/logs", tags=["Isaac ROS"])
    async def isaac_logs(log_type: str = Query(default="all", description="Log type: all, zed, bridge")):
        """Get Isaac ROS container logs."""
        try:
            if log_type == "zed":
                result = subprocess.run(
                    ["docker", "exec", "nomad_isaac_ros", "tail", "-50", "/tmp/zed_nvblox.log"],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
            elif log_type == "bridge":
                result = subprocess.run(
                    ["docker", "exec", "nomad_isaac_ros", "tail", "-50", "/tmp/ros_bridge.log"],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
            else:
                zed_result = subprocess.run(
                    ["docker", "exec", "nomad_isaac_ros", "tail", "-25", "/tmp/zed_nvblox.log"],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                bridge_result = subprocess.run(
                    ["docker", "exec", "nomad_isaac_ros", "tail", "-25", "/tmp/ros_bridge.log"],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                return {
                    "zed_nvblox": zed_result.stdout if zed_result.returncode == 0 else zed_result.stderr,
                    "ros_bridge": bridge_result.stdout if bridge_result.returncode == 0 else bridge_result.stderr,
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
                    } if d.world_x is not None else None,
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
            exclusion_map.append({
                "x": hit_request.x,
                "y": hit_request.y,
                "z": hit_request.z,
                "timestamp": datetime.now(timezone.utc).isoformat(),
            })
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

    # ==================== Video Streaming Endpoints ====================
    # Isaac ROS H.264 video streaming with dynamic topic switching
    
    from .video_stream_manager import get_video_stream_manager
    
    @app.get("/api/video/topics", tags=["Video"])
    async def get_video_topics():
        """
        List available ROS image topics from ZED camera.
        
        Returns topics with both full path and trimmed display names for UI:
        - Full: /zed/zed_node/rgb/image_rect_color
        - Display: zed: rgb/image_rect_color
        
        Use the full name when switching topics via POST /api/video/source.
        """
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(status_code=503, detail="Video stream manager not initialized")
        
        topics = mgr.list_topics()
        return {
            "topics": [t.to_dict() for t in topics],
            "count": len(topics)
        }

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
            raise HTTPException(status_code=503, detail="Video stream manager not initialized")
        
        status = mgr.get_status()
        return status.to_dict()

    @app.post("/api/video/source", tags=["Video"])
    async def switch_video_source(topic: str = Query(..., description="ROS image topic to stream")):
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
            raise HTTPException(status_code=503, detail="Video stream manager not initialized")
        
        success = mgr.switch_topic(topic)
        if not success:
            raise HTTPException(status_code=500, detail="Failed to switch video source")
        
        status = mgr.get_status()
        return {
            "success": True,
            "topic": topic,
            "rtsp_url": mgr.get_rtsp_url(),
            "status": status.to_dict()
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
            "rtsp_url": mgr.get_rtsp_url()
        }

    @app.post("/api/video/start", tags=["Video"])
    async def start_video_stream():
        """
        Start the video streaming pipeline.
        
        Launches the video relay node inside the Isaac ROS container.
        This is typically called automatically on startup.
        """
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(status_code=503, detail="Video stream manager not initialized")
        
        success, reason = mgr.start_with_reason()
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to start video stream: {reason}")
        
        return {
            "success": True,
            "rtsp_url": mgr.get_rtsp_url(),
            "message": "Video pipeline started"
        }

    @app.post("/api/video/stop", tags=["Video"])
    async def stop_video_stream():
        """
        Stop the video streaming pipeline.
        """
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(status_code=503, detail="Video stream manager not initialized")
        
        success = mgr.stop()
        return {
            "success": success,
            "message": "Video pipeline stopped" if success else "Failed to stop"
        }

    @app.post("/api/video/restart", tags=["Video"])
    async def restart_video_stream():
        """
        Restart the video streaming pipeline.
        
        Useful for recovery from errors or after container restart.
        """
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(status_code=503, detail="Video stream manager not initialized")
        
        mgr.stop()
        import asyncio
        await asyncio.sleep(2)
        
        success, reason = mgr.start_with_reason()
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to restart video stream: {reason}")
        
        return {
            "success": True,
            "rtsp_url": mgr.get_rtsp_url(),
            "message": "Video pipeline restarted"
        }

    @app.post("/api/video/bridges/start", tags=["Video"])
    async def start_video_bridges():
        """
        Start video bridges (legacy endpoint for compatibility).
        
        This is an alias for /api/video/start since we simplified to a single bridge.
        Mission Planner Service Control Panel calls this endpoint.
        """
        return await start_video_stream()

    @app.get("/api/video/logs", tags=["Video"])
    async def get_video_logs(lines: int = Query(50, description="Number of log lines to return")):
        """
        Get recent logs from the video relay process.
        
        Useful for debugging video streaming issues.
        """
        mgr = get_video_stream_manager()
        if not mgr:
            raise HTTPException(status_code=503, detail="Video stream manager not initialized")
        
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
            raise HTTPException(status_code=503, detail="Video stream manager not initialized")
        
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
                    "error_count": status.error_count
                },
                "secondary": {
                    "state": "unavailable",
                    "reason": "Single bridge configuration"
                }
            }
        }

    # ==================== SLAM 3D Mesh Endpoints ====================
    # These endpoints stream nvblox 3D mesh data for Mission Planner visualization
    # Mesh data is received from ros_http_bridge running inside the Isaac ROS container
    
    @app.post("/api/task/2/slam/mesh/update", tags=["Task 2", "SLAM"])
    async def update_slam_mesh(request: Request):
        """
        Receive mesh update from ros_http_bridge (internal use).
        
        This endpoint receives mesh data from the ros_http_bridge running
        inside the Isaac ROS container. The mesh data is stored and served
        to Mission Planner via the GET /api/task/2/slam/mesh endpoint.
        
        Posted by: ros_http_bridge.py (inside Isaac ROS container)
        """
        try:
            mesh_data = await request.json()
            
            # Store in app state
            if not hasattr(request.app.state, 'slam_mesh_data'):
                request.app.state.slam_mesh_data = {}
            
            request.app.state.slam_mesh_data = {
                "mesh": mesh_data,
                "received_at": datetime.now(timezone.utc).isoformat(),
                "block_count": len(mesh_data.get("blocks", [])),
                "total_blocks": mesh_data.get("total_blocks", len(mesh_data.get("blocks", []))),
                "mode": mesh_data.get("mode", "blocks"),
            }
            
            # Store drone pose from mesh data (from TF lookup in ros_http_bridge)
            if "drone_position" in mesh_data and mesh_data["drone_position"]:
                request.app.state.slam_mesh_data["drone_position"] = mesh_data["drone_position"]
            if "drone_attitude" in mesh_data and mesh_data["drone_attitude"]:
                request.app.state.slam_mesh_data["drone_attitude"] = mesh_data["drone_attitude"]
            
            return {"status": "ok", "blocks_received": len(mesh_data.get("blocks", []))}
            
        except Exception as e:
            logger.error(f"SLAM mesh update error: {e}")
            raise HTTPException(status_code=400, detail=str(e))

    @app.get("/api/task/2/slam/mesh", tags=["Task 2", "SLAM"])
    async def get_slam_mesh(request: Request, format: str = Query("full", description="'full' or 'summary'")):
        """
        Get current 3D SLAM mesh from nvblox.
        
        This endpoint returns the real-time 3D occupancy map built by nvblox
        from ZED camera depth data. Used by Mission Planner for 3D visualization.
        
        Mesh data is received from ros_http_bridge running inside the Isaac ROS
        container via POST /api/task/2/slam/mesh/update.
        
        Args:
            format: 'full' for complete mesh data, 'summary' for metadata only
        
        Returns:
            - mesh: The mesh data with vertices, triangles, and optional colors
            - drone_position: Current VIO position
            - drone_attitude: Current VIO orientation (roll, pitch, yaw)
            - timestamp: ISO format timestamp
            
        Update Rate: Target 2 Hz for streaming
        """
        try:
            # Check for stored mesh data from ros_http_bridge
            if hasattr(request.app.state, 'slam_mesh_data') and request.app.state.slam_mesh_data:
                stored = request.app.state.slam_mesh_data
                
                if format == "summary":
                    result = {
                        "available": True,
                        "timestamp": stored.get("received_at"),
                        "block_count": stored.get("block_count", 0),
                        "total_blocks": stored.get("total_blocks", 0),
                        "mode": stored.get("mode", "blocks"),
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
                
                # Fallback to external VIO state if mesh didn't include pose
                if "drone_position" not in result:
                    external_vio = request.app.state.external_vio_state
                    if external_vio:
                        result["drone_position"] = {
                            "x": external_vio.get("x", 0),
                            "y": external_vio.get("y", 0),
                            "z": external_vio.get("z", 0),
                        }
                        result["drone_attitude"] = {
                            "roll": external_vio.get("roll", 0),
                            "pitch": external_vio.get("pitch", 0),
                            "yaw": external_vio.get("yaw", 0),
                        }
                
                return result
            
            # Fallback: try ros_mesh_bridge (for when running natively with ROS2)
            try:
                from .ros_mesh_bridge import get_mesh_bridge
                
                bridge = get_mesh_bridge()
                if not bridge or not bridge.is_available():
                    # Return empty mesh with error status
                    return {
                        "available": False,
                        "error": "Mesh bridge not initialized or nvblox not running",
                        "timestamp": datetime.now(timezone.utc).isoformat(),
                        "mesh": None,
                        "drone_position": None,
                        "drone_attitude": None,
                    }
                
                if format == "summary":
                    summary = bridge.get_mesh_summary()
                    if not summary:
                        return {
                            "available": True,
                            "error": "No mesh data available yet",
                            "timestamp": datetime.now(timezone.utc).isoformat(),
                        }
                    
                    # Add drone pose to summary
                    external_vio = request.app.state.external_vio_state
                    if external_vio:
                        summary["drone_position"] = {
                            "x": external_vio.get("x", 0),
                            "y": external_vio.get("y", 0),
                            "z": external_vio.get("z", 0),
                        }
                        summary["drone_attitude"] = {
                            "roll": external_vio.get("roll", 0),
                            "pitch": external_vio.get("pitch", 0),
                            "yaw": external_vio.get("yaw", 0),
                        }
                    return summary
                
                # Full mesh data
                mesh = bridge.get_latest_mesh()
                if not mesh:
                    return {
                        "available": True,
                        "error": "No mesh data available yet",
                        "timestamp": datetime.now(timezone.utc).isoformat(),
                        "mesh": None,
                    }
                
                # Get drone position from VIO
                external_vio = request.app.state.external_vio_state
                drone_position = None
                drone_attitude = None
                
                if external_vio:
                    drone_position = {
                        "x": external_vio.get("x", 0),
                        "y": external_vio.get("y", 0),
                        "z": external_vio.get("z", 0),
                    }
                    drone_attitude = {
                        "roll": external_vio.get("roll", 0),
                        "pitch": external_vio.get("pitch", 0),
                        "yaw": external_vio.get("yaw", 0),
                    }
                
                return {
                    "available": True,
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                    "mesh": mesh,
                    "drone_position": drone_position,
                    "drone_attitude": drone_attitude,
                }
            
            except ImportError:
                return {
                    "available": False,
                    "error": "Mesh bridge module not available",
                    "timestamp": datetime.now(timezone.utc).isoformat(),
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
        
        More efficient than full mesh for continuous updates.
        Only returns mesh blocks that have changed since the last delta request.
        
        Returns:
            - changed_blocks: List of mesh blocks that changed
            - timestamp: When the delta was generated
            - block_count: Number of changed blocks
        """
        try:
            from .ros_mesh_bridge import get_mesh_bridge
            
            bridge = get_mesh_bridge()
            if not bridge or not bridge.is_available():
                return {
                    "available": False,
                    "error": "Mesh bridge not initialized",
                }
            
            delta = bridge.get_mesh_delta()
            if not delta:
                return {
                    "available": True,
                    "has_changes": False,
                    "changed_blocks": [],
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                }
            
            # Add drone position
            external_vio = request.app.state.external_vio_state
            if external_vio:
                delta["drone_position"] = {
                    "x": external_vio.get("x", 0),
                    "y": external_vio.get("y", 0),
                    "z": external_vio.get("z", 0),
                }
                delta["drone_attitude"] = {
                    "roll": external_vio.get("roll", 0),
                    "pitch": external_vio.get("pitch", 0),
                    "yaw": external_vio.get("yaw", 0),
                }
            
            delta["available"] = True
            delta["has_changes"] = True
            return delta
            
        except Exception as e:
            logger.error(f"SLAM mesh delta error: {e}")
            return {"available": False, "error": str(e)}

    @app.get("/api/task/2/slam/status", tags=["Task 2", "SLAM"])
    async def get_slam_status(request: Request):
        """
        Get nvblox SLAM system status.
        
        Returns status information about the 3D mapping pipeline.
        """
        # Check for stored mesh data from ros_http_bridge first
        if hasattr(request.app.state, 'slam_mesh_data') and request.app.state.slam_mesh_data:
            stored = request.app.state.slam_mesh_data
            return {
                "available": True,
                "running": True,
                "source": "ros_http_bridge",
                "block_count": stored.get("block_count", 0),
                "total_vertices": stored.get("total_vertices", 0),
                "total_triangles": stored.get("total_triangles", 0),
                "last_update": stored.get("received_at"),
            }
        
        # Fallback: try ros_mesh_bridge
        try:
            from .ros_mesh_bridge import get_mesh_bridge
            
            bridge = get_mesh_bridge()
            if not bridge:
                return {
                    "available": False,
                    "running": False,
                    "error": "No mesh data available - nvblox may not be running",
                }
            
            status = bridge.get_status()
            return {
                "available": bridge.is_available(),
                "running": bridge.is_running(),
                "source": "ros_mesh_bridge",
                **status,
            }
        except ImportError:
            return {
                "available": False,
                "running": False,
                "error": "No mesh data available",
            }
        except Exception as e:
            return {
                "available": False,
                "running": False,
                "error": str(e),
            }

    @app.post("/api/task/2/slam/clear", tags=["Task 2", "SLAM"])
    async def clear_slam_mesh(request: Request):
        """
        Clear the current SLAM mesh.
        
        This clears the cached mesh data. Note: This does NOT clear the
        nvblox map itself - use the nvblox reset service for that.
        
        Instead of nulling slam_mesh_data (which causes the GET endpoint
        to fall through to the unavailable ros_mesh_bridge fallback),
        we replace it with a valid cleared state containing an empty
        block list and clear=True so clients can clear their local caches.
        """
        # Preserve block_size from the previous mesh data if available
        prev_block_size = 0.05
        if (hasattr(request.app.state, 'slam_mesh_data')
                and isinstance(request.app.state.slam_mesh_data, dict)):
            prev_mesh = request.app.state.slam_mesh_data.get("mesh")
            if isinstance(prev_mesh, dict):
                prev_block_size = prev_mesh.get("block_size", 0.05)
        
        # Replace with a cleared-but-valid state so the GET endpoint
        # still returns available=True and clients see clear=True
        request.app.state.slam_mesh_data = {
            "mesh": {
                "blocks": [],
                "block_size": prev_block_size,
                "total_blocks": 0,
                "mode": "blocks",
                "clear": True,
            },
            "received_at": datetime.now(timezone.utc).isoformat(),
            "block_count": 0,
            "total_blocks": 0,
            "mode": "blocks",
        }
        
        # Also try to clear ros_mesh_bridge if available
        try:
            from .ros_mesh_bridge import get_mesh_bridge
            
            bridge = get_mesh_bridge()
            if bridge:
                bridge.clear_mesh()
        except:
            pass
        
        return {
            "success": True,
            "message": "Mesh cache cleared",
            "timestamp": datetime.now(timezone.utc).isoformat(),
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
        results = {
            "success": True,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "steps": [],
        }
        
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
            results["steps"].append({
                "step": "git stash",
                "success": stash_result.returncode == 0,
                "output": stash_result.stdout.strip(),
                "error": stash_result.stderr.strip() if stash_result.returncode != 0 else None,
            })
            
            # Step 2: git pull
            pull_result = subprocess.run(
                ["git", "pull", "origin", "main"],
                cwd=nomad_dir,
                capture_output=True,
                text=True,
                timeout=60,
            )
            results["steps"].append({
                "step": "git pull origin main",
                "success": pull_result.returncode == 0,
                "output": pull_result.stdout.strip(),
                "error": pull_result.stderr.strip() if pull_result.returncode != 0 else None,
            })
            
            if pull_result.returncode != 0:
                results["success"] = False
                results["error"] = "Git pull failed"
                return results
            
            # Step 3: chmod +x scripts
            chmod_result = subprocess.run(
                ["chmod", "+x", "scripts/*.sh"],
                cwd=nomad_dir,
                shell=True,
                capture_output=True,
                text=True,
                timeout=10,
            )
            # Try with bash -c for glob expansion (recursive)
            chmod_result = subprocess.run(
                ["bash", "-c", "find scripts -name '*.sh' -exec chmod +x {} +"],
                cwd=nomad_dir,
                capture_output=True,
                text=True,
                timeout=10,
            )
            results["steps"].append({
                "step": "chmod +x scripts/**/*.sh",
                "success": chmod_result.returncode == 0,
                "output": "Scripts made executable" if chmod_result.returncode == 0 else chmod_result.stdout.strip(),
                "error": chmod_result.stderr.strip() if chmod_result.returncode != 0 else None,
            })
            
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
                "changes": status_result.stdout.strip().split("\n") if status_result.stdout.strip() else [],
                "timestamp": datetime.now(timezone.utc).isoformat(),
            }
            
        except Exception as e:
            return {
                "error": str(e),
                "branch": "unknown",
                "commit": "unknown",
                "has_changes": False,
            }

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
                    "servos": {}
                }
            
            status = controller.get_status()
            status["available"] = True
            return status
            
        except ImportError:
            return {
                "available": False,
                "error": "Servo controller module not available",
                "servos": {}
            }
        except Exception as e:
            logger.error(f"Servo status error: {e}")
            return {
                "available": False,
                "error": str(e),
                "servos": {}
            }
    
    @app.post("/api/servo/camera/tilt", tags=["Servo"])
    async def set_camera_tilt(angle: float = Query(..., ge=0, le=180, description="Tilt angle 0-180 degrees")):
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
                raise HTTPException(status_code=503, detail="Servo controller not available")
            
            success = controller.set_camera_tilt(angle)
            
            if success:
                return {
                    "status": "ok",
                    "angle": angle,
                    "message": f"Camera tilt set to {angle} degrees"
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
    async def trigger_water_shooter(duration_ms: int = Query(200, ge=50, le=2000, description="Trigger duration in milliseconds")):
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
                raise HTTPException(status_code=503, detail="Servo controller not available")
            
            success = controller.trigger_water_shooter(duration_ms)
            
            if success:
                return {
                    "status": "ok",
                    "duration_ms": duration_ms,
                    "message": f"Water shooter triggered for {duration_ms}ms"
                }
            else:
                raise HTTPException(status_code=500, detail="Failed to trigger water shooter")
                
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
                raise HTTPException(status_code=503, detail="Servo controller not available")
            
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
                raise HTTPException(status_code=503, detail="Servo controller not available")
            
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
        channel: int = Query(..., ge=1, le=18, description="RC channel number (1-18)")
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
                raise HTTPException(status_code=503, detail="RC servo bridge not initialized")
            
            bridge.set_channel(channel)
            return {
                "status": "ok",
                "rc_channel": channel,
                "message": f"RC servo bridge now using channel {channel}"
            }
            
        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Set RC channel error: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    return app
