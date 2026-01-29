"""
NOMAD Edge Core - REST API

Provides HTTP endpoints for system status, health monitoring,
Task 1/Task 2 operations, and video streaming.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

import asyncio
import logging
import os
import re
import subprocess
from datetime import datetime, timezone
from typing import TYPE_CHECKING, Any, Optional

import cv2
import numpy as np

from fastapi import FastAPI, HTTPException, Request, WebSocket, Query
from fastapi.encoders import jsonable_encoder
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse, FileResponse
from fastapi.websockets import WebSocketDisconnect
from pydantic import BaseModel

from .state import StateManager

if TYPE_CHECKING:
    from .health_monitor import JetsonHealthMonitor
    from .zed_camera import ZEDCameraService
    from .vio_pipeline import VIOPipeline
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
    image_name: Optional[str] = None
    error: Optional[str] = None


class Task2ResetRequest(BaseModel):
    """Request model for Task 2 reset."""
    confirm: bool = False


class Task2HitRequest(BaseModel):
    """Request model for Task 2 target hit."""
    x: float
    y: float
    z: float


# Whitelist of allowed terminal commands for safety
COMMAND_WHITELIST: dict[str, str] = {
    "restart_video": "sudo systemctl restart mediamtx",
    "restart_edge_core": "sudo systemctl restart nomad",
    "reboot_jetson": "sudo reboot",
    "shutdown_jetson": "sudo shutdown -h now",
    "check_disk": "df -h",
    "check_memory": "free -h",
    "check_processes": "ps aux | head -20",
    "tailscale_status": "tailscale status",
    "network_info": "ip addr show",
    "gpu_status": "tegrastats --interval 1000 --stop 2",
}


class TerminalCommandRequest(BaseModel):
    """Request model for terminal command execution."""
    command_name: str  # Must be a key in COMMAND_WHITELIST
    timeout: int = 10


class TerminalCommandResponse(BaseModel):
    """Response model for terminal command."""
    success: bool
    stdout: str
    stderr: str
    return_code: int
    command_executed: Optional[str] = None


class VIOStatusResponse(BaseModel):
    """Response model for VIO status."""
    health: str
    tracking_confidence: float
    position_valid: bool
    message_rate_hz: float
    reset_counter: int


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


def set_camera_service(app: FastAPI, camera: "ZEDCameraService") -> None:
    """Register camera service with API via app.state."""
    app.state.camera_service = camera


def set_vio_pipeline(app: FastAPI, pipeline: "VIOPipeline") -> None:
    """Register VIO pipeline with API via app.state."""
    app.state.vio_pipeline = pipeline


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
    
    # Enable CORS for Mission Planner plugin access
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    
    # Initialize app.state with all service references (dependency injection)
    app.state.state_manager = state_manager
    app.state.health_monitor = None
    app.state.camera_service = None
    app.state.vio_pipeline = None
    app.state.isaac_bridge = None
    app.state.nav_controller = None
    app.state.tailscale_manager = None
    app.state.network_monitor = None
    
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
                "terminal": "/api/terminal/exec",
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
        
        # Add VIO health if available
        vio_pipeline = request.app.state.vio_pipeline
        if vio_pipeline:
            vio_status = vio_pipeline.status
            response["vio"] = {
                "health": vio_status.health.value,
                "tracking_confidence": vio_status.tracking_confidence,
                "message_rate_hz": vio_status.message_rate_hz,
            }
        
        return response

    @app.get("/health/detailed", tags=["System"])
    async def detailed_health(request: Request):
        """Get detailed health metrics for monitoring dashboard."""
        health_monitor = request.app.state.health_monitor
        if not health_monitor:
            return {"error": "Health monitor not initialized"}
        
        return health_monitor.health.to_dict()

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
                vio_pipeline = websocket.app.state.vio_pipeline
                if health_monitor:
                    data["jetson_health"] = health_monitor.health.to_dict()
                if vio_pipeline:
                    data["vio_status"] = vio_pipeline.status.to_dict()
                
                await websocket.send_json(data)
                await asyncio.sleep(0.1)  # 10Hz
        except WebSocketDisconnect:
            return

    # ==================== Task 1: Recon (Outdoor) ====================

    @app.post("/api/task/1/capture", tags=["Task 1"], response_model=Task1CaptureResponse)
    async def task1_capture(task_request: Task1CaptureRequest = None, request: Request = None):
        """
        Capture snapshot for Task 1 recon mission.
        
        Captures current position, heading, and camera image.
        Used for outdoor GPS-based reconnaissance.
        """
        state = request.app.state.state_manager.get_state()
        
        # Get values from request or current state
        heading = task_request.heading_deg if task_request and task_request.heading_deg else state.heading_deg
        gimbal_pitch = task_request.gimbal_pitch_deg if task_request and task_request.gimbal_pitch_deg else state.gimbal_pitch_deg
        
        # Validate we have required data
        if not state.gps_fix:
            return Task1CaptureResponse(
                success=False,
                timestamp=datetime.now(timezone.utc).isoformat(),
                error="No GPS fix - cannot capture position"
            )
        
        # Create capture record
        timestamp = datetime.now(timezone.utc)
        capture = {
            "timestamp": timestamp.isoformat(),
            "position": {
                "lat": state.gps_lat,
                "lon": state.gps_lon,
                "alt": state.gps_alt,
            },
            "heading_deg": heading,
            "gimbal_pitch_deg": gimbal_pitch,
        }
        
        # Save to mission log
        log_dir = os.environ.get("NOMAD_LOG_DIR", "./data/mission_logs")
        os.makedirs(log_dir, exist_ok=True)
        
        log_file = os.path.join(log_dir, f"task1_{timestamp.strftime('%Y%m%d_%H%M%S')}.json")
        
        image_filename = None
        
        # Capture image from ZED camera if available
        camera_service = request.app.state.camera_service
        if camera_service:
            try:
                frame = camera_service.get_frame()
                if frame and frame.left_image is not None:
                    # Create image directory
                    image_dir = "./data/task1_images"
                    os.makedirs(image_dir, exist_ok=True)
                    
                    # Generate filename
                    image_filename = f"task1_{timestamp.strftime('%Y%m%d_%H%M%S_%f')[:-3]}.jpg"
                    image_path = os.path.join(image_dir, image_filename)
                    
                    # Convert from BGRA to BGR if needed (ZED often returns BGRA)
                    if frame.left_image.shape[2] == 4:
                        image_bgr = cv2.cvtColor(frame.left_image, cv2.COLOR_BGRA2BGR)
                    else:
                        image_bgr = frame.left_image
                    
                    # Save image
                    cv2.imwrite(image_path, image_bgr)
                    logger.info(f"Task 1 image saved: {image_path}")
                    
                    # Add image reference to capture record
                    capture["image_file"] = image_filename
                else:
                    logger.warning("Task 1 capture: Camera frame not available")
            except Exception as e:
                logger.error(f"Task 1 image capture failed: {e}")
        else:
            logger.warning("Task 1 capture: Camera service not available")
        
        try:
            import json
            with open(log_file, "w") as f:
                json.dump(capture, f, indent=2)
            
            logger.info(f"Task 1 capture saved: {log_file}")
            
            return Task1CaptureResponse(
                success=True,
                timestamp=timestamp.isoformat(),
                target_text=f"Captured at {state.gps_lat:.6f}, {state.gps_lon:.6f}",
                position=capture["position"],
                heading_deg=heading,
                image_name=image_filename,
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
        Retrieve a saved Task 1 image.
        
        Returns the image file captured during a Task 1 recon mission.
        """
        image_dir = "./data/task1_images"
        image_path = os.path.join(image_dir, filename)
        
        # Validate filename to prevent directory traversal
        if ".." in filename or "/" in filename or "\\" in filename:
            raise HTTPException(status_code=400, detail="Invalid filename")
        
        # Check if file exists
        if not os.path.exists(image_path):
            raise HTTPException(status_code=404, detail="Image not found")
        
        return FileResponse(image_path, media_type="image/jpeg")

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
        # Check for external VIO state first
        external_vio_state = request.app.state.external_vio_state
        if external_vio_state:
            return {
                "health": "healthy" if external_vio_state.get("confidence", 0) > 0.5 else "degraded",
                "tracking_confidence": external_vio_state.get("confidence", 0),
                "position_valid": True,
                "message_rate_hz": 30.0,
                "reset_counter": 0,
                "source": external_vio_state.get("source", "external"),
            }
        
        vio_pipeline = request.app.state.vio_pipeline
        if not vio_pipeline:
            return {
                "health": "unknown",
                "tracking_confidence": 0,
                "position_valid": False,
                "message_rate_hz": 0,
                "reset_counter": 0,
                "source": "none",
            }
        
        return vio_pipeline.status.to_dict()

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
        
        vio_pipeline = request.app.state.vio_pipeline
        if not vio_pipeline:
            # Just clear trajectory if no VIO pipeline
            return {
                "success": True,
                "reset_counter": 0,
                "message": "Trajectory cleared (no VIO pipeline)",
            }
        
        success = vio_pipeline.reset_origin()
        return {
            "success": success,
            "reset_counter": vio_pipeline.status.reset_counter,
        }

    @app.get("/api/vio/calibration", tags=["VIO"])
    async def vio_calibration_status(request: Request):
        """Get VIO calibration validation results."""
        camera_service = request.app.state.camera_service
        if not camera_service:
            raise HTTPException(status_code=503, detail="Camera service not initialized")
        
        from .vio_pipeline import VIOCalibration
        results = VIOCalibration.validate_tracking(camera_service, duration_s=3.0)
        return results

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
        vio_pipeline = request.app.state.vio_pipeline
        
        if external_vio_state:
            services["vio"] = {
                "status": "active",
                "running": True,
                "source": external_vio_state.get("source", "external"),
                "confidence": external_vio_state.get("confidence", 0),
                "trajectory_points": len(vio_trajectory),
            }
        elif vio_pipeline:
            vio_status = vio_pipeline.status
            services["vio"] = {
                "status": vio_status.health.value,
                "running": vio_status.health.value != "failed",
                "confidence": vio_status.tracking_confidence,
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
                ["docker", "ps", "--filter", "ancestor=isaac_ros_dev-aarch64", "--format", "{{.Status}}"],
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
        try:
            result = subprocess.run(
                ["docker", "ps", "--filter", "name=nomad_isaac_ros", "--format", "{{.Status}}"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            container_running = bool(result.stdout.strip())
        except Exception:
            pass
        
        isaac_bridge = request.app.state.isaac_bridge
        if not isaac_bridge:
            return {
                "available": False,
                "backend": "not_initialized",
                "container_running": container_running,
                "message": "Isaac ROS bridge not initialized - using direct ZED mode",
            }
        
        return {
            "available": True,
            "container_running": container_running,
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
        script_path = os.path.expanduser("~/NOMAD/scripts/start_isaac_ros_auto.sh")
        
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
        script_path = os.path.expanduser("~/NOMAD/scripts/start_isaac_ros_auto.sh")
        
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

    # ==================== Video Manager Endpoints ====================
    from .video_manager import get_video_manager
    from pydantic import BaseModel, Field
    
    class StartStreamRequest(BaseModel):
        stream_name: str = Field(..., description="Unique name for the stream (e.g., 'zed_left')")
        topic: str = Field(..., description="ROS image topic to subscribe to")
        width: int = Field(1280, description="Output video width", ge=320, le=3840)
        height: int = Field(720, description="Output video height", ge=240, le=2160)
        fps: int = Field(30, description="Output video FPS", ge=1, le=60)
    
    class SwitchTopicRequest(BaseModel):
        instance: str = Field("primary", description="Bridge instance (primary or secondary)")
        topic: str = Field(..., description="ROS image topic to switch to")
    
    @app.get("/api/video/topics", tags=["Video"])
    async def get_video_topics():
        """
        List available ROS image topics.
        
        Returns a list of all sensor_msgs/Image topics currently published in ROS.
        Use this to discover available camera feeds before switching streams.
        """
        mgr = get_video_manager()
        return {"topics": mgr.list_topics()}

    @app.get("/api/video/streams", tags=["Video"])
    async def list_video_streams():
        """
        List all active video streams (persistent bridges).
        
        Returns information about each stream including:
        - Stream name and RTSP URL
        - Current ROS topic
        - Resolution and FPS
        - Overlay status
        """
        mgr = get_video_manager()
        return {"streams": mgr.list_streams()}

    @app.get("/api/video/streams/{stream_name}", tags=["Video"])
    async def get_video_stream(stream_name: str):
        """Get detailed information about a specific stream."""
        mgr = get_video_manager()
        stream = mgr.get_stream(stream_name)
        if not stream:
            raise HTTPException(status_code=404, detail=f"Stream '{stream_name}' not found")
        return stream

    @app.post("/api/video/streams", tags=["Video"])
    async def create_video_stream(request: StartStreamRequest):
        """
        Start or update a video stream (legacy endpoint).
        
        In the new architecture, this maps stream names to persistent bridges:
        - 'zed', 'primary', 'live' -> switches primary bridge topic
        - 'secondary', 'depth' -> switches secondary bridge topic
        
        The RTSP URL stays constant - only the content changes.
        """
        try:
            mgr = get_video_manager()
            stream = mgr.start_stream(
                stream_name=request.stream_name,
                topic=request.topic,
                width=request.width,
                height=request.height,
                fps=request.fps
            )
            return {"success": True, "stream": stream}
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))
        except Exception as e:
            logger.error(f"Failed to start stream: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @app.delete("/api/video/streams/{stream_name}", tags=["Video"])
    async def delete_video_stream(stream_name: str):
        """
        Stop a specific video stream (legacy - bridges are now persistent).
        
        In the new architecture, this endpoint is a no-op since bridges
        run continuously. The RTSP connection stays alive.
        """
        mgr = get_video_manager()
        success = mgr.stop_stream(stream_name)
        return {"success": True, "message": f"Bridges are now persistent - '{stream_name}' continues running"}

    @app.delete("/api/video/streams", tags=["Video"])
    async def delete_all_video_streams():
        """Stop all video streams (legacy - bridges are now persistent)."""
        mgr = get_video_manager()
        count = mgr.stop_all_streams()
        return {"success": True, "message": "Bridges are now persistent and continue running"}

    @app.post("/api/video/switch", tags=["Video"])
    async def switch_video_topic(request: SwitchTopicRequest):
        """
        Switch a bridge instance to a different ROS topic.
        
        This is the primary endpoint for dynamic stream switching from Mission Planner.
        The RTSP URL stays constant, only the content changes - no reconnect needed.
        
        Args:
            instance: Bridge instance name (primary, secondary)
            topic: New ROS image topic to subscribe to
            
        Example:
            POST /api/video/switch
            {"instance": "primary", "topic": "/zed/zed_node/depth/depth_registered"}
        
        The Mission Planner plugin should:
        1. Keep the same RTSP URL (rtsp://<ip>:8554/primary)
        2. Call this endpoint to change what's shown on that stream
        3. The video player continues playing - no reconnect needed
        """
        mgr = get_video_manager()
        success = mgr.switch_video_source(request.instance, request.topic)
        stream = mgr.get_stream(request.instance)
        
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to switch topic for '{request.instance}'")
        
        return {
            "success": True,
            "instance": request.instance,
            "topic": stream.get("topic") if stream else request.topic,
            "rtsp_url": stream.get("rtsp_url") if stream else f"rtsp://localhost:8554/{request.instance}"
        }

    @app.post("/api/video/overlay", tags=["Video"])
    async def set_video_overlay(
        enabled: bool = Query(True, description="Enable or disable overlay"),
        instance: str = Query("primary", description="Bridge instance")
    ):
        """
        Enable or disable object detection overlay for a bridge instance.
        
        When enabled, the bridge draws bounding boxes from ZED object detection
        on the video stream.
        """
        mgr = get_video_manager()
        success = mgr.set_overlay_enabled(instance, enabled)
        
        return {
            "success": success,
            "instance": instance,
            "overlay_enabled": enabled
        }

    @app.get("/api/video/bridges", tags=["Video"])
    async def get_video_bridges():
        """
        Get status of all persistent video bridges.
        
        Returns detailed status from each bridge's HTTP control API including:
        - Current topic and state
        - Frame count and error statistics
        - Overlay status
        """
        mgr = get_video_manager()
        bridges = {}
        
        for instance in ["primary", "secondary"]:
            status = mgr.get_bridge_status(instance)
            if status:
                bridges[instance] = status
            else:
                stream = mgr.get_stream(instance)
                bridges[instance] = stream if stream else {"state": "unknown"}
        
        return {"bridges": bridges}

    @app.post("/api/video/bridges/start", tags=["Video"])
    async def start_video_bridges():
        """
        Start the persistent video bridge instances.
        
        This is typically called automatically on startup, but can be used
        to manually start bridges if they weren't auto-started.
        """
        mgr = get_video_manager()
        results = mgr.start_persistent_bridges()
        
        return {
            "success": all(results.values()),
            "results": results,
            "streams": mgr.list_streams()
        }

    @app.post("/api/video/source", tags=["Video"])
    async def switch_video_source(topic: str = Query(..., description="ROS image topic to stream")):
        """
        Switch the primary video stream to a different ROS topic.
        
        This endpoint enables dynamic stream switching from Mission Planner:
        The RTSP URL stays constant (rtsp://<ip>:8554/primary), only content changes.
        
        In the new architecture, this is equivalent to:
            POST /api/video/switch {"instance": "primary", "topic": "<topic>"}
        
        Example:
            POST /api/video/source?topic=/zed/zed_node/left/image_rect_color
        """
        mgr = get_video_manager()
        success = mgr.switch_video_source("primary", topic)
        stream = mgr.get_stream("primary")
        
        if not success:
            raise HTTPException(status_code=500, detail="Failed to switch video source")
        
        return {
            "success": True,
            "topic": topic,
            "rtsp_url": stream.get("rtsp_url") if stream else "rtsp://localhost:8554/primary",
            "stream": stream
        }

    @app.get("/api/video/source", tags=["Video"])
    async def get_video_source():
        """
        Get the current active video source (primary stream).
        
        Returns information about the currently streaming topic
        and the constant RTSP URL.
        """
        mgr = get_video_manager()
        stream = mgr.get_stream("primary")
        if not stream:
            return {"active": False, "stream": None}
        return {"active": True, "stream": stream}

    @app.get("/api/video/encoding", tags=["Video"])
    async def get_video_encoding():
        """
        Get the current video encoding mode.
        
        In the new zero-copy architecture, NVENC hardware encoding is always used.
        """
        return {
            "use_nvenc": True,
            "encoder": "nvv4l2h264enc (NVENC hardware)",
            "architecture": "Zero-copy GStreamer pipeline",
            "description": "Hardware encoding with ~150ms glass-to-glass latency"
        }

    @app.post("/api/video/encoding", tags=["Video"])
    async def set_video_encoding(use_nvenc: bool = Query(True, description="Use NVENC hardware encoding")):
        """
        Set the video encoding mode (legacy endpoint).
        
        In the new zero-copy architecture, NVENC is always used.
        This endpoint is kept for backward compatibility.
        """
        return {
            "success": True,
            "use_nvenc": True,
            "encoder": "nvv4l2h264enc (NVENC hardware)",
            "note": "NVENC is always enabled in zero-copy architecture"
        }

    return app
