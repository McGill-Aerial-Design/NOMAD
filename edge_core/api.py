"""
NOMAD Edge Core - REST API

Provides HTTP endpoints for system status, health monitoring,
Task 1/Task 2 operations, and video streaming.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

import asyncio
import logging
import os
import subprocess
from datetime import datetime, timezone
from typing import TYPE_CHECKING, Any, Optional

from fastapi import FastAPI, HTTPException, WebSocket, Query
from fastapi.encoders import jsonable_encoder
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from fastapi.websockets import WebSocketDisconnect
from pydantic import BaseModel

from .state import StateManager

if TYPE_CHECKING:
    from .health_monitor import JetsonHealthMonitor
    from .zed_camera import ZEDCameraService
    from .vio_pipeline import VIOPipeline
    from .isaac_ros_bridge import IsaacROSBridge

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
    error: Optional[str] = None


class Task2ResetRequest(BaseModel):
    """Request model for Task 2 reset."""
    confirm: bool = False


class Task2HitRequest(BaseModel):
    """Request model for Task 2 target hit."""
    x: float
    y: float
    z: float


class TerminalCommandRequest(BaseModel):
    """Request model for terminal command execution."""
    command: str
    timeout: int = 10


class TerminalCommandResponse(BaseModel):
    """Response model for terminal command."""
    success: bool
    stdout: str
    stderr: str
    return_code: int


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


# ==================== Global Service References ====================
# These are set by main.py after initialization

_health_monitor: Optional["JetsonHealthMonitor"] = None
_camera_service: Optional["ZEDCameraService"] = None
_vio_pipeline: Optional["VIOPipeline"] = None
_isaac_bridge: Optional["IsaacROSBridge"] = None
_exclusion_map: list[dict] = []

# VIO state from external sources (ROS bridge)
_external_vio_state: Optional[dict] = None
_vio_trajectory: list[dict] = []  # List of {x, y, z, timestamp} points
_vio_trajectory_max_points: int = 1000  # Keep last N points


def set_health_monitor(monitor: "JetsonHealthMonitor") -> None:
    """Set the health monitor reference."""
    global _health_monitor
    _health_monitor = monitor


def set_camera_service(camera: "ZEDCameraService") -> None:
    """Set the camera service reference."""
    global _camera_service
    _camera_service = camera


def set_vio_pipeline(pipeline: "VIOPipeline") -> None:
    """Set the VIO pipeline reference."""
    global _vio_pipeline
    _vio_pipeline = pipeline


def set_isaac_bridge(bridge: "IsaacROSBridge") -> None:
    """Set the Isaac ROS bridge reference."""
    global _isaac_bridge
    _isaac_bridge = bridge


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
    async def health_check():
        """
        Comprehensive health check endpoint.
        
        Returns system health including CPU/GPU temperatures,
        memory usage, VIO status, and network connectivity.
        """
        state = state_manager.get_state()
        
        # Base health response
        response = {
            "status": "ok" if state.connected else "degraded",
            "connected": state.connected,
            "gps_fix": state.gps_fix,
            "flight_mode": state.flight_mode,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        
        # Add Jetson health metrics if available
        if _health_monitor:
            health = _health_monitor.health
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
        if _vio_pipeline:
            vio_status = _vio_pipeline.status
            response["vio"] = {
                "health": vio_status.health.value,
                "tracking_confidence": vio_status.tracking_confidence,
                "message_rate_hz": vio_status.message_rate_hz,
            }
        
        return response

    @app.get("/health/detailed", tags=["System"])
    async def detailed_health():
        """Get detailed health metrics for monitoring dashboard."""
        if not _health_monitor:
            return {"error": "Health monitor not initialized"}
        
        return _health_monitor.health.to_dict()

    # ==================== Status Endpoints ====================

    @app.get("/status", tags=["System"])
    async def get_status():
        """Get current system state including all telemetry."""
        state = state_manager.get_state()
        return jsonable_encoder(state)

    @app.websocket("/ws/state")
    async def ws_state(websocket: WebSocket):
        """WebSocket endpoint for real-time state updates (10Hz)."""
        await websocket.accept()
        try:
            while True:
                state = state_manager.get_state()
                data = jsonable_encoder(state)
                
                # Add additional real-time data
                if _health_monitor:
                    data["jetson_health"] = _health_monitor.health.to_dict()
                if _vio_pipeline:
                    data["vio_status"] = _vio_pipeline.status.to_dict()
                
                await websocket.send_json(data)
                await asyncio.sleep(0.1)  # 10Hz
        except WebSocketDisconnect:
            return

    # ==================== Task 1: Recon (Outdoor) ====================

    @app.post("/api/task/1/capture", tags=["Task 1"], response_model=Task1CaptureResponse)
    async def task1_capture(request: Task1CaptureRequest = None):
        """
        Capture snapshot for Task 1 recon mission.
        
        Captures current position, heading, and camera image.
        Used for outdoor GPS-based reconnaissance.
        """
        state = state_manager.get_state()
        
        # Get values from request or current state
        heading = request.heading_deg if request and request.heading_deg else state.heading_deg
        gimbal_pitch = request.gimbal_pitch_deg if request and request.gimbal_pitch_deg else state.gimbal_pitch_deg
        
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
            )
            
        except Exception as e:
            logger.error(f"Task 1 capture failed: {e}")
            return Task1CaptureResponse(
                success=False,
                timestamp=timestamp.isoformat(),
                error=str(e)
            )

    # ==================== Task 2: Extinguish (Indoor) ====================

    @app.post("/api/task/2/reset_map", tags=["Task 2"])
    async def task2_reset_map(request: Task2ResetRequest = None):
        """
        Reset the exclusion map for Task 2.
        
        Clears all recorded target positions, allowing
        previously sprayed targets to be detected again.
        """
        global _exclusion_map
        
        _exclusion_map = []
        logger.info("Task 2 exclusion map reset")
        
        return {
            "success": True,
            "message": "Exclusion map cleared",
            "total_targets": 0,
        }

    @app.post("/api/task/2/target_hit", tags=["Task 2"])
    async def task2_target_hit(request: Task2HitRequest):
        """
        Register a target hit for Task 2 exclusion map.
        
        Records the 3D position of a sprayed target to prevent
        re-engagement. Uses VIO frame coordinates.
        """
        global _exclusion_map
        
        target = {
            "x": request.x,
            "y": request.y,
            "z": request.z,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        
        _exclusion_map.append(target)
        logger.info(f"Task 2 target hit registered: ({request.x}, {request.y}, {request.z})")
        
        return {
            "success": True,
            "target": target,
            "total_targets": len(_exclusion_map),
        }

    @app.get("/api/task/2/exclusion_map", tags=["Task 2"])
    async def task2_get_exclusion_map():
        """Get current exclusion map targets."""
        return {
            "total_targets": len(_exclusion_map),
            "targets": _exclusion_map,
        }

    # ==================== VIO Endpoints ====================

    @app.get("/api/vio/status", tags=["VIO"])
    async def vio_status():
        """Get VIO pipeline status."""
        # Check for external VIO state first
        if _external_vio_state:
            return {
                "health": "healthy" if _external_vio_state.get("confidence", 0) > 0.5 else "degraded",
                "tracking_confidence": _external_vio_state.get("confidence", 0),
                "position_valid": True,
                "message_rate_hz": 30.0,
                "reset_counter": 0,
                "source": _external_vio_state.get("source", "external"),
            }
        
        if not _vio_pipeline:
            return {
                "health": "unknown",
                "tracking_confidence": 0,
                "position_valid": False,
                "message_rate_hz": 0,
                "reset_counter": 0,
                "source": "none",
            }
        
        return _vio_pipeline.status.to_dict()

    @app.post("/api/vio/update", tags=["VIO"])
    async def vio_update(request: VIOUpdateRequest):
        """
        Receive VIO pose update from external source (ROS bridge).
        
        This endpoint is called by the ros_http_bridge.py script running
        inside the Isaac ROS container to send VIO data to edge_core.
        """
        global _external_vio_state, _vio_trajectory
        
        # Store latest state
        _external_vio_state = {
            "timestamp": request.timestamp,
            "x": request.x,
            "y": request.y,
            "z": request.z,
            "roll": request.roll,
            "pitch": request.pitch,
            "yaw": request.yaw,
            "vx": request.vx,
            "vy": request.vy,
            "vz": request.vz,
            "confidence": request.confidence,
            "source": request.source,
        }
        
        # Add to trajectory
        _vio_trajectory.append({
            "x": request.x,
            "y": request.y,
            "z": request.z,
            "timestamp": request.timestamp,
        })
        
        # Trim trajectory if too long
        if len(_vio_trajectory) > _vio_trajectory_max_points:
            _vio_trajectory = _vio_trajectory[-_vio_trajectory_max_points:]
        
        return {"success": True, "trajectory_points": len(_vio_trajectory)}

    @app.get("/api/vio/pose", tags=["VIO"])
    async def vio_pose():
        """Get current VIO pose (position and orientation)."""
        if _external_vio_state:
            return _external_vio_state
        
        if _isaac_bridge and _isaac_bridge.vio_state:
            vio = _isaac_bridge.vio_state
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
    async def vio_trajectory(max_points: int = Query(default=100, le=1000)):
        """
        Get VIO trajectory for visualization.
        
        Returns a list of (x, y, z) points representing the drone's path.
        Use max_points to limit the response size.
        """
        points = _vio_trajectory[-max_points:] if _vio_trajectory else []
        return {
            "total_points": len(_vio_trajectory),
            "returned_points": len(points),
            "trajectory": points,
        }

    @app.delete("/api/vio/trajectory", tags=["VIO"])
    async def vio_clear_trajectory():
        """Clear the VIO trajectory history."""
        global _vio_trajectory
        count = len(_vio_trajectory)
        _vio_trajectory = []
        return {"success": True, "cleared_points": count}

    @app.post("/api/vio/reset_origin", tags=["VIO"])
    async def vio_reset_origin():
        """Reset VIO tracking origin to current position."""
        global _vio_trajectory
        
        # Clear trajectory on reset
        _vio_trajectory = []
        
        if not _vio_pipeline:
            # Just clear trajectory if no VIO pipeline
            return {
                "success": True,
                "reset_counter": 0,
                "message": "Trajectory cleared (no VIO pipeline)",
            }
        
        success = _vio_pipeline.reset_origin()
        return {
            "success": success,
            "reset_counter": _vio_pipeline.status.reset_counter,
        }

    @app.get("/api/vio/calibration", tags=["VIO"])
    async def vio_calibration_status():
        """Get VIO calibration validation results."""
        if not _camera_service:
            raise HTTPException(status_code=503, detail="Camera service not initialized")
        
        from .vio_pipeline import VIOCalibration
        results = VIOCalibration.validate_tracking(_camera_service, duration_s=3.0)
        return results

    # ==================== Camera Endpoints ====================

    @app.get("/api/camera/status", tags=["Camera"])
    async def camera_status():
        """Get ZED camera status."""
        if not _camera_service:
            return {
                "initialized": False,
                "tracking": False,
                "fps": 0,
            }
        
        return {
            "initialized": _camera_service.is_initialized,
            "tracking": _camera_service.is_tracking,
            "fps": _camera_service.current_fps,
        }

    @app.post("/api/camera/reset_tracking", tags=["Camera"])
    async def camera_reset_tracking():
        """Reset camera positional tracking."""
        if not _camera_service:
            raise HTTPException(status_code=503, detail="Camera not initialized")
        
        success = _camera_service.reset_tracking()
        return {"success": success}

    # ==================== Terminal Endpoints ====================

    @app.post("/api/terminal/exec", tags=["Terminal"], response_model=TerminalCommandResponse)
    async def execute_terminal_command(request: TerminalCommandRequest):
        """
        Execute a shell command on the Jetson.
        
        WARNING: This is a powerful endpoint. Use with caution.
        Only enabled in debug mode by default.
        
        Common uses:
        - System diagnostics
        - Network troubleshooting
        - Service management
        """
        debug_mode = os.environ.get("NOMAD_DEBUG", "false").lower() == "true"
        
        if not debug_mode:
            # In production, only allow specific safe commands
            allowed_prefixes = [
                "tailscale",
                "systemctl status",
                "journalctl",
                "df -h",
                "free -m",
                "uptime",
                "hostname",
                "ip addr",
                "ping -c",
                "cat /proc/loadavg",
                "tegrastats",
            ]
            
            if not any(request.command.startswith(prefix) for prefix in allowed_prefixes):
                return TerminalCommandResponse(
                    success=False,
                    stdout="",
                    stderr="Command not allowed in production mode",
                    return_code=-1,
                )
        
        try:
            result = subprocess.run(
                request.command,
                shell=True,
                capture_output=True,
                text=True,
                timeout=request.timeout,
            )
            
            return TerminalCommandResponse(
                success=result.returncode == 0,
                stdout=result.stdout,
                stderr=result.stderr,
                return_code=result.returncode,
            )
            
        except subprocess.TimeoutExpired:
            return TerminalCommandResponse(
                success=False,
                stdout="",
                stderr=f"Command timed out after {request.timeout}s",
                return_code=-1,
            )
        except Exception as e:
            return TerminalCommandResponse(
                success=False,
                stdout="",
                stderr=str(e),
                return_code=-1,
            )

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
    async def services_status():
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
        if _isaac_bridge:
            services["isaac_ros"] = {
                "status": "active",
                "running": True,
                **_isaac_bridge.get_status(),
            }
        else:
            services["isaac_ros"] = {
                "status": "not_initialized",
                "running": False,
                "message": "Isaac ROS bridge not enabled",
            }
        
        # VIO status
        if _external_vio_state:
            services["vio"] = {
                "status": "active",
                "running": True,
                "source": _external_vio_state.get("source", "external"),
                "confidence": _external_vio_state.get("confidence", 0),
                "trajectory_points": len(_vio_trajectory),
            }
        elif _vio_pipeline:
            vio_status = _vio_pipeline.status
            services["vio"] = {
                "status": vio_status.health.value,
                "running": vio_status.health.value != "failed",
                "confidence": vio_status.tracking_confidence,
                "trajectory_points": len(_vio_trajectory),
            }
        else:
            services["vio"] = {
                "status": "not_initialized",
                "running": False,
                "trajectory_points": len(_vio_trajectory),
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
            "primary_stream": f"{rtsp_base}/live",
            "secondary_stream": f"{rtsp_base}/gimbal",
            "format": "H.264/RTSP",
            "recommended_player": "VLC or FFplay",
        }

    # ==================== Isaac ROS Bridge Endpoints ====================

    @app.get("/api/isaac/status", tags=["Isaac ROS"])
    async def isaac_status():
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
        
        if not _isaac_bridge:
            return {
                "available": False,
                "backend": "not_initialized",
                "container_running": container_running,
                "message": "Isaac ROS bridge not initialized - using direct ZED mode",
            }
        
        return {
            "available": True,
            "container_running": container_running,
            **_isaac_bridge.get_status(),
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
    async def isaac_vio():
        """Get current VIO state from Isaac ROS VSLAM or ZED."""
        if not _isaac_bridge:
            raise HTTPException(status_code=503, detail="Isaac bridge not initialized")
        
        vio = _isaac_bridge.vio_state
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
    async def isaac_detections():
        """Get current YOLO detections from Isaac ROS."""
        if not _isaac_bridge:
            raise HTTPException(status_code=503, detail="Isaac bridge not initialized")
        
        detections = _isaac_bridge.detections
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
    async def isaac_exclusion_map():
        """Get exclusion map from Isaac ROS bridge (auto-managed)."""
        if not _isaac_bridge:
            # Fall back to local exclusion map
            return {
                "backend": "local",
                "total_targets": len(_exclusion_map),
                "targets": _exclusion_map,
            }
        
        exclusion = _isaac_bridge.exclusion_map
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
    async def isaac_add_exclusion(request: Task2HitRequest):
        """Add target to Isaac ROS managed exclusion map."""
        if _isaac_bridge:
            target_id = _isaac_bridge.add_to_exclusion_map(
                x=request.x, y=request.y, z=request.z
            )
            return {"success": True, "target_id": target_id}
        else:
            # Fall back to local
            _exclusion_map.append({
                "x": request.x,
                "y": request.y,
                "z": request.z,
                "timestamp": datetime.now(timezone.utc).isoformat(),
            })
            return {"success": True, "target_id": f"local_{len(_exclusion_map)}"}

    @app.post("/api/isaac/exclusion_map/clear", tags=["Isaac ROS"])
    async def isaac_clear_exclusion():
        """Clear Isaac ROS managed exclusion map."""
        global _exclusion_map
        
        if _isaac_bridge:
            count = _isaac_bridge.clear_exclusion_map()
            return {"success": True, "cleared": count}
        else:
            count = len(_exclusion_map)
            _exclusion_map = []
            return {"success": True, "cleared": count}

    return app
