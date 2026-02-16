"""
NOMAD Edge Core - ZED 2i Camera Interface

Provides camera streaming and Visual Inertial Odometry (VIO) integration
for the ZED 2i stereo camera on Jetson Orin Nano.

Target: Python 3.13 | NVIDIA Jetson Orin Nano | ZED SDK 4.x
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Optional, Tuple

import numpy as np

logger = logging.getLogger("edge_core.zed_camera")


class ZEDResolution(Enum):
    """Supported ZED camera resolutions."""
    HD2K = "HD2K"      # 2208x1242
    HD1080 = "HD1080"  # 1920x1080
    HD720 = "HD720"    # 1280x720 (recommended for Task 1)
    VGA = "VGA"        # 672x376


class ZEDTrackingState(Enum):
    """ZED positional tracking states."""
    OFF = 0
    OK = 1
    SEARCHING = 2
    FPS_TOO_LOW = 3


@dataclass
class ZEDPose:
    """
    Pose data from ZED positional tracking.
    
    Coordinates are in camera frame (right-handed):
    - X: Right
    - Y: Down
    - Z: Forward
    
    For ArduPilot, we convert to NED frame:
    - North: Z (Forward)
    - East: X (Right)
    - Down: Y (Down)
    """
    timestamp_us: int
    position: Tuple[float, float, float]  # (x, y, z) in meters
    orientation: Tuple[float, float, float, float]  # quaternion (x, y, z, w)
    euler: Tuple[float, float, float]  # (roll, pitch, yaw) in radians
    velocity: Tuple[float, float, float]  # (vx, vy, vz) in m/s
    tracking_state: ZEDTrackingState
    confidence: float  # 0-100

    def to_ned(self) -> Tuple[float, float, float, float, float, float]:
        """
        Convert ZED pose to NED (North-East-Down) frame for ArduPilot.
        
        Returns:
            Tuple of (north, east, down, roll, pitch, yaw)
        """
        x, y, z = self.position
        roll, pitch, yaw = self.euler
        
        # ZED to NED conversion:
        # NED North = ZED Z (forward)
        # NED East = ZED X (right)
        # NED Down = ZED Y (down)
        north = z
        east = x
        down = y
        
        # Euler angles: ZED uses right-handed, same convention
        # Roll, Pitch, Yaw are compatible
        return (north, east, down, roll, pitch, yaw)


@dataclass
class ZEDFrame:
    """A captured frame from the ZED camera."""
    timestamp_us: int
    left_image: Optional[np.ndarray] = None
    right_image: Optional[np.ndarray] = None
    depth_map: Optional[np.ndarray] = None
    point_cloud: Optional[np.ndarray] = None


@dataclass
class ZEDConfig:
    """Configuration for ZED camera."""
    resolution: ZEDResolution = ZEDResolution.HD720
    fps: int = 30
    depth_mode: str = "ULTRA"  # NONE, PERFORMANCE, QUALITY, ULTRA
    enable_tracking: bool = True
    enable_depth: bool = True
    enable_spatial_mapping: bool = False
    coordinate_system: str = "RIGHT_HANDED_Z_UP_X_FWD"
    serial_number: Optional[int] = None  # Auto-detect if None


class ZEDCameraService:
    """
    ZED 2i Camera Service for NOMAD.
    
    Provides:
    - Camera initialization and configuration
    - Image capture and streaming
    - Positional tracking (VIO) for Task 2
    - Depth sensing
    - RTSP stream publishing via MediaMTX
    
    Usage:
        service = ZEDCameraService(config)
        service.start()
        
        # Get latest pose for VIO
        pose = service.get_pose()
        
        # Get latest frame
        frame = service.get_frame()
        
        service.stop()
    """
    
    def __init__(
        self,
        config: ZEDConfig = None,
        on_pose_update: Optional[Callable[[ZEDPose], None]] = None,
        on_frame_update: Optional[Callable[[ZEDFrame], None]] = None,
    ):
        self._config = config or ZEDConfig()
        self._on_pose_update = on_pose_update
        self._on_frame_update = on_frame_update
        
        self._zed = None
        self._runtime_params = None
        
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.RLock()
        
        self._latest_pose: Optional[ZEDPose] = None
        self._latest_frame: Optional[ZEDFrame] = None
        
        self._is_initialized = False
        self._tracking_enabled = False
        
        # Performance metrics
        self._fps_counter = 0
        self._fps_timestamp = time.time()
        self._current_fps = 0.0
        
    @property
    def is_initialized(self) -> bool:
        """Check if camera is initialized."""
        return self._is_initialized
    
    @property
    def is_tracking(self) -> bool:
        """Check if positional tracking is active."""
        return self._tracking_enabled
    
    @property
    def current_fps(self) -> float:
        """Get current camera FPS."""
        return self._current_fps

    @property
    def zed_handle(self):
        """Get the raw pyzed.sl.Camera handle for direct sensor access (e.g. calibration)."""
        return self._zed if self._is_initialized else None

    def start(self) -> bool:
        """
        Start the ZED camera service.
        
        Returns:
            True if started successfully
        """
        if self._thread and self._thread.is_alive():
            logger.warning("ZED camera service already running")
            return True
            
        try:
            if not self._initialize_camera():
                return False
                
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
            
            logger.info("ZED camera service started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start ZED camera: {e}")
            return False
    
    def stop(self) -> None:
        """Stop the ZED camera service."""
        self._stop_event.set()
        
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
            
        self._close_camera()
        logger.info("ZED camera service stopped")
    
    def get_pose(self) -> Optional[ZEDPose]:
        """Get the latest pose from positional tracking."""
        with self._lock:
            return self._latest_pose
    
    def get_frame(self) -> Optional[ZEDFrame]:
        """Get the latest captured frame."""
        with self._lock:
            return self._latest_frame
    
    def reset_tracking(self) -> bool:
        """
        Reset positional tracking origin.
        
        Call this when the drone is at a known position to reset
        the VIO coordinate system.
        
        Returns:
            True if reset successful
        """
        if not self._is_initialized or not self._zed:
            return False
            
        try:
            # Import here to avoid issues if SDK not installed
            import pyzed.sl as sl
            
            # Create identity transform at current position
            transform = sl.Transform()
            transform.set_identity()
            
            self._zed.reset_positional_tracking(transform)
            logger.info("Positional tracking reset")
            return True
            
        except Exception as e:
            logger.error(f"Failed to reset tracking: {e}")
            return False
    
    def enable_tracking(self, enable: bool = True) -> bool:
        """Enable or disable positional tracking."""
        if not self._is_initialized or not self._zed:
            return False
            
        try:
            import pyzed.sl as sl
            
            if enable and not self._tracking_enabled:
                tracking_params = sl.PositionalTrackingParameters()
                tracking_params.enable_area_memory = True
                tracking_params.enable_pose_smoothing = True
                
                status = self._zed.enable_positional_tracking(tracking_params)
                if status == sl.ERROR_CODE.SUCCESS:
                    self._tracking_enabled = True
                    logger.info("Positional tracking enabled")
                    return True
                else:
                    logger.error(f"Failed to enable tracking: {status}")
                    return False
                    
            elif not enable and self._tracking_enabled:
                self._zed.disable_positional_tracking()
                self._tracking_enabled = False
                logger.info("Positional tracking disabled")
                return True
                
            return True
            
        except Exception as e:
            logger.error(f"Tracking toggle error: {e}")
            return False
    
    def _initialize_camera(self) -> bool:
        """Initialize the ZED camera."""
        try:
            import pyzed.sl as sl
            
            self._zed = sl.Camera()
            
            # Set initialization parameters
            init_params = sl.InitParameters()
            
            # Resolution
            res_map = {
                ZEDResolution.HD2K: sl.RESOLUTION.HD2K,
                ZEDResolution.HD1080: sl.RESOLUTION.HD1080,
                ZEDResolution.HD720: sl.RESOLUTION.HD720,
                ZEDResolution.VGA: sl.RESOLUTION.VGA,
            }
            init_params.camera_resolution = res_map.get(
                self._config.resolution, sl.RESOLUTION.HD720
            )
            init_params.camera_fps = self._config.fps
            
            # Depth mode
            depth_map = {
                "NONE": sl.DEPTH_MODE.NONE,
                "PERFORMANCE": sl.DEPTH_MODE.PERFORMANCE,
                "QUALITY": sl.DEPTH_MODE.QUALITY,
                "ULTRA": sl.DEPTH_MODE.ULTRA,
            }
            init_params.depth_mode = depth_map.get(
                self._config.depth_mode, sl.DEPTH_MODE.ULTRA
            )
            
            # Coordinate system for ArduPilot compatibility
            init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
            init_params.coordinate_units = sl.UNIT.METER
            
            # Serial number (optional)
            if self._config.serial_number:
                init_params.set_from_serial_number(self._config.serial_number)
            
            # Open camera
            status = self._zed.open(init_params)
            if status != sl.ERROR_CODE.SUCCESS:
                logger.error(f"Failed to open ZED camera: {status}")
                return False
            
            # Log camera info
            info = self._zed.get_camera_information()
            logger.info(f"ZED camera opened: {info.camera_model}, SN: {info.serial_number}")
            logger.info(f"Resolution: {info.camera_configuration.resolution}")
            logger.info(f"FPS: {info.camera_configuration.fps}")
            
            # Enable positional tracking if configured
            if self._config.enable_tracking:
                tracking_params = sl.PositionalTrackingParameters()
                tracking_params.enable_area_memory = True
                tracking_params.enable_pose_smoothing = True
                tracking_params.set_floor_as_origin = False
                
                status = self._zed.enable_positional_tracking(tracking_params)
                if status == sl.ERROR_CODE.SUCCESS:
                    self._tracking_enabled = True
                    logger.info("Positional tracking enabled")
                else:
                    logger.warning(f"Failed to enable tracking: {status}")
            
            # Setup runtime parameters
            self._runtime_params = sl.RuntimeParameters()
            self._runtime_params.enable_depth = self._config.enable_depth
            self._runtime_params.confidence_threshold = 50
            self._runtime_params.texture_confidence_threshold = 100
            
            self._is_initialized = True
            return True
            
        except ImportError:
            logger.error("ZED SDK (pyzed) not installed. Install with: pip install pyzed")
            return False
        except Exception as e:
            logger.error(f"Camera initialization error: {e}")
            return False
    
    def _close_camera(self) -> None:
        """Close the ZED camera."""
        if self._zed:
            try:
                if self._tracking_enabled:
                    self._zed.disable_positional_tracking()
                self._zed.close()
            except Exception as e:
                logger.error(f"Error closing camera: {e}")
            finally:
                self._zed = None
                self._is_initialized = False
                self._tracking_enabled = False
    
    def _run(self) -> None:
        """Main camera loop."""
        try:
            import pyzed.sl as sl
            
            # Pre-allocate objects
            image_left = sl.Mat()
            image_right = sl.Mat()
            depth = sl.Mat()
            pose = sl.Pose()
            
            while not self._stop_event.is_set():
                # Grab frame
                if self._zed.grab(self._runtime_params) == sl.ERROR_CODE.SUCCESS:
                    timestamp_us = self._zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_microseconds()
                    
                    # Get images
                    self._zed.retrieve_image(image_left, sl.VIEW.LEFT)
                    
                    # Build frame
                    frame = ZEDFrame(
                        timestamp_us=timestamp_us,
                        left_image=image_left.get_data().copy(),
                    )
                    
                    # Get depth if enabled
                    if self._config.enable_depth:
                        self._zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                        frame.depth_map = depth.get_data().copy()
                    
                    # Get pose if tracking
                    if self._tracking_enabled:
                        state = self._zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)
                        
                        translation = pose.get_translation().get()
                        orientation = pose.get_orientation().get()
                        euler = pose.get_euler_angles()
                        
                        # Calculate velocity (simplified)
                        velocity = (0.0, 0.0, 0.0)  # Would need previous pose for real velocity
                        
                        tracking_state_map = {
                            sl.POSITIONAL_TRACKING_STATE.OFF: ZEDTrackingState.OFF,
                            sl.POSITIONAL_TRACKING_STATE.OK: ZEDTrackingState.OK,
                            sl.POSITIONAL_TRACKING_STATE.SEARCHING: ZEDTrackingState.SEARCHING,
                            sl.POSITIONAL_TRACKING_STATE.FPS_TOO_LOW: ZEDTrackingState.FPS_TOO_LOW,
                        }
                        
                        zed_pose = ZEDPose(
                            timestamp_us=timestamp_us,
                            position=(translation[0], translation[1], translation[2]),
                            orientation=(orientation[0], orientation[1], orientation[2], orientation[3]),
                            euler=(euler[0], euler[1], euler[2]),
                            velocity=velocity,
                            tracking_state=tracking_state_map.get(state, ZEDTrackingState.OFF),
                            confidence=pose.pose_confidence,
                        )
                        
                        with self._lock:
                            self._latest_pose = zed_pose
                        
                        if self._on_pose_update:
                            self._on_pose_update(zed_pose)
                    
                    # Update latest frame
                    with self._lock:
                        self._latest_frame = frame
                    
                    if self._on_frame_update:
                        self._on_frame_update(frame)
                    
                    # FPS calculation
                    self._fps_counter += 1
                    now = time.time()
                    if now - self._fps_timestamp >= 1.0:
                        self._current_fps = self._fps_counter / (now - self._fps_timestamp)
                        self._fps_counter = 0
                        self._fps_timestamp = now
                        
                else:
                    time.sleep(0.001)
                    
        except Exception as e:
            logger.error(f"Camera loop error: {e}")
