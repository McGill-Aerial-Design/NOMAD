"""
NOMAD Edge Core - ZED 2i Camera Interface

Provides camera streaming and Visual Inertial Odometry (VIO) integration
for the ZED 2i stereo camera on Jetson Orin Nano.

Target: Python 3.13 | NVIDIA Jetson Orin Nano | ZED SDK 5.2.3
"""

from __future__ import annotations

import asyncio
import logging
import os
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
    
    Coordinates use the configured ZED SDK world frame
    RIGHT_HANDED_Z_UP_X_FWD (right-handed):
    - X: Forward
    - Y: Left
    - Z: Up
    
    For ArduPilot, we convert to NED frame:
    - North: X (Forward)
    - East: -Y (Right)
    - Down: -Z (Down)
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
        # ZED RIGHT_HANDED_Z_UP_X_FWD: X-forward, Y-left, Z-up
        # NED North = ZED X (forward)
        # NED East = -ZED Y (right)
        # NED Down = -ZED Z (down)
        north = x
        east = -y
        down = -z
        
        # Euler angles are passed through as provided by the SDK.
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
        config: Optional[ZEDConfig] = None,
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
        self._is_degraded = False
        self._degraded_reason: Optional[str] = None
        
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
    def is_degraded(self) -> bool:
        """Whether the camera service is in a degraded state requiring restart."""
        return self._is_degraded

    @property
    def degraded_reason(self) -> Optional[str]:
        """Human-readable degraded-state reason, when available."""
        return self._degraded_reason

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

            with self._lock:
                self._is_degraded = False
                self._degraded_reason = None
                
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
            if self._thread.is_alive():
                self._mark_degraded("stop_timeout")
                logger.warning(
                    "ZED worker thread still active after stop timeout; attempting guarded forced-close and marking service degraded"
                )
                forced_close_ok = self._guarded_force_close()
                if forced_close_ok:
                    logger.warning("Guarded forced-close completed while worker thread remained active")
                    self._thread.join(timeout=0.25)
                    if self._thread and not self._thread.is_alive():
                        self._thread = None
                else:
                    logger.error("Guarded forced-close failed; manual restart is required")
                return
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
            
            status = self._zed.reset_positional_tracking(transform)
            if status != sl.ERROR_CODE.SUCCESS:
                logger.error(f"Failed to reset tracking: {status}")
                return False

            logger.info("Positional tracking reset")
            return True
            
        except Exception as e:
            logger.error(f"Failed to reset tracking: {e}")
            return False

    def save_area_map(
        self,
        file_path: str,
        wait_for_completion: bool = True,
        timeout_s: float = 30.0,
    ) -> Tuple[bool, str]:
        """Save positional tracking area map for later relocalization."""
        if not self._is_initialized or not self._zed:
            return False, "Camera service is not initialized"
        if not self._tracking_enabled:
            return False, "Positional tracking must be enabled before saving an area map"
        if not file_path:
            return False, "file_path is required"

        map_path = os.path.abspath(file_path)
        try:
            os.makedirs(os.path.dirname(map_path) or ".", exist_ok=True)
        except Exception as e:
            return False, f"Failed to prepare area map directory: {e}"

        try:
            import pyzed.sl as sl

            status = self._zed.save_area_map(map_path)
            if status != sl.ERROR_CODE.SUCCESS:
                message = f"Failed to start area map export: {status}"
                logger.error(message)
                return False, message

            if not wait_for_completion:
                return True, f"Area map export started: {map_path}"

            if not hasattr(self._zed, "get_area_export_state"):
                return True, f"Area map export requested: {map_path}"

            deadline = time.time() + max(timeout_s, 0.0)
            while time.time() <= deadline:
                export_state = self._zed.get_area_export_state()
                export_state_text = str(export_state).upper()
                if "SUCCESS" in export_state_text:
                    logger.info(f"Area map saved: {map_path}")
                    return True, f"Area map saved: {map_path}"
                if "FAIL" in export_state_text or "ERROR" in export_state_text:
                    message = f"Area map export failed: {export_state}"
                    logger.error(message)
                    return False, message
                time.sleep(0.1)

            message = f"Timed out waiting for area map export after {timeout_s:.1f}s"
            logger.error(message)
            return False, message

        except Exception as e:
            logger.error(f"Failed to save area map: {e}")
            return False, f"Failed to save area map: {e}"

    def load_area_map(self, file_path: str) -> Tuple[bool, str]:
        """Load an area map and re-enable tracking for relocalization."""
        if not self._is_initialized or not self._zed:
            return False, "Camera service is not initialized"
        if not file_path:
            return False, "file_path is required"

        map_path = os.path.abspath(file_path)
        if not os.path.isfile(map_path):
            return False, f"Area map file not found: {map_path}"

        was_tracking_enabled = self._tracking_enabled

        try:
            import pyzed.sl as sl

            if was_tracking_enabled:
                self._zed.disable_positional_tracking()
                self._tracking_enabled = False

            tracking_params = sl.PositionalTrackingParameters()
            tracking_params.enable_area_memory = True
            tracking_params.enable_pose_smoothing = True
            tracking_params.set_floor_as_origin = False
            tracking_params.area_file_path = map_path

            status = self._zed.enable_positional_tracking(tracking_params)
            if status != sl.ERROR_CODE.SUCCESS:
                if was_tracking_enabled and not self._tracking_enabled:
                    if self.enable_tracking(True):
                        logger.warning("Restored positional tracking after area map load failure")
                    else:
                        logger.error("Failed to restore positional tracking after area map load failure")
                message = f"Failed to load area map for relocalization: {status}"
                logger.error(message)
                return False, message

            self._tracking_enabled = True
            logger.info(f"Area map loaded for relocalization: {map_path}")
            return True, f"Area map loaded for relocalization: {map_path}"

        except Exception as e:
            if was_tracking_enabled and not self._tracking_enabled:
                if self.enable_tracking(True):
                    logger.warning("Restored positional tracking after area map load exception")
                else:
                    logger.error("Failed to restore positional tracking after area map load exception")
            logger.error(f"Failed to load area map: {e}")
            return False, f"Failed to load area map: {e}"
    
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
            
            # Resolve coordinate system from configuration (validated against SDK enum).
            coordinate_system_name = (self._config.coordinate_system or "").strip().upper()
            try:
                init_params.coordinate_system = getattr(
                    sl.COORDINATE_SYSTEM,
                    coordinate_system_name,
                )
            except AttributeError:
                valid_coordinate_systems = [
                    name for name in dir(sl.COORDINATE_SYSTEM) if name.isupper()
                ]
                logger.error(
                    "Invalid ZED coordinate system '%s' in ZEDConfig.coordinate_system. Valid values: %s",
                    self._config.coordinate_system,
                    ", ".join(valid_coordinate_systems),
                )
                return False
            if coordinate_system_name != "RIGHT_HANDED_Z_UP_X_FWD":
                logger.warning(
                    "Configured coordinate system '%s'; ZEDPose.to_ned assumes RIGHT_HANDED_Z_UP_X_FWD.",
                    coordinate_system_name,
                )
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

    def _mark_degraded(self, reason: str) -> None:
        with self._lock:
            self._is_degraded = True
            self._degraded_reason = reason

    def _guarded_force_close(self) -> bool:
        """Best-effort close when stop timeout occurs and worker thread is still active."""
        lock_acquired = self._lock.acquire(timeout=0.25)
        if not lock_acquired:
            return False
        try:
            self._close_camera()
            return self._zed is None and not self._is_initialized
        finally:
            self._lock.release()
    
    def _close_camera(self) -> None:
        """Close the ZED camera."""
        with self._lock:
            if self._zed:
                try:
                    if self._tracking_enabled:
                        self._zed.disable_positional_tracking()
                    self._zed.close()
                except Exception as e:
                    logger.error(f"Error closing camera: {e}")
            self._zed = None
            self._is_initialized = False
            self._tracking_enabled = False
            self._latest_pose = None
            self._latest_frame = None
            self._current_fps = 0.0
    
    def _run(self) -> None:
        """Main camera loop."""
        try:
            import pyzed.sl as sl
            
            # Pre-allocate objects
            image_left = sl.Mat()
            depth = sl.Mat()
            pose = sl.Pose()
            prev_pose_timestamp_us: Optional[int] = None
            prev_translation: Optional[Tuple[float, float, float]] = None
            min_dt_s = 1e-6
            
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
                        
                        current_position = (translation[0], translation[1], translation[2])
                        velocity = (0.0, 0.0, 0.0)
                        if prev_pose_timestamp_us is not None and prev_translation is not None:
                            dt_s = (timestamp_us - prev_pose_timestamp_us) / 1_000_000.0
                            if dt_s > min_dt_s:
                                velocity = (
                                    (current_position[0] - prev_translation[0]) / dt_s,
                                    (current_position[1] - prev_translation[1]) / dt_s,
                                    (current_position[2] - prev_translation[2]) / dt_s,
                                )
                        prev_pose_timestamp_us = timestamp_us
                        prev_translation = current_position
                        
                        tracking_state_map = {
                            sl.POSITIONAL_TRACKING_STATE.OFF: ZEDTrackingState.OFF,
                            sl.POSITIONAL_TRACKING_STATE.OK: ZEDTrackingState.OK,
                            sl.POSITIONAL_TRACKING_STATE.SEARCHING: ZEDTrackingState.SEARCHING,
                            sl.POSITIONAL_TRACKING_STATE.FPS_TOO_LOW: ZEDTrackingState.FPS_TOO_LOW,
                        }
                        
                        zed_pose = ZEDPose(
                            timestamp_us=timestamp_us,
                            position=current_position,
                            orientation=(orientation[0], orientation[1], orientation[2], orientation[3]),
                            euler=(euler[0], euler[1], euler[2]),
                            velocity=velocity,
                            tracking_state=tracking_state_map.get(state, ZEDTrackingState.OFF),
                            confidence=pose.pose_confidence,
                        )
                        
                        with self._lock:
                            self._latest_pose = zed_pose
                        
                        if self._on_pose_update:
                            try:
                                self._on_pose_update(zed_pose)
                            except Exception as callback_error:
                                logger.error(f"Pose update callback error: {callback_error}")
                    
                    # Update latest frame
                    with self._lock:
                        self._latest_frame = frame
                    
                    if self._on_frame_update:
                        try:
                            self._on_frame_update(frame)
                        except Exception as callback_error:
                            logger.error(f"Frame update callback error: {callback_error}")
                    
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
            try:
                self._close_camera()
            except Exception as close_error:
                logger.error(f"Camera loop cleanup error: {close_error}")
