"""
DEPRECATED: This module is deprecated as of January 2026.
VIO is now handled by edge_core/ros/vio_bridge.py running inside the Isaac ROS container.
The vio_bridge subscribes to /zed/zed_node/odom and publishes to /mavros/vision_pose/pose.

This file is kept for reference only.
See docs/NAV2_INTEGRATION_PLAN.md for the new architecture.
"""

"""
NOMAD Edge Core - VIO Pipeline Service

Visual Inertial Odometry pipeline that provides position feedback for
Jetson-centric indoor autonomous flight. VIO feeds ArduPilot's EKF3 for
state estimation while navigation commands come from the NavController.

Architecture (Jetson-Centric):
    ZED Camera -> VIO Pipeline -> VISION_POSITION_ESTIMATE -> ArduPilot EKF3
    Nav2/Nvblox -> NavController -> SET_POSITION_TARGET -> ArduPilot GUIDED

ArduPilot operates as a low-level flight controller. Jetson handles navigation.

Target: Python 3.13 | NVIDIA Jetson Orin Nano | ZED SDK 4.x
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

logger = logging.getLogger("edge_core.vio_pipeline")


class VIOHealth(Enum):
    """VIO pipeline health states."""
    UNKNOWN = "unknown"
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    FAILED = "failed"


@dataclass
class VIOStatus:
    """VIO pipeline status information."""
    health: VIOHealth = VIOHealth.UNKNOWN
    tracking_confidence: float = 0.0  # 0-100 scale (ZED SDK native)
    position_valid: bool = False
    velocity_valid: bool = False
    last_update_ms: int = 0
    message_rate_hz: float = 0.0
    ekf_healthy: bool = False
    ekf_variance_ok: bool = False
    reset_counter: int = 0
    error_message: Optional[str] = None
    
    def to_dict(self) -> dict:
        return {
            "health": self.health.value,
            "tracking_confidence": self.tracking_confidence,
            "position_valid": self.position_valid,
            "velocity_valid": self.velocity_valid,
            "last_update_ms": self.last_update_ms,
            "message_rate_hz": self.message_rate_hz,
            "ekf_healthy": self.ekf_healthy,
            "ekf_variance_ok": self.ekf_variance_ok,
            "reset_counter": self.reset_counter,
            "error_message": self.error_message,
        }


class VIOPipeline:
    """
    VIO Pipeline for NOMAD Task 2 Indoor Flight.
    
    This service provides position feedback to ArduPilot's EKF3 for state
    estimation. Navigation commands are handled separately by NavController.
    
    Responsibilities:
    1. Receives pose updates from ZED camera tracking
    2. Transforms coordinates to NED frame
    3. Sends VISION_POSITION_ESTIMATE to ArduPilot at 30Hz
    4. Monitors EKF health and VIO confidence
    5. Implements failsafe logic per PRD requirements
    6. Notifies NavController of VIO health for command validation
    
    PRD Requirements:
    - [T2-NAV-01]: Stream VISION_POSITION_ESTIMATE at 30Hz
    - [T2-NAV-02]: Configure EKF source to ExternalNav
    - [T2-NAV-03]: Jetson-centric navigation (see NavController)
    - [T2-SAFE-01]: VIO failure response within 3 seconds
    
    ARCHITECTURE NOTE:
    - VIO provides position FEEDBACK only
    - Navigation COMMANDS come from NavController (Isaac ROS nav2/nvblox)
    - ArduPilot is a flight controller, not a navigation planner
    
    SAFETY NOTES:
    - VIO can fail in feature-poor environments (white walls, smoke)
    - Optical flow sensor provides backup horizontal velocity estimation
    - ALT_HOLD mode is used as failsafe (not RTL which could hit walls)
    
    Usage:
        from edge_core.zed_camera import ZEDCameraService, ZEDConfig
        from edge_core.mavlink_interface import MavlinkService
        
        camera = ZEDCameraService(ZEDConfig(enable_tracking=True))
        mavlink = MavlinkService(state_manager)
        
        pipeline = VIOPipeline(camera, mavlink)
        pipeline.start()
    """
    
    # Target output rate to flight controller
    TARGET_RATE_HZ = 30.0
    
    # Health thresholds (ZED SDK confidence is 0-100 scale)
    MIN_CONFIDENCE = 30.0  # Minimum tracking confidence (0-100 scale)
    MIN_CONFIDENCE_DEGRADED = 15.0  # Below this, consider critically degraded (0-100 scale)
    MAX_STALE_MS = 100  # Maximum age of pose data
    VIO_FAILURE_TIMEOUT_S = 3.0  # Time before declaring VIO failed
    VIO_DEGRADED_TIMEOUT_S = 1.0  # Time before entering degraded state
    
    # EKF variance thresholds (based on EKF_STATUS_REPORT)
    EKF_POS_VARIANCE_THRESHOLD = 1.0
    EKF_VEL_VARIANCE_THRESHOLD = 1.0
    
    # Optical flow integration
    OPTICAL_FLOW_ENABLED = True  # Enable optical flow as backup
    OPTICAL_FLOW_MIN_QUALITY = 50  # Minimum quality for optical flow data
    
    # Failsafe configuration - rangefinder-assisted descent
    ALT_HOLD_MODE = 2  # ArduCopter ALT_HOLD mode ID
    FAILSAFE_DESCENT_RATE = 0.3  # m/s - slow controlled descent
    MAX_RANGEFINDER_ALTITUDE = 10.0  # meters - rangefinder effective range
    
    def __init__(
        self,
        camera_service,  # ZEDCameraService
        mavlink_service,  # MavlinkService
        state_manager=None,  # StateManager
        on_status_change: Optional[Callable[[VIOStatus], None]] = None,
        on_failsafe_trigger: Optional[Callable[[str], None]] = None,
        enable_optical_flow_backup: bool = True,  # Use optical flow when VIO degrades
    ):
        self._camera = camera_service
        self._mavlink = mavlink_service
        self._state_manager = state_manager
        self._on_status_change = on_status_change
        self._on_failsafe_trigger = on_failsafe_trigger
        self._enable_optical_flow = enable_optical_flow_backup
        
        self._status = VIOStatus()
        self._lock = threading.RLock()
        
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        
        # Tracking state
        self._reset_counter = 0
        self._last_healthy_time = 0.0
        self._last_degraded_time = 0.0
        self._failsafe_triggered = False
        self._degraded_warning_sent = False
        
        # Rate tracking
        self._message_count = 0
        self._rate_timestamp = time.time()
        
        # Optical flow backup state
        self._optical_flow_active = False
        self._last_optical_flow_quality = 0
        
    @property
    def status(self) -> VIOStatus:
        """Get current VIO status."""
        with self._lock:
            return self._status
    
    @property
    def is_healthy(self) -> bool:
        """Check if VIO is healthy."""
        with self._lock:
            return self._status.health == VIOHealth.HEALTHY
    
    def start(self) -> bool:
        """Start the VIO pipeline."""
        if self._thread and self._thread.is_alive():
            logger.warning("VIO pipeline already running")
            return True
            
        if not self._camera.is_tracking:
            logger.error("Camera tracking not enabled")
            return False
            
        self._stop_event.clear()
        self._failsafe_triggered = False
        self._last_healthy_time = time.time()
        
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        
        logger.info("VIO pipeline started")
        return True
    
    def stop(self) -> None:
        """Stop the VIO pipeline."""
        self._stop_event.set()
        
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
            
        logger.info("VIO pipeline stopped")
    
    def reset_origin(self) -> bool:
        """
        Reset VIO origin to current position.
        
        Call this when the drone is at a known starting position
        to reset the coordinate system.
        """
        if self._camera.reset_tracking():
            self._reset_counter += 1
            logger.info(f"VIO origin reset (counter={self._reset_counter})")
            return True
        return False
    
    def _run(self) -> None:
        """Main VIO processing loop."""
        interval = 1.0 / self.TARGET_RATE_HZ
        
        while not self._stop_event.is_set():
            try:
                start_time = time.time()
                
                self._process_pose()
                self._check_health()
                self._update_rate()
                
                # Maintain target rate
                elapsed = time.time() - start_time
                if elapsed < interval:
                    time.sleep(interval - elapsed)
                    
            except Exception as e:
                logger.error(f"VIO loop error: {e}")
                self._update_status(
                    health=VIOHealth.DEGRADED,
                    error_message=str(e),
                )
                time.sleep(0.01)
    
    def _process_pose(self) -> None:
        """Process latest pose and send to flight controller."""
        pose = self._camera.get_pose()
        
        if pose is None:
            self._update_status(
                position_valid=False,
                velocity_valid=False,
                tracking_confidence=0.0,
            )
            return
        
        # Check pose quality
        confidence = pose.confidence
        is_tracking = pose.tracking_state.value == 1  # OK state
        
        if not is_tracking or confidence < self.MIN_CONFIDENCE:
            self._update_status(
                position_valid=False,
                tracking_confidence=confidence,
            )
            return
        
        # Transform to NED frame
        north, east, down, roll, pitch, yaw = pose.to_ned()
        
        # Send VISION_POSITION_ESTIMATE to ArduPilot
        success = self._mavlink.send_vision_position_estimate(
            timestamp_us=pose.timestamp_us,
            x=north,
            y=east,
            z=down,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            reset_counter=self._reset_counter,
        )
        
        if success:
            self._message_count += 1
            self._last_healthy_time = time.time()
            
            self._update_status(
                health=VIOHealth.HEALTHY,
                position_valid=True,
                velocity_valid=True,
                tracking_confidence=confidence,
                last_update_ms=int(time.time() * 1000),
            )
        else:
            self._update_status(
                health=VIOHealth.DEGRADED,
                error_message="Failed to send VPE message",
            )
    
    def _check_health(self) -> None:
        """
        Check VIO health and trigger failsafe if needed.
        
        SAFETY: Implements progressive degradation:
        1. HEALTHY -> DEGRADED: Warn pilot, enable optical flow backup
        2. DEGRADED -> FAILED: Trigger failsafe (ALT_HOLD mode)
        
        This gives the pilot time to react before full failsafe,
        which is important in confined indoor spaces.
        """
        now = time.time()
        time_since_healthy = now - self._last_healthy_time
        
        # Check tracking confidence thresholds
        confidence = self._status.tracking_confidence
        
        # Progressive degradation states
        if confidence >= self.MIN_CONFIDENCE:
            # Good confidence - clear degraded state
            self._last_degraded_time = now
            self._degraded_warning_sent = False
            self._optical_flow_active = False
            
        elif confidence >= self.MIN_CONFIDENCE_DEGRADED:
            # Low confidence - entering degraded state
            if not self._degraded_warning_sent:
                logger.warning(f"VIO DEGRADED: Confidence at {confidence:.0f}%, enabling optical flow backup")
                self._degraded_warning_sent = True
                
                # Enable optical flow backup if available
                if self._enable_optical_flow:
                    self._enable_optical_flow_backup()
                    
            self._update_status(health=VIOHealth.DEGRADED)
            
        else:
            # Critical confidence - VIO essentially useless
            logger.error(f"VIO CRITICAL: Confidence at {confidence:.0f}%")
            self._update_status(
                health=VIOHealth.DEGRADED,
                error_message=f"Critical confidence: {confidence:.0f}%"
            )
        
        # Check for VIO failure timeout (PRD T2-SAFE-01)
        if time_since_healthy > self.VIO_FAILURE_TIMEOUT_S:
            if not self._failsafe_triggered:
                self._trigger_failsafe("VIO_TIMEOUT")
                
        elif time_since_healthy > self.VIO_DEGRADED_TIMEOUT_S:
            # Entering degraded state due to stale data
            self._update_status(health=VIOHealth.DEGRADED)
    
    def _enable_optical_flow_backup(self) -> None:
        """
        Enable optical flow sensor as backup position source.
        
        When VIO degrades, optical flow can provide horizontal velocity
        estimation to help maintain position hold. The Cube autopilot
        can fuse optical flow data with barometer for stable hover.
        
        SAFETY: Optical flow only provides horizontal velocity, not position.
        The drone should maintain altitude via barometer in this mode.
        """
        if self._optical_flow_active:
            return
            
        logger.info("Enabling optical flow sensor backup")
        self._optical_flow_active = True
        
        # Request EKF to include optical flow in fusion
        # This is done via MAVLink parameter: EK3_SRC1_VELXY = 5 (OpticalFlow)
        # For now, log the action - actual param change would need mavlink_interface
        logger.info("ACTION: Consider enabling EK3_SRC1_VELXY=5 for optical flow velocity")
        
        # Notify status change
        self._update_status(error_message="VIO degraded - optical flow backup active")
    
    def _check_rangefinder_healthy(self) -> tuple[bool, float]:
        """
        Check if rangefinder is providing valid data.
        
        Queries the system state for rangefinder altitude. Validates that
        the measurement is within the effective range and shows no sign of
        saturation or invalid data.
        
        Returns:
            Tuple of (is_healthy, altitude_m)
                - is_healthy: True if rangefinder provides usable data
                - altitude_m: Altitude reading in meters (0.0 if not healthy)
        """
        if not self._state_manager:
            return False, 0.0
        
        try:
            state = self._state_manager.get_state()
            
            # Check lidar_distance_m as a proxy for rangefinder altitude
            # (commonly available on vehicles with LiDAR/rangefinder sensors)
            rngfnd_alt = getattr(state, 'lidar_distance_m', None)
            
            # Validate rangefinder data:
            # - Not None (sensor present)
            # - Positive (valid measurement)
            # - Within effective range (0.1 to MAX_RANGEFINDER_ALTITUDE)
            if rngfnd_alt is not None:
                if 0.1 < rngfnd_alt < self.MAX_RANGEFINDER_ALTITUDE:
                    logger.debug(f"Rangefinder healthy: altitude={rngfnd_alt:.2f}m")
                    return True, rngfnd_alt
                elif rngfnd_alt >= self.MAX_RANGEFINDER_ALTITUDE:
                    logger.debug(f"Out of rangefinder range: altitude={rngfnd_alt:.2f}m (max={self.MAX_RANGEFINDER_ALTITUDE}m)")
                    return False, 0.0
            
            return False, 0.0
            
        except Exception as e:
            logger.debug(f"Rangefinder health check error: {e}")
            return False, 0.0
    
    def _command_descent(self, rate_mps: float) -> bool:
        """
        Command controlled descent velocity.
        
        Sends a vertical velocity command for controlled descent. This is safer
        than relying solely on barometer (which has poor performance indoors due
        to ground effect and HVAC). The descent rate is kept slow (0.3 m/s) to
        allow the pilot time to take control if needed.
        
        Uses existing NavController if available, otherwise falls back to
        direct MAVLink velocity commands.
        
        FRAME CONVENTIONS:
        - NavController: Body frame with Z-UP convention (vz positive = UP, so vz negative = DOWN)
        - MAVLink (NED): Global frame (vz positive = DOWN)
        
        Both paths will result in downward descent motion.
        
        Args:
            rate_mps: Descent rate in m/s (positive value = downward magnitude)
            
        Returns:
            True if descent command was sent successfully
        """
        try:
            # Clamp descent rate to safe value
            safe_rate = min(abs(rate_mps), 0.5)  # m/s down
            
            logger.info(f"Descending at {safe_rate:.2f} m/s (rangefinder-assisted)")
            
            # Try NavController first if available via state manager
            try:
                if self._state_manager is not None:
                    # NavController is stored in the app state via main.py;
                    # access it through the module-level nav_controller in main.
                    from . import main as main_module
                    nc = getattr(main_module, 'nav_controller', None)
                    if nc is not None:
                        # NavController uses body frame: positive vz = UP
                        # For descent, use negative vz (DOWN in body frame)
                        success = nc.send_velocity(
                            vx=0.0,
                            vy=0.0,
                            vz=-safe_rate,  # Negative = DOWN in body frame
                            yaw_rate=0.0,
                        )
                        if success:
                            logger.info("Descent command sent via NavController")
                            return True
            except (ImportError, AttributeError):
                pass
            
            # Fall back to direct MAVLink velocity command
            if hasattr(self._mavlink, 'send_velocity_command'):
                # MAVLink SET_POSITION_TARGET_LOCAL_NED uses NED frame: positive vz = DOWN
                # For descent, use positive vz (DOWN in NED frame)
                success = self._mavlink.send_velocity_command(
                    vx=0.0,
                    vy=0.0,
                    vz=safe_rate,  # Positive = DOWN in NED frame
                    yaw_rate=0.0,
                )
                if success:
                    logger.info("Descent command sent via direct MAVLink")
                    return True
            
            logger.warning("Could not send descent command - MAVLink interface incomplete")
            return False
            
        except Exception as e:
            logger.error(f"Descent command error: {e}")
            return False
    
    
    def _trigger_failsafe(self, reason: str) -> None:
        """
        Trigger VIO failsafe response with rangefinder-assisted descent.
        
        Per PRD [T2-SAFE-01]:
        1. Check if rangefinder is healthy and altitude is within range
        2. If yes: Command controlled descent at 0.3 m/s (safer for indoor)
        3. If no: Fall back to ALT_HOLD mode (barometer-based)
        4. Alert pilot for manual takeover
        
        SAFETY: Rangefinder-assisted descent is preferred indoors because:
        - Barometer is unreliable indoors (ground effect, HVAC disturbances)
        - Rangefinder provides direct altitude feedback for precise control
        - Slow descent (0.3 m/s) gives pilot time to take control
        
        NOT using Auto-land, RTL, or motor kill as these are unsafe in
        confined indoor spaces.
        """
        self._failsafe_triggered = True
        
        logger.critical(f"VIO FAILSAFE TRIGGERED: {reason}")
        
        self._update_status(
            health=VIOHealth.FAILED,
            error_message=f"Failsafe: {reason}",
        )
        
        # Notify callback first (may want to alert GCS)
        if self._on_failsafe_trigger:
            try:
                self._on_failsafe_trigger(reason)
            except Exception as e:
                logger.error(f"Failsafe callback error: {e}")
        
        # Check if controlled descent is possible (rangefinder healthy, low altitude)
        rngfnd_healthy, current_alt = self._check_rangefinder_healthy()
        
        if rngfnd_healthy and current_alt < self.MAX_RANGEFINDER_ALTITUDE:
            # Controlled descent using rangefinder feedback
            logger.warning(
                f"VIO FAILSAFE: Initiating rangefinder-assisted descent "
                f"at {current_alt:.1f}m (rate={self.FAILSAFE_DESCENT_RATE} m/s)"
            )
            
            if self._command_descent(self.FAILSAFE_DESCENT_RATE):
                logger.info("Rangefinder-assisted descent active - pilot can take over anytime")
                return
            else:
                logger.warning("Descent command failed, falling back to ALT_HOLD")
        else:
            # Log why we're not using rangefinder
            if not rngfnd_healthy:
                logger.warning("VIO FAILSAFE: Rangefinder not healthy, switching to ALT_HOLD")
            else:
                logger.warning(
                    f"VIO FAILSAFE: Altitude {current_alt:.1f}m too high for rangefinder "
                    f"(max={self.MAX_RANGEFINDER_ALTITUDE}m), switching to ALT_HOLD"
                )
        
        # Fall back to ALT_HOLD mode if no rangefinder
        try:
            if hasattr(self._mavlink, 'set_mode'):
                success = self._mavlink.set_mode(self.ALT_HOLD_MODE)
                if success:
                    logger.info("Successfully switched to ALT_HOLD mode")
                else:
                    logger.error("Failed to switch to ALT_HOLD mode - pilot intervention required")
            else:
                logger.warning("MAVLink service does not support mode change - pilot must switch manually")
        except Exception as e:
            logger.error(f"Mode change error: {e}")
        
        logger.critical("PILOT ACTION REQUIRED: VIO FAILED - Take manual control (ALT_HOLD mode)")
        
        # Update state manager if available
        if self._state_manager:
            try:
                self._state_manager.set_vio_failed(True, reason)
            except Exception as e:
                logger.debug(f"Failed to update state manager: {e}")
    
    
    def _update_rate(self) -> None:
        """Update message rate calculation."""
        now = time.time()
        elapsed = now - self._rate_timestamp
        
        if elapsed >= 1.0:
            rate = self._message_count / elapsed
            self._update_status(message_rate_hz=rate)
            self._message_count = 0
            self._rate_timestamp = now
    
    def _update_status(self, **kwargs) -> None:
        """Update VIO status."""
        with self._lock:
            old_health = self._status.health
            
            # Update fields
            for key, value in kwargs.items():
                if hasattr(self._status, key):
                    setattr(self._status, key, value)
            
            # Rebuild status object (dataclass is not mutable by default)
            self._status = VIOStatus(
                health=kwargs.get("health", self._status.health),
                tracking_confidence=kwargs.get("tracking_confidence", self._status.tracking_confidence),
                position_valid=kwargs.get("position_valid", self._status.position_valid),
                velocity_valid=kwargs.get("velocity_valid", self._status.velocity_valid),
                last_update_ms=kwargs.get("last_update_ms", self._status.last_update_ms),
                message_rate_hz=kwargs.get("message_rate_hz", self._status.message_rate_hz),
                ekf_healthy=kwargs.get("ekf_healthy", self._status.ekf_healthy),
                ekf_variance_ok=kwargs.get("ekf_variance_ok", self._status.ekf_variance_ok),
                reset_counter=self._reset_counter,
                error_message=kwargs.get("error_message", self._status.error_message),
            )
            
            new_health = self._status.health
        
        # Notify status change
        if old_health != new_health and self._on_status_change:
            self._on_status_change(self._status)


class VIOCalibration:
    """
    VIO calibration utilities for NOMAD.
    
    Provides tools for:
    - Initial position calibration
    - IMU-Camera extrinsic validation
    - Floor plane detection
    """
    
    @staticmethod
    def create_initial_pose(
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw_deg: float = 0.0,
    ) -> dict:
        """
        Create initial pose for VIO calibration.
        
        Args:
            x: Initial X position (meters, NED North)
            y: Initial Y position (meters, NED East)
            z: Initial Z position (meters, NED Down, typically negative for above ground)
            yaw_deg: Initial heading (degrees, 0=North, 90=East)
            
        Returns:
            Dictionary with initial pose parameters
        """
        import math
        
        yaw_rad = math.radians(yaw_deg)
        
        return {
            "position": [x, y, z],
            "orientation": [0.0, 0.0, math.sin(yaw_rad / 2), math.cos(yaw_rad / 2)],
            "yaw_deg": yaw_deg,
        }
    
    @staticmethod
    def validate_tracking(camera_service, duration_s: float = 5.0) -> dict:
        """
        Validate VIO tracking quality.
        
        Collects pose data for specified duration and analyzes:
        - Tracking stability
        - Confidence levels
        - Drift estimation
        
        Returns:
            Validation results dictionary
        """
        results = {
            "valid": False,
            "avg_confidence": 0.0,
            "tracking_losses": 0,
            "drift_estimate_m": 0.0,
            "samples": 0,
        }
        
        if not camera_service.is_tracking:
            results["error"] = "Tracking not enabled"
            return results
        
        start_time = time.time()
        confidences = []
        tracking_losses = 0
        initial_pose = None
        final_pose = None
        
        while time.time() - start_time < duration_s:
            pose = camera_service.get_pose()
            
            if pose:
                if initial_pose is None:
                    initial_pose = pose
                final_pose = pose
                
                confidences.append(pose.confidence)
                
                if pose.tracking_state.value != 1:  # Not OK
                    tracking_losses += 1
            else:
                tracking_losses += 1
                
            time.sleep(0.033)  # ~30Hz
        
        if confidences:
            results["avg_confidence"] = sum(confidences) / len(confidences)
            results["samples"] = len(confidences)
            results["tracking_losses"] = tracking_losses
            
            # Estimate drift (assuming stationary)
            if initial_pose and final_pose:
                dx = final_pose.position[0] - initial_pose.position[0]
                dy = final_pose.position[1] - initial_pose.position[1]
                dz = final_pose.position[2] - initial_pose.position[2]
                results["drift_estimate_m"] = (dx**2 + dy**2 + dz**2) ** 0.5
            
            # Valid if confidence > 50% and few tracking losses
            results["valid"] = (
                results["avg_confidence"] > 50 and
                tracking_losses < results["samples"] * 0.1
            )
        
        return results
