"""
DEPRECATED: This module is deprecated as of January 2026.
The nav_controller middleware has been replaced by direct ROS2-MAVLink architecture:
- Nav2 publishes to /cmd_vel
- mavros2 subscribes and sends SET_POSITION_TARGET_LOCAL_NED to ArduPilot

This file is kept for reference and potential fallback only.
See docs/NAV2_INTEGRATION_PLAN.md for the new architecture.
"""

"""
NOMAD Edge Core - Navigation Controller

Jetson-centric navigation that bridges ROS2 nav2/nvblox velocity commands
to ArduPilot MAVLink. ArduPilot operates in GUIDED mode as a low-level
flight controller, while Jetson handles all navigation planning.

Architecture:
    Isaac ROS (nav2/nvblox) -> /cmd_vel -> ros_http_bridge -> Edge Core API
    -> NavController -> MavlinkService -> GUIDED mode velocity commands

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Callable, Optional

if TYPE_CHECKING:
    from .mavlink_interface import MavlinkService
    from .state import StateManager

logger = logging.getLogger("edge_core.nav_controller")


class NavMode(Enum):
    """Navigation controller operating modes."""
    DISABLED = "disabled"       # Not accepting commands
    STANDBY = "standby"         # Ready but not moving
    VELOCITY = "velocity"       # Following velocity commands
    POSITION = "position"       # Moving to position target
    VISUAL_SERVO = "visual_servo"  # Target tracking mode


class NavHealth(Enum):
    """Navigation system health states."""
    UNKNOWN = "unknown"
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    FAILED = "failed"


@dataclass
class VelocityCommand:
    """Velocity command from ROS2 nav2/nvblox."""
    timestamp: float
    vx: float       # Forward velocity (m/s) - body frame
    vy: float       # Lateral velocity (m/s) - body frame
    vz: float       # Vertical velocity (m/s) - positive up
    yaw_rate: float # Yaw rate (rad/s) - positive CCW
    source: str = "nav2"


@dataclass 
class PositionTarget:
    """Position target for waypoint navigation."""
    timestamp: float
    x: float        # North (NED frame, meters)
    y: float        # East (NED frame, meters)
    z: float        # Down (NED frame, meters, positive down)
    yaw: float      # Heading (radians)
    source: str = "nav2"


@dataclass
class NavStatus:
    """Navigation controller status."""
    mode: NavMode = NavMode.DISABLED
    health: NavHealth = NavHealth.UNKNOWN
    last_command_age_ms: int = 0
    command_rate_hz: float = 0.0
    guided_mode_active: bool = False
    vio_healthy: bool = False
    armed: bool = False
    error_message: Optional[str] = None
    
    # Current commanded values
    cmd_vx: float = 0.0
    cmd_vy: float = 0.0
    cmd_vz: float = 0.0
    cmd_yaw_rate: float = 0.0
    
    def to_dict(self) -> dict:
        return {
            "mode": self.mode.value,
            "health": self.health.value,
            "last_command_age_ms": self.last_command_age_ms,
            "command_rate_hz": self.command_rate_hz,
            "guided_mode_active": self.guided_mode_active,
            "vio_healthy": self.vio_healthy,
            "armed": self.armed,
            "error_message": self.error_message,
            "cmd_vx": self.cmd_vx,
            "cmd_vy": self.cmd_vy,
            "cmd_vz": self.cmd_vz,
            "cmd_yaw_rate": self.cmd_yaw_rate,
        }


class NavController:
    """
    Jetson Navigation Controller for NOMAD Task 2.
    
    This controller receives velocity commands from ROS2 nav2/nvblox stack
    and translates them to ArduPilot MAVLink messages. ArduPilot must be
    in GUIDED mode for this to work.
    
    Key responsibilities:
    1. Accept velocity commands from Isaac ROS (via HTTP API)
    2. Validate VIO health before sending commands
    3. Send SET_POSITION_TARGET_LOCAL_NED to ArduPilot
    4. Implement safety timeouts (stop if no commands)
    5. Handle mode transitions and failsafes
    
    PRD Requirements:
    - [T2-NAV-03]: Jetson-centric navigation - AP is flight controller only
    - [T2-SAFE-01]: VIO failure triggers safe mode
    
    Usage:
        nav = NavController(mavlink_service, state_manager)
        nav.start()
        nav.send_velocity(vx=0.5, vy=0.0, vz=0.0, yaw_rate=0.0)
    """
    
    # Command timeout - stop if no commands received
    COMMAND_TIMEOUT_S = 0.5
    
    # Maximum velocities (safety limits)
    MAX_VELOCITY_XY = 2.0   # m/s horizontal
    MAX_VELOCITY_Z = 1.0    # m/s vertical
    MAX_YAW_RATE = 1.0      # rad/s
    
    # Minimum VIO confidence to accept commands
    MIN_VIO_CONFIDENCE = 0.3
    
    # ArduPilot GUIDED mode ID
    GUIDED_MODE_ID = 4
    
    def __init__(
        self,
        mavlink_service: "MavlinkService",
        state_manager: "StateManager",
        on_status_change: Optional[Callable[[NavStatus], None]] = None,
    ):
        self._mavlink = mavlink_service
        self._state_manager = state_manager
        self._on_status_change = on_status_change
        
        self._status = NavStatus()
        self._lock = threading.RLock()
        
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        
        # Command tracking
        self._last_velocity_cmd: Optional[VelocityCommand] = None
        self._last_command_time = 0.0
        self._command_count = 0
        self._rate_timestamp = time.time()
        
        # VIO state reference (set by main orchestrator)
        self._vio_confidence = 0.0
        self._vio_healthy = False
        
        logger.info("NavController initialized")
    
    @property
    def status(self) -> NavStatus:
        """Get current navigation status."""
        with self._lock:
            return self._status
    
    @property
    def is_active(self) -> bool:
        """Check if navigation is actively controlling the vehicle."""
        with self._lock:
            return self._status.mode in (NavMode.VELOCITY, NavMode.POSITION, NavMode.VISUAL_SERVO)
    
    def start(self) -> bool:
        """Start the navigation controller."""
        if self._thread and self._thread.is_alive():
            logger.warning("NavController already running")
            return True
        
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        
        self._update_status(mode=NavMode.STANDBY, health=NavHealth.HEALTHY)
        logger.info("NavController started")
        return True
    
    def stop(self) -> None:
        """Stop the navigation controller and send zero velocity."""
        self._stop_event.set()
        
        # Send stop command
        self._send_stop_velocity()
        
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        
        self._update_status(mode=NavMode.DISABLED)
        logger.info("NavController stopped")
    
    def set_vio_state(self, confidence: float, healthy: bool) -> None:
        """Update VIO health status (called by VIO pipeline)."""
        with self._lock:
            self._vio_confidence = confidence
            self._vio_healthy = healthy
            self._status.vio_healthy = healthy
    
    def send_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float,
        source: str = "nav2",
    ) -> bool:
        """
        Send velocity command to the vehicle.
        
        This is the main entry point for ROS2 nav2/nvblox velocity commands.
        Commands are in body frame (forward, right, up).
        
        Args:
            vx: Forward velocity (m/s, positive = forward)
            vy: Lateral velocity (m/s, positive = right)
            vz: Vertical velocity (m/s, positive = up)
            yaw_rate: Yaw rate (rad/s, positive = CCW)
            source: Command source identifier
            
        Returns:
            True if command was accepted and sent
        """
        with self._lock:
            # Check if we can accept commands
            if self._status.mode == NavMode.DISABLED:
                logger.warning("NavController is disabled - ignoring velocity command")
                return False
            
            # Check VIO health
            if not self._vio_healthy:
                logger.warning(f"VIO unhealthy (confidence={self._vio_confidence:.2f}) - refusing velocity command")
                return False
            
            # Check armed state
            state = self._state_manager.get_state()
            if not state.armed:
                logger.warning("Vehicle not armed - refusing velocity command")
                return False
            
            # Create command
            cmd = VelocityCommand(
                timestamp=time.time(),
                vx=self._clamp(vx, -self.MAX_VELOCITY_XY, self.MAX_VELOCITY_XY),
                vy=self._clamp(vy, -self.MAX_VELOCITY_XY, self.MAX_VELOCITY_XY),
                vz=self._clamp(vz, -self.MAX_VELOCITY_Z, self.MAX_VELOCITY_Z),
                yaw_rate=self._clamp(yaw_rate, -self.MAX_YAW_RATE, self.MAX_YAW_RATE),
                source=source,
            )
            
            self._last_velocity_cmd = cmd
            self._last_command_time = time.time()
            self._command_count += 1
            
            # Update status
            self._status.mode = NavMode.VELOCITY
            self._status.cmd_vx = cmd.vx
            self._status.cmd_vy = cmd.vy
            self._status.cmd_vz = cmd.vz
            self._status.cmd_yaw_rate = cmd.yaw_rate
        
        # Send to MAVLink
        return self._send_velocity_mavlink(cmd)
    
    def send_position(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        source: str = "nav2",
    ) -> bool:
        """
        Send position target to the vehicle.
        
        Position is in local NED frame relative to VIO origin.
        
        Args:
            x: North position (meters)
            y: East position (meters)
            z: Down position (meters, positive = down)
            yaw: Heading (radians)
            source: Command source identifier
            
        Returns:
            True if command was accepted and sent
        """
        with self._lock:
            if self._status.mode == NavMode.DISABLED:
                return False
            
            if not self._vio_healthy:
                logger.warning("VIO unhealthy - refusing position command")
                return False
            
            state = self._state_manager.get_state()
            if not state.armed:
                return False
            
            self._last_command_time = time.time()
            self._status.mode = NavMode.POSITION
        
        # Send position target via MAVLink
        return self._send_position_mavlink(x, y, z, yaw)
    
    def stop_movement(self) -> bool:
        """Send zero velocity command to stop all movement."""
        logger.info("Stop movement commanded")
        return self._send_stop_velocity()
    
    def enable_guided_mode(self) -> bool:
        """Request ArduPilot to enter GUIDED mode."""
        logger.info("Requesting GUIDED mode")
        return self._mavlink.set_mode(self.GUIDED_MODE_ID)
    
    def _run_loop(self) -> None:
        """Main control loop - handles timeouts and status updates."""
        loop_rate = 20.0  # Hz
        interval = 1.0 / loop_rate
        
        while not self._stop_event.is_set():
            try:
                start_time = time.time()
                
                self._check_command_timeout()
                self._check_flight_mode()
                self._update_rate()
                
                elapsed = time.time() - start_time
                if elapsed < interval:
                    time.sleep(interval - elapsed)
                    
            except Exception as e:
                logger.error(f"NavController loop error: {e}")
                time.sleep(0.1)
    
    def _check_command_timeout(self) -> None:
        """Check for command timeout and stop if needed."""
        with self._lock:
            if self._status.mode not in (NavMode.VELOCITY, NavMode.POSITION):
                return
            
            now = time.time()
            age = now - self._last_command_time
            self._status.last_command_age_ms = int(age * 1000)
            
            if age > self.COMMAND_TIMEOUT_S:
                logger.warning(f"Command timeout ({age:.2f}s) - stopping")
                self._send_stop_velocity()
                self._status.mode = NavMode.STANDBY
                self._status.cmd_vx = 0.0
                self._status.cmd_vy = 0.0
                self._status.cmd_vz = 0.0
                self._status.cmd_yaw_rate = 0.0
    
    def _check_flight_mode(self) -> None:
        """Check if ArduPilot is in GUIDED mode."""
        state = self._state_manager.get_state()
        with self._lock:
            self._status.guided_mode_active = state.flight_mode == "GUIDED"
            self._status.armed = state.armed
    
    def _update_rate(self) -> None:
        """Update command rate calculation."""
        now = time.time()
        elapsed = now - self._rate_timestamp
        
        if elapsed >= 1.0:
            with self._lock:
                self._status.command_rate_hz = self._command_count / elapsed
            self._command_count = 0
            self._rate_timestamp = now
    
    def _send_velocity_mavlink(self, cmd: VelocityCommand) -> bool:
        """Send velocity command via MAVLink."""
        try:
            # Convert from body frame (FRD) to MAVLink convention
            # nav2 cmd_vel: x=forward, y=left, z=up, yaw=CCW positive
            # ArduPilot body: x=forward, y=right, z=down
            success = self._mavlink.send_velocity_command(
                vx=cmd.vx,           # Forward
                vy=-cmd.vy,          # Left -> Right (negate)
                vz=-cmd.vz,          # Up -> Down (negate)
                yaw_rate=-cmd.yaw_rate,  # CCW -> CW (negate for NED)
            )
            
            if not success:
                logger.warning("Failed to send velocity command to MAVLink")
            
            return success
            
        except Exception as e:
            logger.error(f"Velocity MAVLink error: {e}")
            return False
    
    def _send_position_mavlink(
        self, x: float, y: float, z: float, yaw: float
    ) -> bool:
        """Send position target via MAVLink."""
        try:
            # Use SET_POSITION_TARGET_LOCAL_NED with position mask
            if hasattr(self._mavlink, 'send_position_target'):
                return self._mavlink.send_position_target(x, y, z, yaw)
            else:
                logger.warning("Position target not implemented in MAVLink service")
                return False
                
        except Exception as e:
            logger.error(f"Position MAVLink error: {e}")
            return False
    
    def _send_stop_velocity(self) -> bool:
        """Send zero velocity to stop movement."""
        try:
            return self._mavlink.send_velocity_command(0.0, 0.0, 0.0, 0.0)
        except Exception:
            return False
    
    def _update_status(self, **kwargs) -> None:
        """Update navigation status."""
        with self._lock:
            old_mode = self._status.mode
            
            for key, value in kwargs.items():
                if hasattr(self._status, key):
                    setattr(self._status, key, value)
            
            # Rebuild status
            self._status = NavStatus(
                mode=kwargs.get("mode", self._status.mode),
                health=kwargs.get("health", self._status.health),
                last_command_age_ms=kwargs.get("last_command_age_ms", self._status.last_command_age_ms),
                command_rate_hz=kwargs.get("command_rate_hz", self._status.command_rate_hz),
                guided_mode_active=kwargs.get("guided_mode_active", self._status.guided_mode_active),
                vio_healthy=kwargs.get("vio_healthy", self._status.vio_healthy),
                armed=kwargs.get("armed", self._status.armed),
                error_message=kwargs.get("error_message", self._status.error_message),
                cmd_vx=kwargs.get("cmd_vx", self._status.cmd_vx),
                cmd_vy=kwargs.get("cmd_vy", self._status.cmd_vy),
                cmd_vz=kwargs.get("cmd_vz", self._status.cmd_vz),
                cmd_yaw_rate=kwargs.get("cmd_yaw_rate", self._status.cmd_yaw_rate),
            )
            
            new_mode = self._status.mode
        
        # Notify callback
        if old_mode != new_mode and self._on_status_change:
            self._on_status_change(self._status)
    
    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        """Clamp value to range."""
        return max(min_val, min(max_val, value))
