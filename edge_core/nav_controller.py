from __future__ import annotations

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

import logging
import math
import os
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
    vio_age_ms: Optional[int] = None
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
            "vio_age_ms": self.vio_age_ms,
            "armed": self.armed,
            "error_message": self.error_message,
            "cmd_vx": self.cmd_vx,
            "cmd_vy": self.cmd_vy,
            "cmd_vz": self.cmd_vz,
            "cmd_yaw_rate": self.cmd_yaw_rate,
        }


@dataclass
class RthLandingStatus:
    """Operator-gated guided return-home landing workflow status."""
    active: bool = False
    phase: str = "idle"
    next_phase: str = "climb"
    awaiting_approval: bool = False
    paused: bool = False
    message: str = "RTH landing idle"
    error_message: Optional[str] = None
    climb_alt_m: float = 30.0
    descent_rate_mps: float = 0.5
    descent_target_agl_m: float = 0.3
    home_lat: Optional[float] = None
    home_lon: Optional[float] = None
    home_alt_msl: Optional[float] = None
    target_lat: Optional[float] = None
    target_lon: Optional[float] = None
    target_alt_msl: Optional[float] = None
    distance_to_home_m: Optional[float] = None
    altitude_error_m: Optional[float] = None
    current_alt_msl: Optional[float] = None
    current_alt_agl_m: Optional[float] = None
    updated_at: float = 0.0

    def to_dict(self) -> dict:
        return {
            "active": self.active,
            "phase": self.phase,
            "next_phase": self.next_phase,
            "awaiting_approval": self.awaiting_approval,
            "paused": self.paused,
            "message": self.message,
            "error_message": self.error_message,
            "climb_alt_m": self.climb_alt_m,
            "descent_rate_mps": self.descent_rate_mps,
            "descent_target_agl_m": self.descent_target_agl_m,
            "home_lat": self.home_lat,
            "home_lon": self.home_lon,
            "home_alt_msl": self.home_alt_msl,
            "target_lat": self.target_lat,
            "target_lon": self.target_lon,
            "target_alt_msl": self.target_alt_msl,
            "distance_to_home_m": self.distance_to_home_m,
            "altitude_error_m": self.altitude_error_m,
            "current_alt_msl": self.current_alt_msl,
            "current_alt_agl_m": self.current_alt_agl_m,
            "updated_at": self.updated_at,
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
    RTH_CLIMB_ALT_TOLERANCE_M = 1.0
    RTH_HOME_RADIUS_M = 2.0
    RTH_DESCENT_INTERVAL_S = 0.5

    # Maximum local position targets relative to the VIO origin. Position
    # commands are less frequently used than velocity commands, so reject
    # outliers instead of silently clipping to a surprising destination.
    MAX_POSITION_XY_M = 50.0
    MAX_POSITION_Z_M = 20.0
    
    # Minimum VIO confidence to accept commands
    MIN_VIO_CONFIDENCE = 0.3
    
    # ArduPilot GUIDED mode ID
    GUIDED_MODE_ID = 4
    
    def __init__(
        self,
        mavlink_service: "MavlinkService",
        state_manager: "StateManager",
        on_status_change: Optional[Callable[[NavStatus], None]] = None,
        nav2_enabled: bool = False,
    ):
        self._mavlink = mavlink_service
        self._state_manager = state_manager
        self._on_status_change = on_status_change
        self._nav2_enabled = nav2_enabled
        
        self._status = NavStatus()
        self._lock = threading.RLock()
        
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        
        # Command tracking
        self._last_velocity_cmd: Optional[VelocityCommand] = None
        self._last_command_time = 0.0
        self._command_count = 0
        # Monotonic clock for all interval/age/timeout math: time_manager
        # forces GPS wall-clock syncs at runtime, so time.time() can jump
        # backwards or forwards and either trip the velocity watchdog falsely
        # or blind it while the drone keeps moving.
        self._rate_timestamp = time.monotonic()
        
        # VIO state reference (set by main orchestrator)
        self._vio_confidence = 0.0
        self._vio_healthy = False
        self._vio_last_update_monotonic: float | None = None
        self._vio_max_age_s = self._read_positive_float("NOMAD_VIO_MAX_AGE_S", 1.0)

        self._rth_status = RthLandingStatus(updated_at=time.time())
        self._rth_descent_thread: Optional[threading.Thread] = None
        self._rth_descent_stop = threading.Event()
        
        mode_desc = "nav2 mode" if nav2_enabled else "API mode"
        logger.info(f"NavController initialized ({mode_desc})")
    
    @property
    def status(self) -> NavStatus:
        """Get current navigation status."""
        with self._lock:
            return self._status

    @property
    def rth_landing_status(self) -> RthLandingStatus:
        """Get current guided RTH landing status."""
        with self._lock:
            self._refresh_rth_progress_locked()
            return self._rth_status
    
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
    
    def set_vio_state(
        self,
        confidence: float,
        healthy: bool,
        received_monotonic: float | None = None,
    ) -> None:
        """Update VIO health status (called by VIO pipeline)."""
        with self._lock:
            self._vio_confidence = confidence
            self._vio_healthy = healthy
            self._vio_last_update_monotonic = (
                received_monotonic if received_monotonic is not None else time.monotonic()
            )
            vio_fresh = self._is_vio_fresh_locked()
            self._status.vio_healthy = healthy and vio_fresh
            self._status.vio_age_ms = self._vio_age_ms_locked()
            if healthy and vio_fresh:
                self._status.health = NavHealth.HEALTHY
    
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
        Commands are in ROS REP-103 convention (FLU body frame); they are
        negated to MAVLink BODY_OFFSET_NED (FRD) inside _send_velocity_mavlink.

        Args:
            vx: Forward velocity (m/s, positive = forward)
            vy: Lateral velocity (m/s, positive = LEFT)
            vz: Vertical velocity (m/s, positive = UP)
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
            if not self._is_vio_ready_locked():
                age_ms = self._vio_age_ms_locked()
                age_text = "unknown" if age_ms is None else f"{age_ms}ms"
                logger.warning(
                    "VIO unhealthy or stale (confidence=%.2f, age=%s) - refusing velocity command",
                    self._vio_confidence,
                    age_text,
                )
                return False
            
            # Check armed state
            state = self._state_manager.get_state()
            if not state.armed:
                logger.warning("Vehicle not armed - refusing velocity command")
                return False
            if state.flight_mode != "GUIDED":
                logger.warning(
                    "Vehicle not in GUIDED mode (%s) - refusing velocity command",
                    state.flight_mode,
                )
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
            self._last_command_time = time.monotonic()
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
            
            if not self._is_vio_ready_locked():
                age_ms = self._vio_age_ms_locked()
                age_text = "unknown" if age_ms is None else f"{age_ms}ms"
                logger.warning(
                    "VIO unhealthy or stale (confidence=%.2f, age=%s) - refusing position command",
                    self._vio_confidence,
                    age_text,
                )
                return False
            
            state = self._state_manager.get_state()
            if not state.armed:
                return False
            if state.flight_mode != "GUIDED":
                logger.warning(
                    "Vehicle not in GUIDED mode (%s) - refusing position command",
                    state.flight_mode,
                )
                return False

            horizontal_distance = (x * x + y * y) ** 0.5
            if horizontal_distance > self.MAX_POSITION_XY_M:
                logger.warning(
                    "Position target rejected: horizontal distance %.1fm exceeds %.1fm",
                    horizontal_distance,
                    self.MAX_POSITION_XY_M,
                )
                return False
            if abs(z) > self.MAX_POSITION_Z_M:
                logger.warning(
                    "Position target rejected: |z| %.1fm exceeds %.1fm",
                    abs(z),
                    self.MAX_POSITION_Z_M,
                )
                return False
            
            self._last_command_time = time.monotonic()
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

    def wait_for_guided(self, timeout_s: float = 3.0) -> bool:
        """Block until the autopilot reports it is in GUIDED mode.

        Used by the autonomy entry path (auto-takeoff + spray) so the caller
        knows the next velocity command will actually be accepted instead of
        racing the mode change. Returns ``False`` on timeout.
        """
        deadline = time.monotonic() + max(0.1, timeout_s)
        while time.monotonic() < deadline:
            if self._state_manager.get_state().flight_mode == "GUIDED":
                return True
            time.sleep(0.05)
        return False

    def auto_takeoff(self, altitude_m: float = 30.0) -> dict:
        """End-to-end autonomous takeoff for the CONOPS 5.2.4 5-pt criterion.

        Sequence: switch to GUIDED -> wait for GUIDED ack -> arm motors ->
        send NAV_TAKEOFF. The caller (or the operator) keeps the RC mode
        switch as a safety override; flipping the switch immediately drops
        GUIDED and returns the vehicle to the pilot's control.

        Returns a structured result with which step (if any) failed so the
        UI can show an actionable error.
        """
        altitude_m = max(1.0, min(float(altitude_m), 60.0))  # clamp to sane band

        if not self.enable_guided_mode():
            return {"success": False, "stage": "set_mode", "error": "Failed to send mode-change command"}

        if not self.wait_for_guided(timeout_s=3.0):
            return {
                "success": False,
                "stage": "wait_guided",
                "error": "Autopilot did not enter GUIDED within 3 s - check pre-arm/EKF",
            }

        # Arm. ArduCopter rejects ARM in some pre-arm failure states; we have
        # no way to introspect why from here, so surface the failure to the UI.
        try:
            self._mavlink.arm_disarm(True)
        except Exception as e:
            return {"success": False, "stage": "arm", "error": f"Arm command exception: {e}"}

        # Give the FC ~1s to actually arm before we send takeoff (NAV_TAKEOFF
        # is rejected if disarmed).
        armed_deadline = time.monotonic() + 2.0
        while time.monotonic() < armed_deadline:
            if self._state_manager.get_state().armed:
                break
            time.sleep(0.1)
        if not self._state_manager.get_state().armed:
            return {"success": False, "stage": "arm", "error": "Autopilot did not arm - check pre-arm checks"}

        if not self._mavlink.takeoff(altitude_m):
            return {"success": False, "stage": "takeoff", "error": "NAV_TAKEOFF was not acknowledged"}

        logger.info(f"Auto-takeoff commanded to {altitude_m:.1f} m AGL")
        return {"success": True, "altitude_m": altitude_m, "stage": "takeoff"}

    def start_rth_landing(
        self,
        climb_alt_m: float = 30.0,
        descent_rate_mps: float = 0.5,
        descent_target_agl_m: float = 0.3,
    ) -> dict:
        """Create an operator-approved guided return-home landing plan.

        The plan never switches to ArduCopter LAND mode. The operator must
        approve each phase:
        1. climb/hold to a fixed altitude above home,
        2. fly to the ArduPilot home point at that altitude,
        3. descend over home using repeated guided position targets.
        """
        climb_alt_m = self._clamp(float(climb_alt_m), 1.0, 150.0)
        descent_rate_mps = self._clamp(float(descent_rate_mps), 0.1, 1.0)
        descent_target_agl_m = self._clamp(float(descent_target_agl_m), 0.1, 5.0)

        state = self._state_manager.get_state()
        if not state.connected:
            return {"success": False, "error": "Autopilot is not connected"}
        if not state.armed:
            return {"success": False, "error": "Vehicle is not armed"}
        if not state.has_valid_gps() or state.gps_alt is None:
            return {"success": False, "error": "Current GPS position/altitude is unavailable"}

        home_lat, home_lon, home_alt = self._get_home_position()
        if home_lat is None or home_lon is None or home_alt is None:
            return {
                "success": False,
                "error": "ArduPilot home position is unavailable; wait for HOME_POSITION or set home before RTH",
            }

        if climb_alt_m < descent_target_agl_m + 2.0:
            return {
                "success": False,
                "error": (
                    f"RTH altitude {climb_alt_m:.1f}m above home is too low for "
                    f"descent target {descent_target_agl_m:.1f}m AGL"
                ),
            }

        self._rth_descent_stop.set()
        with self._lock:
            self._rth_status = RthLandingStatus(
                active=True,
                phase="idle",
                next_phase="climb",
                awaiting_approval=True,
                paused=False,
                message="Awaiting approval: climb/hold to RTH altitude",
                climb_alt_m=climb_alt_m,
                descent_rate_mps=descent_rate_mps,
                descent_target_agl_m=descent_target_agl_m,
                home_lat=home_lat,
                home_lon=home_lon,
                home_alt_msl=home_alt,
                current_alt_msl=state.gps_alt,
                current_alt_agl_m=state.alt_agl_m,
                updated_at=time.time(),
            )
            self._refresh_rth_progress_locked()

        logger.info(
            "RTH landing plan started: climb_alt=%.1fm above home home=(%.7f, %.7f) descent=%.2fm/s",
            climb_alt_m,
            home_lat,
            home_lon,
            descent_rate_mps,
        )
        return {"success": True, **self.rth_landing_status.to_dict()}

    def approve_rth_landing_phase(self, phase: str | None = None) -> dict:
        """Approve and command the next RTH landing phase."""
        with self._lock:
            self._refresh_rth_progress_locked()
            status = self._rth_status
            if not status.active:
                return {"success": False, "error": "No active RTH landing plan"}
            if status.paused:
                return {"success": False, "error": "RTH landing is paused; resume or edit first"}
            if not status.awaiting_approval:
                return {"success": False, "error": f"Phase {status.phase} is already active"}
            next_phase = (phase or status.next_phase or "").strip().lower()
            if next_phase != status.next_phase:
                return {
                    "success": False,
                    "error": f"Expected approval for {status.next_phase}, not {next_phase}",
                }

        if next_phase == "climb":
            return self._command_rth_climb()
        if next_phase == "go_home":
            return self._command_rth_go_home()
        if next_phase == "descend":
            return self._command_rth_descend()
        return {"success": False, "error": f"Unknown RTH landing phase: {next_phase}"}

    def pause_rth_landing(self) -> dict:
        """Pause RTH landing by commanding a guided hold at the current position."""
        with self._lock:
            if not self._rth_status.active:
                return {"success": False, "error": "No active RTH landing plan"}
            self._rth_status.paused = True
            self._rth_status.awaiting_approval = False
            self._rth_status.message = "Paused: holding current guided position"
            self._rth_status.updated_at = time.time()

        self._send_guided_hold_current_position()
        return {"success": True, **self.rth_landing_status.to_dict()}

    def resume_rth_landing(self) -> dict:
        """Resume the active RTH phase after a pause/edit."""
        with self._lock:
            status = self._rth_status
            if not status.active:
                return {"success": False, "error": "No active RTH landing plan"}
            if not status.paused:
                return {"success": True, **status.to_dict()}
            phase = status.phase
            status.paused = False
            status.message = f"Resuming phase: {phase}"
            status.updated_at = time.time()

        if phase == "climb":
            return self._command_rth_climb(resume=True)
        if phase == "go_home":
            return self._command_rth_go_home(resume=True)
        if phase == "descend":
            return self._command_rth_descend(resume=True)
        with self._lock:
            self._rth_status.awaiting_approval = True
            self._rth_status.message = f"Awaiting approval: {self._rth_status.next_phase}"
        return {"success": True, **self.rth_landing_status.to_dict()}

    def edit_rth_landing(
        self,
        climb_alt_m: float | None = None,
        descent_rate_mps: float | None = None,
        descent_target_agl_m: float | None = None,
    ) -> dict:
        """Edit the active RTH plan, then reissue the current active target."""
        with self._lock:
            status = self._rth_status
            if not status.active:
                return {"success": False, "error": "No active RTH landing plan"}
            if climb_alt_m is not None:
                status.climb_alt_m = self._clamp(float(climb_alt_m), 1.0, 150.0)
            if descent_rate_mps is not None:
                status.descent_rate_mps = self._clamp(float(descent_rate_mps), 0.1, 1.0)
            if descent_target_agl_m is not None:
                status.descent_target_agl_m = self._clamp(float(descent_target_agl_m), 0.1, 5.0)
            status.message = "RTH landing plan edited"
            status.updated_at = time.time()
            phase = status.phase
            paused = status.paused

        if not paused and phase == "climb":
            return self._command_rth_climb(resume=True)
        if not paused and phase == "go_home":
            return self._command_rth_go_home(resume=True)
        return {"success": True, **self.rth_landing_status.to_dict()}

    def abort_rth_landing(self) -> dict:
        """Abort the guided RTH landing workflow without entering LAND mode."""
        self._rth_descent_stop.set()
        self._send_guided_hold_current_position()
        with self._lock:
            self._rth_status.active = False
            self._rth_status.phase = "aborted"
            self._rth_status.next_phase = ""
            self._rth_status.awaiting_approval = False
            self._rth_status.paused = False
            self._rth_status.message = "RTH landing aborted; holding current position"
            self._rth_status.updated_at = time.time()
        return {"success": True, **self.rth_landing_status.to_dict()}

    def auto_land(self) -> dict:
        """Backward-compatible entry point for the guided RTH landing plan."""
        return self.start_rth_landing()

    def _command_rth_climb(self, resume: bool = False) -> dict:
        if not self._ensure_guided_ready():
            return {"success": False, "error": "Could not enter GUIDED mode for RTH climb"}

        state = self._state_manager.get_state()
        if not state.has_valid_gps() or state.gps_alt is None:
            return {"success": False, "error": "Current GPS position/altitude is unavailable"}

        with self._lock:
            target_alt = (self._rth_status.home_alt_msl or 0.0) + self._rth_status.climb_alt_m

        if not self._mavlink.send_global_position_target(state.gps_lat, state.gps_lon, target_alt):
            return {"success": False, "error": "Failed to send guided climb target"}

        with self._lock:
            self._rth_status.phase = "climb"
            self._rth_status.next_phase = "go_home"
            self._rth_status.awaiting_approval = False
            self._rth_status.paused = False
            self._rth_status.message = (
                "Resumed climb/hold to RTH altitude"
                if resume else "Commanded climb/hold to RTH altitude"
            )
            self._rth_status.target_lat = state.gps_lat
            self._rth_status.target_lon = state.gps_lon
            self._rth_status.target_alt_msl = target_alt
            self._rth_status.updated_at = time.time()
            self._refresh_rth_progress_locked()
        return {"success": True, **self.rth_landing_status.to_dict()}

    def _command_rth_go_home(self, resume: bool = False) -> dict:
        if not self._ensure_guided_ready():
            return {"success": False, "error": "Could not enter GUIDED mode for RTH"}

        with self._lock:
            status = self._rth_status
            home_lat, home_lon = status.home_lat, status.home_lon
            target_alt = (status.home_alt_msl or 0.0) + status.climb_alt_m

        if home_lat is None or home_lon is None:
            return {"success": False, "error": "Home position is unavailable"}
        if not self._mavlink.send_global_position_target(home_lat, home_lon, target_alt):
            return {"success": False, "error": "Failed to send guided home target"}

        with self._lock:
            self._rth_status.phase = "go_home"
            self._rth_status.next_phase = "descend"
            self._rth_status.awaiting_approval = False
            self._rth_status.paused = False
            self._rth_status.message = (
                "Resumed return to home point"
                if resume else "Commanded return to home point"
            )
            self._rth_status.target_lat = home_lat
            self._rth_status.target_lon = home_lon
            self._rth_status.target_alt_msl = target_alt
            self._rth_status.updated_at = time.time()
            self._refresh_rth_progress_locked()
        return {"success": True, **self.rth_landing_status.to_dict()}

    def _command_rth_descend(self, resume: bool = False) -> dict:
        if not self._ensure_guided_ready():
            return {"success": False, "error": "Could not enter GUIDED mode for RTH descent"}

        with self._lock:
            status = self._rth_status
            if status.home_lat is None or status.home_lon is None or status.home_alt_msl is None:
                return {"success": False, "error": "Home position is unavailable"}
            status.phase = "descend"
            status.next_phase = ""
            status.awaiting_approval = False
            status.paused = False
            status.message = (
                "Resumed controlled descent over home"
                if resume else "Commanded controlled descent over home"
            )
            status.target_lat = status.home_lat
            status.target_lon = status.home_lon
            status.target_alt_msl = status.home_alt_msl + status.descent_target_agl_m
            status.updated_at = time.time()

        self._rth_descent_stop.clear()
        if self._rth_descent_thread is None or not self._rth_descent_thread.is_alive():
            self._rth_descent_thread = threading.Thread(
                target=self._rth_descent_loop,
                name="nomad-rth-guided-descent",
                daemon=True,
            )
            self._rth_descent_thread.start()
        return {"success": True, **self.rth_landing_status.to_dict()}

    def _rth_descent_loop(self) -> None:
        last_command = 0.0
        while not self._rth_descent_stop.is_set():
            with self._lock:
                status = self._rth_status
                if not status.active or status.phase != "descend":
                    return
                paused = status.paused
                home_lat, home_lon, home_alt = status.home_lat, status.home_lon, status.home_alt_msl
                rate = status.descent_rate_mps
                target_agl = status.descent_target_agl_m
            if paused:
                time.sleep(self.RTH_DESCENT_INTERVAL_S)
                continue

            state = self._state_manager.get_state()
            if (
                home_lat is None or home_lon is None or home_alt is None
                or state.gps_alt is None or state.alt_agl_m is None
            ):
                self._fail_rth_landing("GPS altitude unavailable during guided descent")
                return

            target_alt_msl = home_alt + target_agl
            if state.alt_agl_m <= target_agl + 0.2 or state.gps_alt <= target_alt_msl + 0.2:
                self._mavlink.send_global_position_target(home_lat, home_lon, target_alt_msl)
                with self._lock:
                    self._rth_status.active = False
                    self._rth_status.phase = "complete"
                    self._rth_status.next_phase = ""
                    self._rth_status.awaiting_approval = False
                    self._rth_status.message = (
                        "Guided descent reached touchdown altitude; disarm manually when stable"
                    )
                    self._rth_status.updated_at = time.time()
                    self._refresh_rth_progress_locked()
                logger.info("RTH guided descent complete at %.2fm AGL", state.alt_agl_m)
                return

            now = time.monotonic()
            step_dt = max(self.RTH_DESCENT_INTERVAL_S, now - last_command) if last_command else self.RTH_DESCENT_INTERVAL_S
            last_command = now
            next_alt_msl = max(target_alt_msl, state.gps_alt - rate * step_dt)
            if not self._mavlink.send_global_position_target(home_lat, home_lon, next_alt_msl):
                self._fail_rth_landing("Failed to send guided descent target")
                return

            with self._lock:
                self._rth_status.target_alt_msl = next_alt_msl
                self._rth_status.updated_at = time.time()
                self._refresh_rth_progress_locked()
            time.sleep(self.RTH_DESCENT_INTERVAL_S)

    def _fail_rth_landing(self, error: str) -> None:
        logger.error("RTH landing failed: %s", error)
        with self._lock:
            self._rth_status.active = False
            self._rth_status.phase = "failed"
            self._rth_status.next_phase = ""
            self._rth_status.awaiting_approval = False
            self._rth_status.error_message = error
            self._rth_status.message = error
            self._rth_status.updated_at = time.time()

    def _ensure_guided_ready(self) -> bool:
        state = self._state_manager.get_state()
        if not state.armed:
            return False
        if state.flight_mode != "GUIDED":
            if not self.enable_guided_mode():
                return False
            if not self.wait_for_guided(timeout_s=3.0):
                return False
        return True

    def _send_guided_hold_current_position(self) -> bool:
        state = self._state_manager.get_state()
        if not state.has_valid_gps() or state.gps_alt is None:
            return self._send_stop_velocity()
        return self._mavlink.send_global_position_target(state.gps_lat, state.gps_lon, state.gps_alt)

    def _get_home_position(self) -> tuple[float | None, float | None, float | None]:
        if hasattr(self._mavlink, "request_home_position"):
            try:
                self._mavlink.request_home_position()
            except Exception:
                pass

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            state = self._state_manager.get_state()
            if state.home_lat is not None and state.home_lon is not None and state.home_alt is not None:
                return state.home_lat, state.home_lon, state.home_alt
            time.sleep(0.05)
        state = self._state_manager.get_state()
        return state.home_lat, state.home_lon, state.home_alt

    def _refresh_rth_progress_locked(self) -> None:
        status = self._rth_status
        state = self._state_manager.get_state()
        status.current_alt_msl = state.gps_alt
        status.current_alt_agl_m = state.alt_agl_m

        if state.has_valid_gps() and status.home_lat is not None and status.home_lon is not None:
            status.distance_to_home_m = self._haversine_m(
                state.gps_lat,
                state.gps_lon,
                status.home_lat,
                status.home_lon,
            )
        else:
            status.distance_to_home_m = None

        if state.gps_alt is not None and status.target_alt_msl is not None:
            status.altitude_error_m = status.target_alt_msl - state.gps_alt
        elif state.gps_alt is not None and status.phase in ("climb", "go_home"):
            status.altitude_error_m = ((status.home_alt_msl or 0.0) + status.climb_alt_m) - state.gps_alt
        else:
            status.altitude_error_m = None

        if status.paused or not status.active or status.awaiting_approval:
            return

        if (
            status.phase == "climb"
            and state.gps_alt is not None
            and status.home_alt_msl is not None
            and abs((status.home_alt_msl + status.climb_alt_m) - state.gps_alt) <= self.RTH_CLIMB_ALT_TOLERANCE_M
        ):
            status.awaiting_approval = True
            status.next_phase = "go_home"
            status.message = "At RTH altitude. Awaiting approval: go home"
            status.updated_at = time.time()
        elif (
            status.phase == "go_home"
            and status.distance_to_home_m is not None
            and status.distance_to_home_m <= self.RTH_HOME_RADIUS_M
        ):
            status.awaiting_approval = True
            status.next_phase = "descend"
            status.message = "At home point. Awaiting approval: controlled descent"
            status.updated_at = time.time()

    @staticmethod
    def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        radius_m = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = (
            math.sin(dphi / 2.0) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
        )
        return radius_m * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    
    def _run_loop(self) -> None:
        """Main control loop - handles timeouts and status updates."""
        loop_rate = 20.0  # Hz
        interval = 1.0 / loop_rate
        
        while not self._stop_event.is_set():
            try:
                start_time = time.monotonic()

                self._check_command_timeout()
                self._check_flight_mode()
                self._check_vio_freshness()
                self._update_rate()

                elapsed = time.monotonic() - start_time
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
            
            now = time.monotonic()
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

    def _check_vio_freshness(self) -> None:
        """Expire VIO health when updates stop arriving."""
        with self._lock:
            fresh = self._is_vio_fresh_locked()
            self._status.vio_age_ms = self._vio_age_ms_locked()
            self._status.vio_healthy = self._vio_healthy and fresh
            if self._vio_healthy and not fresh:
                self._status.health = NavHealth.DEGRADED
                if self._status.mode in (NavMode.VELOCITY, NavMode.POSITION, NavMode.VISUAL_SERVO):
                    logger.warning("VIO stale - stopping active navigation")
                    self._send_stop_velocity()
                    self._status.mode = NavMode.STANDBY
                    self._status.cmd_vx = 0.0
                    self._status.cmd_vy = 0.0
                    self._status.cmd_vz = 0.0
                    self._status.cmd_yaw_rate = 0.0
    
    def _update_rate(self) -> None:
        """Update command rate calculation."""
        now = time.monotonic()
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
                vio_age_ms=kwargs.get("vio_age_ms", self._status.vio_age_ms),
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
        if not math.isfinite(value):
            raise ValueError("Navigation command values must be finite")
        return max(min_val, min(max_val, value))

    @staticmethod
    def _read_positive_float(env_name: str, default: float) -> float:
        raw = os.environ.get(env_name, str(default)).strip()
        try:
            value = float(raw)
            if value <= 0.0:
                raise ValueError("value must be positive")
            return value
        except Exception:
            logger.warning(
                "Invalid %s='%s'; falling back to %.2fs",
                env_name,
                raw,
                default,
            )
            return default

    def _vio_age_ms_locked(self) -> int | None:
        if self._vio_last_update_monotonic is None:
            return None
        return int(max(0.0, time.monotonic() - self._vio_last_update_monotonic) * 1000)

    def _is_vio_fresh_locked(self) -> bool:
        if self._vio_last_update_monotonic is None:
            return False
        return (time.monotonic() - self._vio_last_update_monotonic) <= self._vio_max_age_s

    def _is_vio_ready_locked(self) -> bool:
        return (
            self._vio_healthy
            and self._vio_confidence >= self.MIN_VIO_CONFIDENCE
            and self._is_vio_fresh_locked()
        )
