"""
NOMAD Edge Core - Operational Mode Manager (Section 9)

Coordinates servo behavior, VIO source, nvblox configuration, and obstacle
avoidance profile as a unified operational mode. Each mode sets all subsystems
to a consistent state for the current phase of flight.

Modes:
    outdoor_transit  - GPS flight, servo level, 5cm voxels, 2m buffer
    outdoor_survey   - GPS flight, servo sweep, 5cm voxels, 2m buffer
    indoor_nav       - VIO flight, servo level (scan-stop), 3cm voxels, 0.7m buffer
    spray_approach   - Visual servoing, target sector excluded from avoidance
    emergency        - RC kill, all autonomy bypassed

nvblox config switching requires node restart (~2-3 seconds). The drone must
be hovering (velocity < 0.1 m/s) during a mode switch that changes nvblox config.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Optional

logger = logging.getLogger("edge_core.operational_mode")


class OpMode(Enum):
    """Operational modes per Section 9 requirements."""
    OUTDOOR_TRANSIT = "outdoor_transit"
    OUTDOOR_SURVEY = "outdoor_survey"
    INDOOR_NAV = "indoor_nav"
    SPRAY_APPROACH = "spray_approach"
    EMERGENCY = "emergency"


@dataclass
class ModeConfig:
    """Configuration for an operational mode."""
    servo_behavior: str          # "fixed_level", "sweep", "visual_servo", "none"
    servo_fixed_angle: float     # degrees (only used when servo_behavior == "fixed_level")
    vio_source: str              # "gps_ekf", "vio_cuvsslam", "mode_dependent"
    nvblox_config: str           # "performance" (outdoor) or "indoor"
    obstacle_buffer_m: float     # horizontal obstacle buffer in meters
    obstacle_vertical: bool      # True = also check vertical clearance
    scan_stop_enabled: bool      # VO-004: block servo tilt during motion
    depth_rate_hz: float         # nvblox depth integration rate
    description: str


# Mode configurations (Section 9)
MODE_CONFIGS: dict[OpMode, ModeConfig] = {
    OpMode.OUTDOOR_TRANSIT: ModeConfig(
        servo_behavior="fixed_level",
        servo_fixed_angle=90.0,   # level forward
        vio_source="gps_ekf",
        nvblox_config="performance",
        obstacle_buffer_m=2.0,
        obstacle_vertical=False,
        scan_stop_enabled=False,
        depth_rate_hz=10.0,
        description="Outdoor transit - GPS, servo level, 5cm voxels, 2m buffer",
    ),
    OpMode.OUTDOOR_SURVEY: ModeConfig(
        servo_behavior="sweep",
        servo_fixed_angle=90.0,
        vio_source="gps_ekf",
        nvblox_config="performance",
        obstacle_buffer_m=2.0,
        obstacle_vertical=False,
        scan_stop_enabled=False,  # GPS doesn't need scan-stop
        depth_rate_hz=10.0,
        description="Outdoor survey - GPS, servo sweep, 5cm voxels, 2m buffer",
    ),
    OpMode.INDOOR_NAV: ModeConfig(
        servo_behavior="fixed_level",
        servo_fixed_angle=90.0,
        vio_source="vio_cuvsslam",
        nvblox_config="indoor",
        obstacle_buffer_m=0.7,
        obstacle_vertical=True,
        scan_stop_enabled=True,  # VO-004: block tilt during motion
        depth_rate_hz=15.0,
        description="Indoor nav - VIO, servo level (scan-stop), 3cm voxels, 0.7m buffer",
    ),
    OpMode.SPRAY_APPROACH: ModeConfig(
        servo_behavior="visual_servo",
        servo_fixed_angle=90.0,
        vio_source="mode_dependent",  # GPS outdoors, VIO indoors
        nvblox_config="keep",         # don't change nvblox during spray
        obstacle_buffer_m=2.0,
        obstacle_vertical=False,
        scan_stop_enabled=False,      # visual servo needs free tilt
        depth_rate_hz=10.0,
        description="Spray approach - visual servoing, target sector excluded",
    ),
    OpMode.EMERGENCY: ModeConfig(
        servo_behavior="none",
        servo_fixed_angle=90.0,
        vio_source="none",
        nvblox_config="keep",
        obstacle_buffer_m=0.0,
        obstacle_vertical=False,
        scan_stop_enabled=False,
        depth_rate_hz=0.0,
        description="Emergency - RC kill, all autonomy bypassed",
    ),
}


@dataclass
class ModeStatus:
    """Current operational mode status."""
    current_mode: str = "outdoor_transit"
    previous_mode: str = ""
    last_switch_time: float = 0.0
    nvblox_restarting: bool = False
    switch_in_progress: bool = False
    error: Optional[str] = None

    def to_dict(self) -> dict:
        return {
            "current_mode": self.current_mode,
            "previous_mode": self.previous_mode,
            "last_switch_time": self.last_switch_time,
            "nvblox_restarting": self.nvblox_restarting,
            "switch_in_progress": self.switch_in_progress,
            "error": self.error,
        }


class OperationalModeManager:
    """
    Manages operational mode transitions for the NOMAD drone.

    Coordinates servo, VIO, nvblox, and obstacle avoidance when switching
    between outdoor transit, survey, indoor nav, spray approach, and emergency.
    """

    def __init__(
        self,
        servo_controller: Any = None,
        state_manager: Any = None,
        on_mode_change: Optional[Callable[[OpMode, ModeConfig], None]] = None,
    ):
        self._servo = servo_controller
        self._state = state_manager
        self._on_mode_change = on_mode_change

        self._current_mode = OpMode.OUTDOOR_TRANSIT
        self._status = ModeStatus()
        self._lock = threading.RLock()

        # nvblox restart callback (set by main.py or isaac_ros_bridge)
        self._nvblox_restart_fn: Optional[Callable[[str], bool]] = None

        # Obstacle distance bridge reference (for buffer updates)
        self._obstacle_bridge: Any = None

        logger.info(f"Operational mode manager initialized: {self._current_mode.value}")

    @property
    def current_mode(self) -> OpMode:
        with self._lock:
            return self._current_mode

    @property
    def current_config(self) -> ModeConfig:
        with self._lock:
            return MODE_CONFIGS[self._current_mode]

    @property
    def status(self) -> ModeStatus:
        with self._lock:
            return self._status

    def set_nvblox_restart_fn(self, fn: Callable[[str], bool]) -> None:
        """Set callback to restart nvblox with a specific config."""
        self._nvblox_restart_fn = fn

    def set_obstacle_bridge(self, bridge: Any) -> None:
        """Set obstacle distance bridge reference for buffer updates."""
        self._obstacle_bridge = bridge

    def switch_mode(self, target: OpMode | str) -> dict:
        """
        Switch to a new operational mode.

        Args:
            target: Target mode (OpMode enum or string name).

        Returns:
            dict with success status and details.
        """
        # Parse string mode name
        if isinstance(target, str):
            try:
                target = OpMode(target)
            except ValueError:
                return {"success": False, "error": f"Unknown mode: {target}"}

        with self._lock:
            if self._status.switch_in_progress:
                return {"success": False, "error": "Mode switch already in progress"}

            if target == self._current_mode:
                return {"success": True, "message": f"Already in {target.value}"}

            old_mode = self._current_mode
            old_config = MODE_CONFIGS[old_mode]
            new_config = MODE_CONFIGS[target]

            self._status.switch_in_progress = True
            self._status.error = None

        # Check if drone is hovering (required for nvblox restart)
        needs_nvblox_restart = (
            new_config.nvblox_config != "keep"
            and new_config.nvblox_config != old_config.nvblox_config
        )

        if needs_nvblox_restart and self._state:
            state = self._state.get_state()
            # Check velocity if available
            vel = getattr(state, 'groundspeed', None)
            if vel is not None and vel > 0.2:
                with self._lock:
                    self._status.switch_in_progress = False
                return {
                    "success": False,
                    "error": f"Drone must be hovering for nvblox config switch "
                             f"(current speed: {vel:.1f} m/s)",
                }

        try:
            result = self._apply_mode(target, new_config, needs_nvblox_restart)

            with self._lock:
                self._status.previous_mode = old_mode.value
                self._current_mode = target
                self._status.current_mode = target.value
                self._status.last_switch_time = time.time()
                self._status.switch_in_progress = False

            logger.info(f"Mode switch: {old_mode.value} -> {target.value}")

            if self._on_mode_change:
                self._on_mode_change(target, new_config)

            return {
                "success": True,
                "mode": target.value,
                "description": new_config.description,
                "nvblox_restarted": needs_nvblox_restart,
            }

        except Exception as e:
            with self._lock:
                self._status.switch_in_progress = False
                self._status.error = str(e)
            logger.error(f"Mode switch failed: {e}")
            return {"success": False, "error": str(e)}

    def _apply_mode(
        self,
        mode: OpMode,
        config: ModeConfig,
        restart_nvblox: bool,
    ) -> None:
        """Apply mode configuration to all subsystems."""

        # 1. Set servo behavior
        if config.servo_behavior == "fixed_level" and self._servo:
            self._servo.set_camera_tilt(config.servo_fixed_angle)
            logger.info(f"Servo set to fixed {config.servo_fixed_angle} deg")

        # 2. Restart nvblox with correct config if needed
        if restart_nvblox and self._nvblox_restart_fn:
            with self._lock:
                self._status.nvblox_restarting = True

            logger.info(f"Restarting nvblox with config: {config.nvblox_config}")
            success = self._nvblox_restart_fn(config.nvblox_config)

            with self._lock:
                self._status.nvblox_restarting = False

            if not success:
                logger.error("nvblox restart failed")

        # 3. Update obstacle buffer distance
        if self._obstacle_bridge and hasattr(self._obstacle_bridge, 'set_obstacle_buffer'):
            self._obstacle_bridge.set_obstacle_buffer(config.obstacle_buffer_m)

        # 4. Update scan-stop-scan setting in bridge state
        if self._state:
            try:
                self._state.update_state(
                    scan_stop_enabled=config.scan_stop_enabled,
                    operational_mode=mode.value,
                )
            except Exception:
                pass

    def get_available_modes(self) -> list[dict]:
        """Get list of available modes with descriptions."""
        return [
            {
                "mode": mode.value,
                "description": config.description,
                "current": mode == self._current_mode,
            }
            for mode, config in MODE_CONFIGS.items()
        ]


# Module-level instance
_manager: Optional[OperationalModeManager] = None


def init_mode_manager(
    servo_controller: Any = None,
    state_manager: Any = None,
    on_mode_change: Optional[Callable] = None,
) -> OperationalModeManager:
    """Initialize the global operational mode manager."""
    global _manager
    _manager = OperationalModeManager(
        servo_controller=servo_controller,
        state_manager=state_manager,
        on_mode_change=on_mode_change,
    )
    return _manager


def get_mode_manager() -> Optional[OperationalModeManager]:
    """Get the global operational mode manager."""
    return _manager
