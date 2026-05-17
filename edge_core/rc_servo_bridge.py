"""
RC Channel to Servo Bridge.

Maps an RC channel from the ELRS controller (received as MAVLink RC_CHANNELS)
to the nozzle servo on Pin 15. A knob or slider on the controller controls
the servo angle in real time.

Architecture:
  ELRS TX (knob) -> ELRS RX -> Flight Controller -> MAVLink RC_CHANNELS
  -> Jetson MavlinkService -> RCServoBridge -> ServoController -> GPIO PWM

Configuration:
  RC_CHANNEL:       Which channel to read (1-18, default 9)
  RC_MIN/RC_MAX:    Expected RC PWM range  (default 1000-2000 us)
  SERVO_MIN/MAX:    Target servo angle range (default 0-180 deg)
  DEADBAND:         Ignore changes smaller than this (default 5 us)
  UPDATE_RATE_HZ:   Max servo updates per second (default 20)
"""

import logging
import time
import threading
from dataclasses import dataclass, field
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class RCServoConfig:
    """Configuration for the RC-to-servo mapping."""
    rc_channel: int = 9            # RC channel number (1-18)
    rc_min: int = 1000             # RC PWM low end (us)
    rc_max: int = 2000             # RC PWM high end (us)
    servo_min_angle: float = 0.0   # Servo angle at rc_min
    servo_max_angle: float = 180.0 # Servo angle at rc_max
    deadband: int = 5              # Ignore RC changes smaller than this (us)
    update_rate_hz: float = 20.0   # Max servo update rate
    inverted: bool = False         # If True, rc_max -> servo_min_angle
    enabled: bool = True           # Master enable switch


class RCServoBridge:
    """
    Bridges RC channel input to servo output.

    Call `on_rc_channels()` every time an RC_CHANNELS MAVLink message arrives.
    The bridge rate-limits and deadbands the updates before commanding the servo.
    """

    def __init__(self, config: Optional[RCServoConfig] = None):
        self.config = config or RCServoConfig()
        self._last_rc_value: int = 0
        self._last_update_time: float = 0.0
        self._last_commanded_angle: Optional[float] = None
        self._servo_controller = None
        self._active = False
        self._lock = threading.Lock()

    def start(self, servo_controller) -> bool:
        """
        Start the bridge with a reference to the servo controller.

        Args:
            servo_controller: ServoController instance from servo_controller.py

        Returns:
            True if started successfully.
        """
        if not self.config.enabled:
            logger.info("RC servo bridge disabled by config")
            return False

        if servo_controller is None or not servo_controller.is_available():
            logger.warning("RC servo bridge: servo controller not available")
            return False

        self._servo_controller = servo_controller
        self._active = True
        logger.info(
            f"RC servo bridge started: channel {self.config.rc_channel}, "
            f"RC [{self.config.rc_min}-{self.config.rc_max}] -> "
            f"servo [{self.config.servo_min_angle}-{self.config.servo_max_angle}] deg"
        )
        return True

    def stop(self):
        """Stop the bridge."""
        self._active = False
        self._servo_controller = None
        logger.info("RC servo bridge stopped")

    @property
    def is_active(self) -> bool:
        return self._active and self._servo_controller is not None

    def set_channel(self, channel: int) -> None:
        """Thread-safe channel change. Resets deadband state for clean transition."""
        with self._lock:
            self.config.rc_channel = channel
            self._last_rc_value = 0  # reset deadband for new channel

    def on_rc_channels(self, msg) -> None:
        """
        Process an RC_CHANNELS MAVLink message.

        Extracts the configured channel value and commands the servo
        if the value changed beyond the deadband and rate limit allows.

        Args:
            msg: pymavlink RC_CHANNELS message (type 65)
        """
        if not self._active or self._servo_controller is None:
            return

        # RC_CHANNELS has chan1_raw through chan18_raw
        channel_attr = f"chan{self.config.rc_channel}_raw"
        rc_value = getattr(msg, channel_attr, 0)

        # Ignore invalid/missing channel data (0 or 65535 = not available)
        if rc_value == 0 or rc_value == 65535:
            return

        with self._lock:
            # Deadband: skip if change is too small
            if abs(rc_value - self._last_rc_value) < self.config.deadband:
                return

            # Rate limit
            now = time.monotonic()
            min_interval = 1.0 / max(self.config.update_rate_hz, 1.0)
            if (now - self._last_update_time) < min_interval:
                return

            self._last_rc_value = rc_value
            self._last_update_time = now

        # Map RC value to servo angle
        angle = self._rc_to_angle(rc_value)

        # Command the servo
        success = self._servo_controller.set_camera_tilt(angle)
        if success:
            self._last_commanded_angle = angle
            logger.debug(
                f"RC ch{self.config.rc_channel}={rc_value} -> servo {angle:.1f} deg"
            )

    def _rc_to_angle(self, rc_value: int) -> float:
        """Map RC PWM value to servo angle with clamping."""
        # Clamp to configured range
        rc_clamped = max(self.config.rc_min, min(self.config.rc_max, rc_value))

        # Normalize to 0.0 - 1.0
        rc_range = self.config.rc_max - self.config.rc_min
        if rc_range <= 0:
            return self.config.servo_min_angle

        normalized = (rc_clamped - self.config.rc_min) / rc_range

        # Invert if configured
        if self.config.inverted:
            normalized = 1.0 - normalized

        # Scale to servo angle range
        angle_range = self.config.servo_max_angle - self.config.servo_min_angle
        angle = self.config.servo_min_angle + (normalized * angle_range)

        return round(angle, 1)

    def get_status(self) -> dict:
        """Get current bridge status."""
        return {
            "active": self._active,
            "enabled": self.config.enabled,
            "rc_channel": self.config.rc_channel,
            "last_rc_value": self._last_rc_value,
            "last_commanded_angle": self._last_commanded_angle,
            "rc_range": [self.config.rc_min, self.config.rc_max],
            "servo_range": [self.config.servo_min_angle, self.config.servo_max_angle],
            "inverted": self.config.inverted,
            "update_rate_hz": self.config.update_rate_hz,
        }


# Singleton instance
_bridge: Optional[RCServoBridge] = None


def init_rc_servo_bridge(
    servo_controller,
    rc_channel: int = 9,
    enabled: bool = True,
) -> Optional[RCServoBridge]:
    """
    Initialize the RC servo bridge singleton.

    Args:
        servo_controller: ServoController instance
        rc_channel: RC channel number to read (1-18)
        enabled: Whether to enable the bridge

    Returns:
        RCServoBridge instance if successful, None otherwise.
    """
    global _bridge
    config = RCServoConfig(rc_channel=rc_channel, enabled=enabled)
    _bridge = RCServoBridge(config)
    if _bridge.start(servo_controller):
        return _bridge
    _bridge = None
    return None


def get_rc_servo_bridge() -> Optional[RCServoBridge]:
    """Get the singleton bridge instance."""
    return _bridge


def shutdown_rc_servo_bridge() -> None:
    """Shutdown the bridge."""
    global _bridge
    if _bridge:
        _bridge.stop()
        _bridge = None
