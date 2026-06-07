#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""MAVLink-only servo and relay controller for NOMAD.

All payload, camera tilt, and water pump hardware is driven by the Cube Orange.
The Jetson does not own any local GPIO or PWM lines for servos.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any

logger = logging.getLogger(__name__)


class ServoFunction(Enum):
    """Servo functions tracked by Edge Core."""

    CAMERA_TILT = "camera_tilt"


@dataclass
class ServoState:
    """Current state for a Cube-controlled output."""

    angle: float
    enabled: bool
    last_update: float


class MavlinkServo:
    """Servo controlled through MAV_CMD_DO_SET_SERVO on the Cube Orange."""

    def __init__(
        self,
        name: str,
        mavlink_service: Any | None,
        channel: int,
        *,
        min_angle: float = 0.0,
        max_angle: float = 180.0,
        min_pulse_us: int = 500,
        max_pulse_us: int = 2500,
        neutral_angle: float = 90.0,
    ) -> None:
        self.name = name
        self._mav = mavlink_service
        self._channel = int(channel)
        self._min_angle = float(min_angle)
        self._max_angle = float(max_angle)
        self._min_pulse_us = int(min_pulse_us)
        self._max_pulse_us = int(max_pulse_us)
        self._current_angle = float(neutral_angle)
        self._enabled = False

    @property
    def channel(self) -> int:
        return self._channel

    def initialize(self) -> bool:
        if self._channel < 1 or self._channel > 16:
            logger.error(f"Invalid MAVLink servo channel: {self._channel}")
            return False
        self._enabled = True
        logger.info(f"MAVLink servo {self.name} configured on Cube channel {self._channel}")
        return True

    def enable(self) -> bool:
        self._enabled = True
        return True

    def disable(self) -> bool:
        self._enabled = False
        return True

    def set_angle(self, angle: float) -> bool:
        angle = max(self._min_angle, min(self._max_angle, float(angle)))
        pulse_us = self._angle_to_pulse_us(angle)
        self._current_angle = angle
        return self.set_pwm(pulse_us)

    def set_pwm(self, pulse_us: int) -> bool:
        pulse_us = int(pulse_us)
        if self._mav is None:
            logger.warning("MAVLink service not available for servo command")
            return False
        success = self._mav.trigger_payload(pulse_us, self._channel)
        if not success:
            logger.debug(f"MAVLink servo command failed (channel={self._channel}, pwm={pulse_us})")
        return success

    def get_state(self) -> ServoState:
        return ServoState(
            angle=self._current_angle,
            enabled=self._enabled,
            last_update=time.time(),
        )

    def _angle_to_pulse_us(self, angle: float) -> int:
        span = max(self._max_angle - self._min_angle, 1.0)
        normalized = (angle - self._min_angle) / span
        pulse_range = self._max_pulse_us - self._min_pulse_us
        return int(self._min_pulse_us + normalized * pulse_range)


class ServoController:
    """MAVLink-only controller for Cube Orange servo and relay outputs."""

    def __init__(self) -> None:
        self._mavlink_service: Any | None = None
        self._servos: dict[ServoFunction, MavlinkServo] = {}
        self._initialized = False
        self._camera_tilt_channel: int | None = None
        self._water_pump_relay_number: int = 0
        self._last_relay_trigger: float = 0.0

    def initialize(self) -> bool:
        self._initialized = True
        logger.info("Servo controller initialized in Cube Orange MAVLink-only mode")
        return True

    def is_available(self) -> bool:
        return self._initialized

    def configure_camera_tilt_mavlink(self, channel: int) -> bool:
        if channel < 1 or channel > 16:
            logger.error(f"Invalid camera tilt servo channel: {channel}")
            return False

        old_servo = self._servos.get(ServoFunction.CAMERA_TILT)
        last_angle = 90.0
        if old_servo is not None:
            last_angle = old_servo.get_state().angle

        servo = MavlinkServo(
            "camera_tilt",
            mavlink_service=self._mavlink_service,
            channel=channel,
            neutral_angle=last_angle,
        )
        if not servo.initialize():
            return False

        self._camera_tilt_channel = channel
        self._servos[ServoFunction.CAMERA_TILT] = servo
        return True

    def configure_water_pump_relay(self, relay_number: int) -> bool:
        if relay_number < 0 or relay_number > 15:
            logger.error(f"Invalid water pump relay number: {relay_number}")
            return False
        self._water_pump_relay_number = int(relay_number)
        logger.info(f"Water pump configured on Cube relay {self._water_pump_relay_number}")
        return True

    def set_channel_pwm(self, channel: int, pwm_us: int) -> bool:
        if channel < 1 or channel > 16:
            logger.error(f"Invalid Cube servo channel: {channel}")
            return False
        if pwm_us < 500 or pwm_us > 2500:
            logger.error(f"Invalid Cube servo PWM: {pwm_us}")
            return False
        if self._mavlink_service is None:
            logger.warning("MAVLink service not available for Cube servo command")
            return False
        return self._mavlink_service.trigger_payload(int(pwm_us), int(channel))

    def set_camera_tilt(self, angle: float) -> bool:
        servo = self._servos.get(ServoFunction.CAMERA_TILT)
        if servo is None:
            logger.warning("Camera tilt channel not configured by Mission Planner")
            return False
        return servo.set_angle(angle)

    def get_camera_tilt(self) -> float | None:
        servo = self._servos.get(ServoFunction.CAMERA_TILT)
        if servo is None:
            return None
        return servo.get_state().angle

    def trigger_water_shooter(self, duration_ms: int = 200) -> bool:
        if self._mavlink_service is None:
            logger.warning("MAVLink service not available for Cube relay command")
            return False

        duration_s = max(0.05, min(float(duration_ms) / 1000.0, 5.0))
        relay = self._water_pump_relay_number
        if not self._mavlink_service.set_relay(relay, True):
            # The command may still have reached the FC even if COMMAND_ACK was
            # lost or negative. Always issue a best-effort off command before
            # reporting failure so the pump cannot be left energized.
            try:
                self._mavlink_service.set_relay(relay, False)
            except Exception:
                pass
            return False
        self._last_relay_trigger = time.time()
        try:
            time.sleep(duration_s)
        finally:
            self._mavlink_service.set_relay(relay, False)
        return True

    def enable_all(self) -> None:
        for servo in self._servos.values():
            servo.enable()

    def disable_all(self) -> None:
        for servo in self._servos.values():
            servo.disable()

    def get_status(self) -> dict:
        status: dict[str, Any] = {
            "initialized": self._initialized,
            "mode": "cube_orange_mavlink",
            "servo_count": len(self._servos),
            "relay_count": 1,
            "servos": {},
            "relays": {
                "water_shooter": {
                    "relay_number": self._water_pump_relay_number,
                    "last_trigger": self._last_relay_trigger,
                    "type": "mavlink_relay",
                }
            },
        }

        for function, servo in self._servos.items():
            state = servo.get_state()
            status["servos"][function.value] = {
                "angle": state.angle,
                "enabled": state.enabled,
                "last_update": state.last_update,
                "type": "mavlink",
                "channel": servo.channel,
            }

        return status

    def shutdown(self) -> None:
        self.disable_all()
        self._initialized = False


_controller: ServoController | None = None


def init_servo_controller(
    mavlink_service: Any | None = None,
    camera_tilt_channel: int | None = None,
) -> bool:
    """Initialize the global MAVLink-only servo controller."""
    global _controller
    if _controller is None:
        _controller = ServoController()

    _controller._mavlink_service = mavlink_service
    initialized = _controller.initialize()
    if camera_tilt_channel is not None and camera_tilt_channel > 0:
        _controller.configure_camera_tilt_mavlink(int(camera_tilt_channel))
    return initialized


def get_servo_controller() -> ServoController | None:
    """Get the global servo controller instance."""
    return _controller


def shutdown_servo_controller() -> None:
    """Shutdown the global servo controller."""
    global _controller
    if _controller:
        _controller.shutdown()
        _controller = None
