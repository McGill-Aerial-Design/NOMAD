#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""MAVLink-only servo and relay controller for NOMAD.

All payload, camera tilt, and water pump hardware is driven by the Cube Orange.
The Jetson does not own any local GPIO or PWM lines for servos.

Tier SC adapter (requirements SR-PAY-01/02/03, hazard H-06): the channel/PWM
ranges, duration clamp, and the arm->release interlock are decided by the pure
``edge_core.safety.payload`` core; this module owns the I/O — MAVLink
transmission, the interlock state under its lock, and the
de-energize-in-``finally`` guarantee on the pump path.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any

from edge_core.safety import (
    MAX_SERVO_CHANNEL,
    MIN_SERVO_CHANNEL,
    InterlockPolicy,
    InterlockState,
    arm_release,
    clamp_release_duration,
    evaluate_release,
    validate_servo_command,
)

logger = logging.getLogger(__name__)


CAMERA_TILT = "camera_tilt"


@dataclass
class ServoState:
    """Current state for a Cube-controlled output."""

    angle: float
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

    @property
    def channel(self) -> int:
        return self._channel

    def initialize(self) -> bool:
        if self._channel < MIN_SERVO_CHANNEL or self._channel > MAX_SERVO_CHANNEL:
            logger.error("Invalid MAVLink servo channel: %s", self._channel)
            return False
        logger.info("MAVLink servo %s configured on Cube channel %s", self.name, self._channel)
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
            logger.debug("MAVLink servo command failed (channel=%s, pwm=%s)", self._channel, pulse_us)
        return success

    def get_state(self) -> ServoState:
        return ServoState(
            angle=self._current_angle,
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
        self._servos: dict[str, MavlinkServo] = {}
        self._initialized = False
        self._camera_tilt_channel: int | None = None
        self._water_pump_relay_number: int = 0
        self._last_relay_trigger: float = 0.0
        self._interlock_policy = InterlockPolicy()
        self._interlock_state = InterlockState()
        self._interlock_lock = threading.Lock()

    def initialize(self) -> bool:
        self._initialized = True
        logger.info("Servo controller initialized in Cube Orange MAVLink-only mode")
        return True

    def is_available(self) -> bool:
        return self._initialized

    def configure_camera_tilt_mavlink(self, channel: int) -> bool:
        if channel < MIN_SERVO_CHANNEL or channel > MAX_SERVO_CHANNEL:
            logger.error("Invalid camera tilt servo channel: %s", channel)
            return False

        old_servo = self._servos.get(CAMERA_TILT)
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
        self._servos[CAMERA_TILT] = servo
        return True

    def configure_water_pump_relay(self, relay_number: int) -> bool:
        if relay_number < 0 or relay_number > 15:
            logger.error("Invalid water pump relay number: %s", relay_number)
            return False
        self._water_pump_relay_number = int(relay_number)
        logger.info("Water pump configured on Cube relay %s", self._water_pump_relay_number)
        return True

    def set_channel_pwm(self, channel: int, pwm_us: int) -> bool:
        decision = validate_servo_command(channel, pwm_us)
        if not decision.allowed:
            logger.error("%s", decision.message)
            return False
        if self._mavlink_service is None:
            logger.warning("MAVLink service not available for Cube servo command")
            return False
        return self._mavlink_service.trigger_payload(int(pwm_us), int(channel))

    def set_camera_tilt(self, angle: float) -> bool:
        servo = self._servos.get(CAMERA_TILT)
        if servo is None:
            logger.warning("Camera tilt channel not configured by Mission Planner")
            return False
        return servo.set_angle(angle)

    def get_camera_tilt(self) -> float | None:
        servo = self._servos.get(CAMERA_TILT)
        if servo is None:
            return None
        return servo.get_state().angle

    def arm_release(self) -> float:
        """Open the release interlock window (SR-PAY-03); returns its length in s."""
        with self._interlock_lock:
            self._interlock_state = arm_release(time.monotonic())
        return self._interlock_policy.arm_window_s

    def trigger_water_shooter(self, duration_ms: int = 200) -> bool:
        # Interlock first: the arm is consumed whether or not the release goes
        # on to succeed, so every attempt needs a fresh, deliberate arm.
        with self._interlock_lock:
            self._interlock_state, decision = evaluate_release(
                self._interlock_policy, self._interlock_state, time.monotonic()
            )
        if not decision.allowed:
            logger.warning("%s", decision.message)
            return False

        if self._mavlink_service is None:
            logger.warning("MAVLink service not available for Cube relay command")
            return False

        try:
            duration_s = clamp_release_duration(float(duration_ms) / 1000.0)
        except ValueError:
            logger.error("Non-finite water shooter duration - rejected")
            return False
        relay = self._water_pump_relay_number
        if not self._mavlink_service.set_relay(relay, True):
            # The command may still have reached the FC even if COMMAND_ACK was
            # lost or negative. Always issue a best-effort off command before
            # reporting failure so the pump cannot be left energized.
            try:
                self._mavlink_service.set_relay(relay, False)
            except Exception as e:
                logger.debug("Best-effort relay-off after failed on-command also failed: %s", e)
            return False
        self._last_relay_trigger = time.time()
        try:
            time.sleep(duration_s)
        finally:
            self._mavlink_service.set_relay(relay, False)
        return True

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
            status["servos"][function] = {
                "angle": state.angle,
                "last_update": state.last_update,
                "type": "mavlink",
                "channel": servo.channel,
            }

        return status

    def shutdown(self) -> None:
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
