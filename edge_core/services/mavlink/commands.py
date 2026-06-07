# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
from __future__ import annotations

import logging
import time

from pymavlink import mavutil

logger = logging.getLogger("edge_core.mavlink.commands")


class MavlinkCommands:
    """Implements outgoing MAVLink messages and commands."""

    def arm_disarm(self, should_arm: bool) -> None:
        if not hasattr(self, "_conn") or self._conn is None:  # type: ignore
            return
        try:
            self._send_command_long_and_wait_ack(  # type: ignore
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                1 if should_arm else 0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        except Exception as exc:
            logger.debug("Arm/disarm command error: %s", exc)

    def send_vision_position_estimate(
        self,
        timestamp_us: int,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        covariance: list[float] | None = None,
        reset_counter: int = 0,
    ) -> bool:
        """Send VISION_POSITION_ESTIMATE message to the flight controller."""
        if not hasattr(self, "_conn") or self._conn is None:  # type: ignore
            return False

        try:
            if covariance is None:
                covariance = [
                    0.01,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0.01,
                    0,
                    0,
                    0,
                    0,
                    0.01,
                    0,
                    0,
                    0,
                    0.01,
                    0,
                    0,
                    0.01,
                    0,
                    0.01,
                ]

            self._conn.mav.vision_position_estimate_send(  # type: ignore
                timestamp_us,
                x,
                y,
                z,
                roll,
                pitch,
                yaw,
                covariance,
                reset_counter,
            )
            return True
        except Exception as e:
            logger.debug("VIO send error: %s", e)
            return False

    def send_vision_speed_estimate(
        self,
        timestamp_us: int,
        vx: float,
        vy: float,
        vz: float,
        covariance: list[float] | None = None,
        reset_counter: int = 0,
    ) -> bool:
        """Send VISION_SPEED_ESTIMATE message to the flight controller."""
        if not hasattr(self, "_conn") or self._conn is None:  # type: ignore
            return False

        try:
            if covariance is None:
                covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

            self._conn.mav.vision_speed_estimate_send(  # type: ignore
                timestamp_us,
                vx,
                vy,
                vz,
                covariance,
                reset_counter,
            )
            return True
        except Exception as e:
            logger.debug("Vision speed send error: %s", e)
            return False

    def send_velocity_command(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float = 0.0,
        coordinate_frame: int | None = None,
    ) -> bool:
        """Send velocity command via SET_POSITION_TARGET_LOCAL_NED."""
        if not hasattr(self, "_conn") or self._conn is None:  # type: ignore
            return False

        try:
            if coordinate_frame is None:
                coordinate_frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED

            type_mask = 0b0000_0111_1100_0111

            self._conn.mav.set_position_target_local_ned_send(  # type: ignore
                0,
                self._conn.target_system,  # type: ignore
                self._conn.target_component,  # type: ignore
                coordinate_frame,
                type_mask,
                0,
                0,
                0,
                vx,
                vy,
                vz,
                0,
                0,
                0,
                0,
                yaw_rate,
            )
            return True
        except Exception as e:
            logger.debug("Velocity command error: %s", e)
            return False

    def send_gimbal_command(
        self,
        pitch: float,
        yaw: float,
        roll: float = 0.0,
    ) -> bool:
        """Send gimbal control command via COMMAND_LONG (MAV_CMD_DO_MOUNT_CONTROL)."""
        if not hasattr(self, "_conn") or self._conn is None:  # type: ignore
            return False

        try:
            self._conn.mav.command_long_send(  # type: ignore
                self._conn.target_system,  # type: ignore
                self._conn.target_component,  # type: ignore
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                0,
                pitch,
                roll,
                yaw,
                0,
                0,
                0,
                mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
            )
            return True
        except Exception as e:
            logger.debug("Gimbal command error: %s", e)
            return False

    def send_gimbal_rate_command(
        self,
        pitch_rate: float,
        yaw_rate: float,
        roll_rate: float = 0.0,
    ) -> bool:
        """Send gimbal angular rate command."""
        if not hasattr(self, "_conn") or self._conn is None:  # type: ignore
            return False

        try:
            self._conn.mav.command_long_send(  # type: ignore
                self._conn.target_system,  # type: ignore
                self._conn.target_component,  # type: ignore
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                0,
                pitch_rate,
                yaw_rate,
                pitch_rate,
                yaw_rate,
                0,
                0,
                0,
            )
            return True
        except Exception as e:
            logger.debug("Gimbal rate command error: %s", e)
            return False

    def trigger_payload(
        self,
        pwm_value: int,
        servo_channel: int = 9,
    ) -> bool:
        """Trigger payload (water pump) via MAV_CMD_DO_SET_SERVO."""
        if not hasattr(self, "_conn") or self._conn is None:  # type: ignore
            return False

        try:
            return self._send_command_long_and_wait_ack(  # type: ignore
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                servo_channel,
                pwm_value,
                0,
                0,
                0,
                0,
                0,
            )
        except Exception as e:
            logger.debug("Payload trigger error: %s", e)
            return False

    def set_relay(self, relay_number: int, enabled: bool) -> bool:
        """Set a Cube Orange relay through MAV_CMD_DO_SET_RELAY."""
        if not hasattr(self, "_conn") or self._conn is None:  # type: ignore
            return False

        try:
            return self._send_command_long_and_wait_ack(  # type: ignore
                mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                int(relay_number),
                1 if enabled else 0,
                0,
                0,
                0,
                0,
                0,
            )
        except Exception as e:
            logger.debug("Relay command error: %s", e)
            return False

    def stop_velocity(self) -> bool:
        """Send zero velocity command to stop movement."""
        return self.send_velocity_command(0.0, 0.0, 0.0, 0.0)

    def send_statustext(
        self,
        text: str,
        severity: int | None = None,
    ) -> bool:
        """Send STATUSTEXT message to GCS."""
        if not hasattr(self, "_conn") or self._conn is None:  # type: ignore
            return False

        try:
            if severity is None:
                severity = mavutil.mavlink.MAV_SEVERITY_INFO

            text = text[:50]
            self._conn.mav.statustext_send(  # type: ignore
                severity,
                text.encode("utf-8"),
            )
            return True
        except Exception as e:
            logger.debug("Statustext error: %s", e)
            return False

    def set_mode(self, mode_id: int) -> bool:
        """Set the flight mode on the autopilot."""
        if not hasattr(self, "_conn") or self._conn is None:  # type: ignore
            return False

        try:
            return self._send_command_long_and_wait_ack(  # type: ignore
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                1,
                mode_id,
                0,
                0,
                0,
                0,
                0,
            )
        except Exception as e:
            logger.debug("Set mode error: %s", e)
            return False

    def takeoff(self, altitude_m: float) -> bool:
        """Command an autonomous takeoff to ``altitude_m`` AGL."""
        if not hasattr(self, "_conn") or self._conn is None:  # type: ignore
            return False
        try:
            return self._send_command_long_and_wait_ack(  # type: ignore
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                float(altitude_m),
                timeout_s=2.0,
            )
        except Exception as e:
            logger.debug("Takeoff error: %s", e)
            return False

    def land(self) -> bool:
        """Switch the autopilot to LAND mode for an autonomous descent."""
        return self.set_mode(9)

    def request_home_position(self) -> bool:
        """Ask ArduPilot to send HOME_POSITION once."""
        if not hasattr(self, "_conn") or self._conn is None:  # type: ignore
            return False
        try:
            return self._send_command_long_and_wait_ack(  # type: ignore
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
                0,
                0,
                0,
                0,
                0,
                0,
                timeout_s=1.0,
            )
        except Exception as e:
            logger.debug("Home position request error: %s", e)
            return False

    def send_global_position_target(
        self,
        lat: float,
        lon: float,
        alt_msl: float,
        yaw: float | None = None,
    ) -> bool:
        """Send a GUIDED global position target in WGS84 / MSL."""
        if not hasattr(self, "_conn") or self._conn is None:  # type: ignore
            return False

        try:
            type_mask = 0b0000_1111_1111_1000
            if yaw is not None:
                type_mask &= ~(1 << 10)

            self._conn.mav.set_position_target_global_int_send(  # type: ignore
                0,
                self._conn.target_system,  # type: ignore
                self._conn.target_component,  # type: ignore
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                type_mask,
                int(round(float(lat) * 1e7)),
                int(round(float(lon) * 1e7)),
                float(alt_msl),
                0,
                0,
                0,
                0,
                0,
                0,
                float(yaw or 0.0),
                0,
            )
            return True
        except Exception as e:
            logger.debug("Global position target error: %s", e)
            return False

    def send_position_target(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
    ) -> bool:
        """Send position target via SET_POSITION_TARGET_LOCAL_NED."""
        if not hasattr(self, "_conn") or self._conn is None:  # type: ignore
            return False

        try:
            type_mask = 0b0000_1111_1111_1000
            self._conn.mav.set_position_target_local_ned_send(  # type: ignore
                0,
                self._conn.target_system,  # type: ignore
                self._conn.target_component,  # type: ignore
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                x,
                y,
                z,
                0,
                0,
                0,
                0,
                0,
                0,
                yaw,
                0,
            )
            return True
        except Exception as e:
            logger.debug("Position target error: %s", e)
            return False

    def send_obstacle_distance(
        self,
        distances: list[int],
        increment: int = 5,
        min_distance: int = 20,
        max_distance: int = 2000,
        angle_offset: int = 0,
        frame: int = 0,
    ) -> bool:
        """Send OBSTACLE_DISTANCE message to ArduPilot."""
        if not hasattr(self, "_conn") or self._conn is None:  # type: ignore
            return False

        try:
            dist_array = list(distances[:72])
            while len(dist_array) < 72:
                dist_array.append(max_distance)

            self._conn.mav.obstacle_distance_send(  # type: ignore
                int(time.time() * 1e3) & 0xFFFFFFFF,
                0,
                dist_array,
                increment,
                min_distance,
                max_distance,
                float(increment),
                float(angle_offset),
                frame,
            )
            return True
        except Exception as e:
            logger.debug("Obstacle distance error: %s", e)
            return False
