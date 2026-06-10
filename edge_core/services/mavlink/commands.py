# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
from __future__ import annotations

import logging
import math
import os
import time

from pymavlink import mavutil

from edge_core.safety import Decision, FencePolicy, Point, evaluate_position

from ..geospatial import GPSCoordinate, calculate_gps_offset_meters

logger = logging.getLogger("edge_core.mavlink.commands")


def _load_fence_config() -> tuple[tuple[GPSCoordinate, ...] | None, float]:
    """Parse ``NOMAD_FENCE_POLYGON`` / ``NOMAD_FENCE_MARGIN_M`` (SR-FEN-02).

    Polygon format: semicolon-separated ``lat,lon`` vertices (WGS84 degrees,
    >= 3). Returns ``(None, margin)`` when unset — no NOMAD-side fence, the FC
    fence is the only enforcement. A malformed value returns an **empty**
    boundary, which rejects every position target: a broken fence config must
    fail closed, not silently fly unfenced.
    """
    try:
        margin = float(os.environ.get("NOMAD_FENCE_MARGIN_M", "2.0"))
        if not math.isfinite(margin) or margin < 0.0:
            raise ValueError(f"margin must be finite and >= 0, got {margin}")
    except ValueError as exc:
        logger.error("Invalid NOMAD_FENCE_MARGIN_M (%s) - rejecting all position targets", exc)
        return (), 0.0

    raw = os.environ.get("NOMAD_FENCE_POLYGON", "").strip()
    if not raw:
        return None, margin

    try:
        vertices = []
        for pair in raw.split(";"):
            lat_s, lon_s = pair.split(",")
            lat, lon = float(lat_s), float(lon_s)
            if not (math.isfinite(lat) and math.isfinite(lon)):
                raise ValueError(f"non-finite vertex {pair!r}")
            vertices.append(GPSCoordinate(lat, lon))
        if len(vertices) < 3:
            raise ValueError(f"need >= 3 vertices, got {len(vertices)}")
        return tuple(vertices), margin
    except ValueError as exc:
        logger.error("Invalid NOMAD_FENCE_POLYGON (%s) - rejecting all position targets", exc)
        return (), margin


class MavlinkCommands:
    """Implements outgoing MAVLink messages and commands."""

    def __init__(self) -> None:
        self._fence_latlon, self._fence_margin_m = _load_fence_config()
        if self._fence_latlon:
            logger.info(
                "NOMAD geofence active: %d vertices, %.1f m keep-in margin",
                len(self._fence_latlon),
                self._fence_margin_m,
            )

    def _fence_decision(
        self, boundary_latlon: tuple[GPSCoordinate, ...], ref: GPSCoordinate, target: Point
    ) -> Decision:
        """Project the lat/lon boundary into metres about ``ref`` and ask the SC core."""
        boundary = tuple(
            Point(offset.north, offset.east)
            for offset in (calculate_gps_offset_meters(ref, vertex) for vertex in boundary_latlon)
        )
        return evaluate_position(FencePolicy(boundary=boundary, margin=self._fence_margin_m), target)

    def _fence_allows_global(self, lat: float, lon: float) -> bool:
        """SR-FEN-02 gate for global (lat/lon) position targets."""
        if self._fence_latlon is None:
            return True
        if not self._fence_latlon:
            logger.warning("Geofence config invalid - rejecting global position target")
            return False
        ref = self._fence_latlon[0]
        offset = calculate_gps_offset_meters(ref, GPSCoordinate(lat, lon))
        decision = self._fence_decision(self._fence_latlon, ref, Point(offset.north, offset.east))
        if not decision.allowed:
            logger.warning("Geofence rejected global target (%.7f, %.7f): %s", lat, lon, decision.message)
        return decision.allowed

    def _fence_allows_local(self, north: float, east: float) -> bool:
        """SR-FEN-02 gate for LOCAL_NED position targets.

        LOCAL_NED is relative to the EKF origin; the FC home position is the
        closest available proxy for it. They can differ by metres — keep the
        fence margin larger than that separation. With a fence configured but
        no known home, containment cannot be verified, so the target is
        rejected.
        """
        if self._fence_latlon is None:
            return True
        if not self._fence_latlon:
            logger.warning("Geofence config invalid - rejecting local position target")
            return False
        state_manager = getattr(self, "state_manager", None)
        state = state_manager.get_state() if state_manager is not None else None
        if state is None or state.home_lat is None or state.home_lon is None:
            logger.warning("Geofence configured but home position unknown - rejecting local position target")
            return False
        home = GPSCoordinate(state.home_lat, state.home_lon)
        decision = self._fence_decision(self._fence_latlon, home, Point(north, east))
        if not decision.allowed:
            logger.warning("Geofence rejected local target (N=%.1f, E=%.1f): %s", north, east, decision.message)
        return decision.allowed

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
        if not self._fence_allows_global(lat, lon):
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
                round(float(lat) * 1e7),
                round(float(lon) * 1e7),
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
        if not self._fence_allows_local(x, y):
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
