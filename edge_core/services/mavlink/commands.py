# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
from __future__ import annotations

import logging
import math
import os
from collections.abc import Callable
from typing import Any

from pymavlink import mavutil

from edge_core.safety import VELOCITY_TYPE_MASK, Decision, FencePolicy, Point, evaluate_position

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
    """Implements outgoing MAVLink messages and commands.

    Mixin: ``MavlinkService(MavlinkConnection, MavlinkCommands)``. The declared
    attributes below are the contract the connection class must provide; the
    class-level defaults keep a bare ``MavlinkCommands`` (tests) fail-closed.
    """

    # Provided by MavlinkConnection: the pymavlink link and the shared state
    # manager (home position for the local-NED fence gate).
    _conn: Any = None
    state_manager: Any = None

    def _send_command_long_and_wait_ack(self, command_id: int, *params: float, timeout_s: float = 0.75) -> bool:
        raise NotImplementedError("provided by MavlinkConnection")

    def _send_guarded(self, label: str, send: Callable[[], Any]) -> bool:
        """Run ``send`` against a live link; nothing is transmitted on failure.

        Returns False when disconnected or on any transmit error (logged at
        debug, matching the historical per-method behavior); a ``send`` that
        returns a value (the ack result) is passed through as bool.
        """
        if self._conn is None:
            return False
        try:
            result = send()
            return True if result is None else bool(result)
        except Exception as exc:
            logger.debug("%s error: %s", label, exc)
            return False

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
        state = self.state_manager.get_state() if self.state_manager is not None else None
        if state is None or state.home_lat is None or state.home_lon is None:
            logger.warning("Geofence configured but home position unknown - rejecting local position target")
            return False
        home = GPSCoordinate(state.home_lat, state.home_lon)
        decision = self._fence_decision(self._fence_latlon, home, Point(north, east))
        if not decision.allowed:
            logger.warning("Geofence rejected local target (N=%.1f, E=%.1f): %s", north, east, decision.message)
        return decision.allowed

    def arm_disarm(self, should_arm: bool) -> bool:
        return self._send_guarded(
            "Arm/disarm command",
            lambda: self._send_command_long_and_wait_ack(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                1 if should_arm else 0,
            ),
        )

    def send_velocity_command(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float = 0.0,
        coordinate_frame: int | None = None,
    ) -> bool:
        """Send velocity command via SET_POSITION_TARGET_LOCAL_NED."""

        def _send() -> None:
            frame = coordinate_frame if coordinate_frame is not None else mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED
            self._conn.mav.set_position_target_local_ned_send(
                0,
                self._conn.target_system,
                self._conn.target_component,
                frame,
                VELOCITY_TYPE_MASK,
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

        return self._send_guarded("Velocity command", _send)

    def trigger_payload(
        self,
        pwm_value: int,
        servo_channel: int = 9,
    ) -> bool:
        """Trigger payload (water pump) via MAV_CMD_DO_SET_SERVO."""
        return self._send_guarded(
            "Payload trigger",
            lambda: self._send_command_long_and_wait_ack(
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                servo_channel,
                pwm_value,
            ),
        )

    def set_relay(self, relay_number: int, enabled: bool) -> bool:
        """Set a Cube Orange relay through MAV_CMD_DO_SET_RELAY."""
        return self._send_guarded(
            "Relay command",
            lambda: self._send_command_long_and_wait_ack(
                mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                int(relay_number),
                1 if enabled else 0,
            ),
        )

    def stop_velocity(self) -> bool:
        """Send zero velocity command to stop movement."""
        return self.send_velocity_command(0.0, 0.0, 0.0, 0.0)

    def send_statustext(
        self,
        text: str,
        severity: int | None = None,
    ) -> bool:
        """Send STATUSTEXT message to GCS."""

        def _send() -> None:
            sev = severity if severity is not None else mavutil.mavlink.MAV_SEVERITY_INFO
            self._conn.mav.statustext_send(sev, text[:50].encode("utf-8"))

        return self._send_guarded("Statustext", _send)

    def set_mode(self, mode_id: int) -> bool:
        """Set the flight mode on the autopilot."""
        return self._send_guarded(
            "Set mode",
            lambda: self._send_command_long_and_wait_ack(
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                1,
                mode_id,
            ),
        )

    def takeoff(self, altitude_m: float) -> bool:
        """Command an autonomous takeoff to ``altitude_m`` AGL."""
        return self._send_guarded(
            "Takeoff",
            lambda: self._send_command_long_and_wait_ack(
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                float(altitude_m),
                timeout_s=2.0,
            ),
        )

    def land(self) -> bool:
        """Switch the autopilot to LAND mode for an autonomous descent."""
        return self.set_mode(9)

    def request_home_position(self) -> bool:
        """Ask ArduPilot to send HOME_POSITION once."""
        return self._send_guarded(
            "Home position request",
            lambda: self._send_command_long_and_wait_ack(
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
                timeout_s=1.0,
            ),
        )

    def send_global_position_target(
        self,
        lat: float,
        lon: float,
        alt_msl: float,
        yaw: float | None = None,
    ) -> bool:
        """Send a GUIDED global position target in WGS84 / MSL."""
        if self._conn is None:
            return False
        if not self._fence_allows_global(lat, lon):
            return False

        def _send() -> None:
            type_mask = 0b0000_1111_1111_1000
            if yaw is not None:
                type_mask &= ~(1 << 10)
            self._conn.mav.set_position_target_global_int_send(
                0,
                self._conn.target_system,
                self._conn.target_component,
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

        return self._send_guarded("Global position target", _send)

    def send_position_target(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
    ) -> bool:
        """Send position target via SET_POSITION_TARGET_LOCAL_NED."""
        if self._conn is None:
            return False
        if not self._fence_allows_local(x, y):
            return False

        def _send() -> None:
            self._conn.mav.set_position_target_local_ned_send(
                0,
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000_1111_1111_1000,
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

        return self._send_guarded("Position target", _send)
