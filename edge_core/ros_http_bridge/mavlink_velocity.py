# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
Direct ROS -> MAVLink velocity controller for the NOMAD ROS-HTTP bridge.

nav2/nvblox publish ``/cmd_vel`` (Twist, ROS REP-103 FLU body frame). This
controller opens its own PyMAVLink link to mavlink-router and streams
SET_POSITION_TARGET_LOCAL_NED setpoints to ArduPilot in GUIDED mode - no HTTP
hop through Edge Core. It owns the flight-safety gating for these setpoints:

- velocity clamping (MAX_VELOCITY_XY / _Z, MAX_YAW_RATE) and finite checks,
- VIO freshness/confidence gate (refuse setpoints when tracking is stale/poor),
- armed + GUIDED-mode gate (parsed from the FC HEARTBEAT on its own link),
- a command-timeout watchdog that zeroes velocity if cmd_vel stops arriving,
- the ROS FLU -> MAVLink FRD frame conversion (negate vy, vz, yaw_rate).

The flight-safety *decisions* above live in the dependency-light
:mod:`edge_core.safety` core (Tier SC); this class is the I/O **adapter** that
snapshots state under its lock, asks ``safety`` whether a command is permitted,
and transmits what it returns. See ``docs/safety/`` for the requirements these
implement.

The link is independent of the Edge Core orchestrator link; mavlink-router
multiplexes both. See infra/transport/mavlink_router/main.conf
([UdpEndpoint nav_bridge], port 14552).
"""

from __future__ import annotations

import logging
import os
import threading
import time

from edge_core.safety import (
    VELOCITY_TYPE_MASK,
    Decision,
    EnvelopePolicy,
    FlightConditions,
    VelocityCommand,
    VelocityLimits,
    evaluate,
    heartbeat_from_vehicle,
    vio_fresh,
    watchdog_decision,
)

try:
    from pymavlink import mavutil

    MAVLINK_AVAILABLE = True
except Exception:  # pragma: no cover - exercised only when pymavlink is absent
    mavutil = None  # type: ignore[assignment]
    MAVLINK_AVAILABLE = False

logger = logging.getLogger("ros_http_bridge.mavlink_velocity")


class MavlinkVelocityController:
    """Streams nav2 ``/cmd_vel`` to ArduPilot GUIDED mode over a direct link."""

    # Stop the vehicle if no fresh command arrives within this window.
    COMMAND_TIMEOUT_S = 0.5

    # Safety velocity limits (m/s, rad/s).
    MAX_VELOCITY_XY = 2.0
    MAX_VELOCITY_Z = 1.0
    MAX_YAW_RATE = 1.0

    # Minimum VIO confidence to accept commands (matches the old NavController).
    MIN_VIO_CONFIDENCE = 0.3

    # ArduPilot flight mode required for guided velocity control.
    GUIDED_MODE = "GUIDED"

    # type_mask for SET_POSITION_TARGET_LOCAL_NED: velocity + yaw_rate only.
    # Single source of truth in the SC core (edge_core.safety).
    _TYPE_MASK = VELOCITY_TYPE_MASK

    def __init__(
        self,
        endpoint: str | None = None,
        *,
        vio_max_age_s: float | None = None,
        require_armed: bool = True,
        require_guided: bool = True,
        heartbeat_timeout_s: float | None = None,
        logger_adapter=None,
    ) -> None:
        default_endpoint = os.environ.get("NOMAD_BRIDGE_MAVLINK_ENDPOINT", "127.0.0.1:14552")
        endpoint = endpoint or default_endpoint
        # Mirror MavlinkConnection: a bare host:port means a bound UDP socket that
        # mavlink-router (Mode=Normal) pushes the FC stream to.
        if "://" in endpoint or endpoint.split(":", 1)[0] in ("udp", "udpin", "udpout", "tcp"):
            self.endpoint = endpoint
        else:
            self.endpoint = f"udp:{endpoint}"

        self._require_armed = require_armed
        self._require_guided = require_guided
        self._vio_max_age_s = (
            vio_max_age_s if vio_max_age_s is not None else _read_positive_float("NOMAD_VIO_MAX_AGE_S", 1.0)
        )
        self._heartbeat_timeout_s = (
            heartbeat_timeout_s
            if heartbeat_timeout_s is not None
            else _read_positive_float("NOMAD_MAVLINK_DISCONNECT_TIMEOUT_S", 3.0)
        )
        self._log = logger_adapter or logger

        # The SC-core policy: limits + gate thresholds, built from this adapter's
        # constants/config. The class constants remain the source of the limits.
        self._policy = EnvelopePolicy(
            limits=VelocityLimits(self.MAX_VELOCITY_XY, self.MAX_VELOCITY_Z, self.MAX_YAW_RATE),
            require_armed=require_armed,
            require_guided=require_guided,
            guided_mode=self.GUIDED_MODE,
            heartbeat_timeout_s=self._heartbeat_timeout_s,
            vio_max_age_s=self._vio_max_age_s,
            min_vio_confidence=self.MIN_VIO_CONFIDENCE,
        )

        self._conn = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._rx_thread: threading.Thread | None = None
        self._watchdog_thread: threading.Thread | None = None

        # FC state, updated from HEARTBEAT on the rx thread.
        self._armed = False
        self._flight_mode = "UNKNOWN"
        self._last_heartbeat = 0.0

        # VIO health, updated by the bridge from the odom callback.
        self._vio_confidence = 0.0
        self._vio_healthy = False
        self._vio_last_update = 0.0

        # Command tracking for the watchdog.
        self._last_command_time = 0.0
        self._active = False

        # Throttled-warning bookkeeping {key: last_log_monotonic}.
        self._warn_times: dict[str, float] = {}
        self._warn_interval_s = 2.0

        # Stats (read by the bridge for its status payload).
        self.sent_count = 0
        self.rejected_count = 0

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> bool:
        """Open the MAVLink link and start the rx + watchdog threads."""
        if not MAVLINK_AVAILABLE:
            self._log.error(
                "pymavlink is not available - direct cmd_vel -> MAVLink control is DISABLED. "
                "Install pymavlink in the ROS container to enable autonomous velocity control."
            )
            return False
        if self._rx_thread and self._rx_thread.is_alive():
            return True

        if not self._connect():
            # Keep the threads running anyway; _rx_loop retries the connection.
            self._log.warning("Initial MAVLink connect to %s failed; will retry", self.endpoint)

        self._stop_event.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, name="nomad-navvel-rx", daemon=True)
        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, name="nomad-navvel-watchdog", daemon=True)
        self._rx_thread.start()
        self._watchdog_thread.start()
        self._log.info("Direct MAVLink velocity controller started (endpoint=%s)", self.endpoint)
        return True

    def stop(self) -> None:
        """Zero the velocity, stop the threads, and close the link."""
        self._stop_event.set()
        try:
            self._send_stop()
        except Exception as e:
            self._log.debug("Final zero-velocity send failed during stop: %s", e)
        for thread in (self._rx_thread, self._watchdog_thread):
            if thread and thread.is_alive():
                thread.join(timeout=2.0)
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception as e:
                self._log.debug("MAVLink close error during stop: %s", e)
            self._conn = None

    def _connect(self) -> bool:
        if not MAVLINK_AVAILABLE:
            return False
        try:
            with self._lock:
                self._conn = mavutil.mavlink_connection(self.endpoint, autoreconnect=True)
            return True
        except Exception as e:
            self._log.debug("MAVLink connect error (%s): %s", self.endpoint, e)
            with self._lock:
                self._conn = None
            return False

    # -- inputs from the bridge --------------------------------------------

    def note_vio(self, confidence: float, healthy: bool = True) -> None:
        """Record a VIO/odom update so the freshness gate stays open."""
        with self._lock:
            self._vio_confidence = confidence
            self._vio_healthy = healthy
            self._vio_last_update = time.monotonic()

    def submit(self, vx: float, vy: float, vz: float, yaw_rate: float) -> bool:
        """Gate, clamp, frame-convert and send one nav2 velocity command.

        Inputs are ROS REP-103 body frame (x forward, y left, z up, yaw CCW+).
        Returns True if a setpoint was sent to the flight controller.

        This is a thin adapter: it snapshots FC/VIO state under the lock, then
        defers every flight-safety decision to :func:`edge_core.safety.evaluate`.
        """
        now = time.monotonic()
        with self._lock:
            conditions = self._snapshot_conditions_locked()

        decision: Decision = evaluate(self._policy, conditions, VelocityCommand(vx, vy, vz, yaw_rate), now)
        if not decision.allowed:
            self._warn(decision.reason or "rejected", decision.message or "cmd_vel rejected")
            with self._lock:
                self.rejected_count += 1
            return False

        if decision.setpoint is None:  # allowed always carries a setpoint
            self._warn("rejected", "envelope allowed a command without a setpoint - dropping")
            return False
        sent = self._send_velocity_frd(*decision.setpoint)
        if sent:
            with self._lock:
                self._last_command_time = now
                self._active = True
                self.sent_count += 1
        else:
            with self._lock:
                self.rejected_count += 1
        return sent

    def _snapshot_conditions_locked(self) -> FlightConditions:
        """Build an immutable view of FC/VIO state for the SC envelope.

        Must be called with ``self._lock`` held.
        """
        return FlightConditions(
            connected=self._conn is not None,
            armed=self._armed,
            flight_mode=self._flight_mode,
            last_heartbeat=self._last_heartbeat,
            vio_confidence=self._vio_confidence,
            vio_healthy=self._vio_healthy,
            vio_last_update=self._vio_last_update,
        )

    # -- MAVLink TX ---------------------------------------------------------

    def _send_velocity_frd(self, vx: float, vy: float, vz: float, yaw_rate: float) -> bool:
        with self._lock:
            conn = self._conn
        if conn is None:
            return False
        try:
            conn.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms (0 = use system time)
                conn.target_system,
                conn.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                self._TYPE_MASK,
                0.0,
                0.0,
                0.0,  # x, y, z (ignored)
                vx,
                vy,
                vz,
                0.0,
                0.0,
                0.0,  # afx, afy, afz (ignored)
                0.0,  # yaw (ignored)
                yaw_rate,
            )
            return True
        except Exception as e:
            self._log.debug("Velocity send error: %s", e)
            return False

    def _send_stop(self) -> bool:
        return self._send_velocity_frd(0.0, 0.0, 0.0, 0.0)

    # -- background threads -------------------------------------------------

    def _rx_loop(self) -> None:
        """Parse HEARTBEAT to track armed state + flight mode; auto-reconnect."""
        while not self._stop_event.is_set():
            with self._lock:
                conn = self._conn
            if conn is None:
                time.sleep(0.5)
                self._connect()
                continue
            try:
                msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            except Exception:
                msg = None
            if msg is None:
                continue
            # Only track the autopilot we command; ignore GCS and other systems'
            # heartbeats sharing the link, which would otherwise flip our view of
            # armed/mode every other beat and drop valid setpoints (SR-VEL-06).
            if not heartbeat_from_vehicle(
                msg.get_srcSystem(), msg.type, conn.target_system, gcs_type=mavutil.mavlink.MAV_TYPE_GCS
            ):
                continue
            try:
                mode = mavutil.mode_string_v10(msg) or "UNKNOWN"
            except Exception:
                mode = "UNKNOWN"
            armed = bool(getattr(msg, "base_mode", 0) & 128)
            with self._lock:
                self._flight_mode = mode
                self._armed = armed
                self._last_heartbeat = time.monotonic()

    def _watchdog_loop(self) -> None:
        """Zero velocity if cmd_vel stops or VIO goes stale while moving."""
        interval = 1.0 / 20.0
        while not self._stop_event.wait(interval):
            now = time.monotonic()
            with self._lock:
                active = self._active
                last_command_time = self._last_command_time
                vio_is_fresh = vio_fresh(self._vio_last_update, now, self._vio_max_age_s)
            result = watchdog_decision(
                active=active,
                last_command_time=last_command_time,
                now=now,
                command_timeout_s=self.COMMAND_TIMEOUT_S,
                vio_is_fresh=vio_is_fresh,
            )
            if result.stop:
                self._log.warning("Stopping velocity (%s)", result.reason)
                self._send_stop()
                with self._lock:
                    self._active = False

    def _warn(self, key: str, message: str) -> None:
        now = time.monotonic()
        last = self._warn_times.get(key, 0.0)
        if now - last >= self._warn_interval_s:
            self._warn_times[key] = now
            self._log.warning(message)


def _read_positive_float(env_name: str, default: float) -> float:
    raw = os.environ.get(env_name, str(default)).strip()
    try:
        value = float(raw)
        if value <= 0.0:
            raise ValueError("value must be positive")
        return value
    except Exception:
        logger.warning("Invalid %s='%s'; falling back to %.2f", env_name, raw, default)
        return default
