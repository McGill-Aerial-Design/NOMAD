# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
from __future__ import annotations

import logging
import os
import threading
import time
from typing import Any

from pymavlink import mavutil

from ..state import StateManager

logger = logging.getLogger("edge_core.mavlink.connection")


class MavlinkConnection:
    """Handles connection setup, PyMAVLink receiver loop, and state dispatching."""

    def __init__(self, state_manager: StateManager, endpoint: str | None = None) -> None:
        self.state_manager = state_manager
        default_endpoint = os.environ.get("NOMAD_MAVLINK_ENDPOINT", "127.0.0.1:14550")
        endpoint = endpoint or default_endpoint
        # Pass through explicit pymavlink schemes (udpin:/udpout:/tcp:/serial:/…);
        # otherwise treat a bare host:port as an outbound UDP endpoint.
        _schemes = ("udp:", "udpin:", "udpout:", "tcp:", "tcpin:", "serial:", "/dev/", "com")
        self.endpoint = endpoint if endpoint.lower().startswith(_schemes) else f"udp:{endpoint}"
        self._conn: Any = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._last_heartbeat = 0.0
        self.disconnect_timeout = float(os.environ.get("NOMAD_MAVLINK_DISCONNECT_TIMEOUT_S", "3.0"))

        # Time sync service reference (set externally)
        self._time_sync_service: Any = None

        self._health_thread: threading.Thread | None = None
        self._health_stop_event: threading.Event | None = None
        self._health_interval: float = 2.0

        # SERVO_OUTPUT_RAW: last seen PWM per channel (1-indexed, us).
        self._servo_output: dict[int, int] = {}
        self._servo_output_lock = threading.Lock()
        self._ack_condition = threading.Condition()
        self._command_acks: list[dict[str, Any]] = []

    def set_time_sync_service(self, service: Any) -> None:
        """Set the TimeSyncService to receive GPS time updates."""
        self._time_sync_service = service

    def get_servo_output_pwm(self, channel: int) -> int | None:
        """Return the last SERVO_OUTPUT_RAW PWM (us) for channel (1-indexed), or None."""
        with self._servo_output_lock:
            return self._servo_output.get(channel)

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self.stop_health_broadcast()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._conn:
            try:
                self._conn.close()
            except Exception:
                pass

    def _connect(self) -> None:
        try:
            self._conn = mavutil.mavlink_connection(self.endpoint, autoreconnect=True)
        except Exception:
            self._conn = None

    def _run(self) -> None:
        self._connect()
        while not self._stop_event.is_set():
            if self._conn is None:
                time.sleep(0.5)
                self._connect()
                continue

            try:
                msg_types = [
                    "HEARTBEAT",
                    "SYS_STATUS",
                    "GLOBAL_POSITION_INT",
                    "ATTITUDE",
                    "SYSTEM_TIME",
                    "COMMAND_ACK",
                    "HOME_POSITION",
                    "SERVO_OUTPUT_RAW",
                ]
                msg = self._conn.recv_match(
                    type=msg_types,
                    blocking=True,
                    timeout=0.2,
                )
            except Exception:
                msg = None

            now = time.time()
            if msg is None:
                self._update_connection_status(now)
                continue

            msg_type = msg.get_type()
            if msg_type == "HEARTBEAT":
                self._last_heartbeat = now
                mode = self._resolve_mode(msg)
                base_mode = getattr(msg, "base_mode", 0)
                is_armed = bool(base_mode & 128)
                self.state_manager.force_state_update(flight_mode=mode, connected=True, armed=is_armed)
            elif msg_type == "SYS_STATUS":
                voltage = getattr(msg, "voltage_battery", 0) or 0
                if voltage:
                    self.state_manager.update_state(battery_voltage=voltage / 1000.0)
            elif msg_type == "GLOBAL_POSITION_INT":
                lat_raw = getattr(msg, "lat", 0)
                lon_raw = getattr(msg, "lon", 0)
                alt_raw = getattr(msg, "alt", 0)
                rel_alt_raw = getattr(msg, "relative_alt", 0)
                gps_fix = bool(lat_raw or lon_raw)
                gps_lat = lat_raw / 1e7 if gps_fix else None
                gps_lon = lon_raw / 1e7 if gps_fix else None
                gps_alt = alt_raw / 1000.0 if gps_fix else None
                alt_agl_m = rel_alt_raw / 1000.0 if gps_fix else None
                self.state_manager.update_state(
                    gps_fix=gps_fix,
                    gps_lat=gps_lat,
                    gps_lon=gps_lon,
                    gps_alt=gps_alt,
                    alt_agl_m=alt_agl_m,
                )
            elif msg_type == "HOME_POSITION":
                lat_raw = getattr(msg, "latitude", 0)
                lon_raw = getattr(msg, "longitude", 0)
                alt_raw = getattr(msg, "altitude", 0)
                if lat_raw or lon_raw:
                    self.state_manager.update_state(
                        home_lat=lat_raw / 1e7,
                        home_lon=lon_raw / 1e7,
                        home_alt=alt_raw / 1000.0,
                    )
            elif msg_type == "ATTITUDE":
                import math

                roll_rad = getattr(msg, "roll", 0.0)
                pitch_rad = getattr(msg, "pitch", 0.0)
                yaw_rad = getattr(msg, "yaw", 0.0)
                heading_deg = math.degrees(yaw_rad) % 360.0
                pitch_deg = math.degrees(pitch_rad)
                roll_deg = math.degrees(roll_rad)
                self.state_manager.update_state(
                    heading_deg=heading_deg,
                    pitch_deg=pitch_deg,
                    roll_deg=roll_deg,
                )
            elif msg_type == "SYSTEM_TIME":
                if self._time_sync_service is not None:
                    time_unix_usec = getattr(msg, "time_unix_usec", 0)
                    time_boot_ms = getattr(msg, "time_boot_ms", 0)
                    if time_unix_usec > 0:
                        self._time_sync_service.update_gps_time(time_unix_usec, time_boot_ms)
            elif msg_type == "COMMAND_ACK":
                self._record_command_ack(msg)
            elif msg_type == "SERVO_OUTPUT_RAW":
                with self._servo_output_lock:
                    for ch in range(1, 17):
                        attr = f"servo{ch}_raw"
                        val = getattr(msg, attr, 0)
                        if val > 0:
                            self._servo_output[ch] = val

    def _record_command_ack(self, msg: Any) -> None:
        ack = {
            "command": int(getattr(msg, "command", -1)),
            "result": int(getattr(msg, "result", -1)),
            "timestamp": time.monotonic(),
        }
        with self._ack_condition:
            self._command_acks.append(ack)
            self._command_acks = self._command_acks[-25:]
            self._ack_condition.notify_all()

    def _wait_command_ack(
        self,
        command_id: int,
        *,
        since: float,
        timeout_s: float = 0.75,
    ) -> bool:
        accepted = {
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
        }
        deadline = time.monotonic() + timeout_s
        with self._ack_condition:
            while True:
                for ack in reversed(self._command_acks):
                    if ack["timestamp"] < since:
                        break
                    if ack["command"] == int(command_id):
                        return ack["result"] in accepted
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._ack_condition.wait(timeout=remaining)

    def _send_command_long_and_wait_ack(
        self,
        command_id: int,
        *params: float,
        timeout_s: float = 0.75,
    ) -> bool:
        if self._conn is None:
            return False
        padded = list(params[:7]) + [0.0] * max(0, 7 - len(params))
        start = time.monotonic()
        self._conn.mav.command_long_send(
            self._conn.target_system,
            self._conn.target_component,
            command_id,
            0,
            *padded[:7],
        )
        return self._wait_command_ack(command_id, since=start, timeout_s=timeout_s)

    def _update_connection_status(self, now: float) -> None:
        if self._last_heartbeat and (now - self._last_heartbeat) > self.disconnect_timeout:
            if self.state_manager.get_state().connected:
                self.state_manager.force_state_update(connected=False, flight_mode="LOST")

    @staticmethod
    def _resolve_mode(msg: Any) -> str:
        try:
            mode = mavutil.mode_string_v10(msg)
            return mode or "UNKNOWN"
        except Exception:
            return "UNKNOWN"

    def start_health_broadcast(self, interval: float = 2.0) -> None:
        """Start background task to broadcast hardware health via STATUSTEXT."""
        if self._health_thread is not None:
            return

        self._health_interval = interval
        self._health_stop_event = threading.Event()
        self._health_thread = threading.Thread(
            target=self._broadcast_health_loop,
            daemon=True,
            name="nomad-health-broadcast",
        )
        self._health_thread.start()

    def stop_health_broadcast(self) -> None:
        """Stop the health broadcast background task."""
        if self._health_stop_event is not None:
            self._health_stop_event.set()
        if self._health_thread is not None:
            self._health_thread.join(timeout=2.0)
            self._health_thread = None

    def _broadcast_health_loop(self) -> None:
        """Background loop to broadcast health status."""
        assert self._health_stop_event is not None
        stop_event = self._health_stop_event
        while not stop_event.is_set():
            try:
                state = self.state_manager.get_state()
                cpu_temp = state.cpu_temp_c if state.cpu_temp_c else 0
                gpu_load = state.gpu_load_pct if state.gpu_load_pct else 0
                disk_free = state.disk_free_gb if state.disk_free_gb else 0

                status_msg = f"NOMAD: CPU {cpu_temp:.0f}C | GPU {gpu_load:.0f}% | NVMe {disk_free:.0f}GB"
                if hasattr(self, "send_statustext"):
                    self.send_statustext(status_msg)  # type: ignore
            except Exception as e:
                logger.debug(f"Health broadcast error: {e}")

            stop_event.wait(self._health_interval)
