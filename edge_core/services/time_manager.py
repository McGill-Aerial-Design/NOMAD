# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
Hybrid Time Synchronization Service for NOMAD.

Ensures accurate time synchronization using a hybrid approach:
1. Primary: NTP synchronization (when internet available)
2. Fallback: GPS time from MAVLink SYSTEM_TIME

Critical for:
- Mission logging timestamp accuracy
- VIO/sensor fusion timing
- Multi-system coordination

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import logging
import platform
import socket
import subprocess
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum, auto
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .state import StateManager

logger = logging.getLogger(__name__)


class TimeSyncSource(Enum):
    """Time synchronization source."""

    NONE = auto()  # Not synchronized
    NTP = auto()  # Network Time Protocol
    GPS = auto()  # GPS time from flight controller
    MANUAL = auto()  # Manually set


@dataclass
class TimeSyncStatus:
    """Current time synchronization status."""

    synced: bool
    source: TimeSyncSource
    offset_seconds: float  # Estimated offset from true time
    last_sync: datetime | None
    gps_time_available: bool
    ntp_reachable: bool

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {
            "synced": self.synced,
            "source": self.source.name,
            "offset_seconds": self.offset_seconds,
            "last_sync": self.last_sync.isoformat() if self.last_sync else None,
            "gps_time_available": self.gps_time_available,
            "ntp_reachable": self.ntp_reachable,
        }


class TimeSyncService:
    """
    Hybrid Time Synchronization Service.

    Monitors time synchronization status and provides fallback
    to GPS time when NTP is unavailable.

    Usage:
        service = TimeSyncService(state_manager)
        service.start()

        # Check status
        if service.is_synced:
            print(f"Time synced via {service.status.source.name}")

        # Force sync from GPS (requires sudo/CAP_SYS_TIME)
        if service.force_sync_from_gps():
            print("Synced to GPS time")
    """

    # Warning threshold for time offset (seconds)
    OFFSET_WARNING_THRESHOLD = 1.0

    # NTP check interval (seconds)
    NTP_CHECK_INTERVAL = 60.0

    # GPS time update interval (seconds)
    GPS_CHECK_INTERVAL = 10.0

    # NTP server to check connectivity
    NTP_SERVER = "pool.ntp.org"
    NTP_PORT = 123

    def __init__(
        self,
        state_manager: StateManager | None = None,
        on_sync_change: Callable[[TimeSyncStatus], None] | None = None,
    ) -> None:
        """
        Initialize time synchronization service.

        Args:
            state_manager: StateManager for updating time_synced flag
            on_sync_change: Callback when sync status changes
        """
        self._state_manager = state_manager
        self._on_sync_change = on_sync_change

        # Status
        self._status = TimeSyncStatus(
            synced=False,
            source=TimeSyncSource.NONE,
            offset_seconds=0.0,
            last_sync=None,
            gps_time_available=False,
            ntp_reachable=False,
        )
        self._lock = threading.RLock()

        # MAVLink time data
        self._gps_time_us: int = 0
        self._gps_boot_ms: int = 0
        self._last_gps_time_update: float = 0.0

        # Background monitoring
        self._running = False
        self._monitor_thread: threading.Thread | None = None

        # Platform detection
        self._is_linux = platform.system() == "Linux"
        self._is_jetson = self._detect_jetson()

    @staticmethod
    def _detect_jetson() -> bool:
        """Detect if running on NVIDIA Jetson."""
        try:
            with open("/etc/nv_tegra_release") as f:
                return "NVIDIA" in f.read()
        except FileNotFoundError:
            return False

    @property
    def status(self) -> TimeSyncStatus:
        """Get current synchronization status."""
        with self._lock:
            return self._status

    @property
    def is_synced(self) -> bool:
        """Check if time is synchronized."""
        with self._lock:
            return self._status.synced

    @property
    def sync_source(self) -> TimeSyncSource:
        """Get current sync source."""
        with self._lock:
            return self._status.source

    def start(self) -> None:
        """Start the time synchronization service."""
        if self._running:
            return

        self._running = True

        # Do initial sync check
        self._check_sync_status()

        # Start background monitoring
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True,
            name="time_sync_monitor",
        )
        self._monitor_thread.start()

        logger.info(f"TimeSyncService started (source={self._status.source.name})")

    def stop(self) -> None:
        """Stop the time synchronization service."""
        self._running = False

        if self._monitor_thread:
            self._monitor_thread.join(timeout=2.0)
            self._monitor_thread = None

        logger.info("TimeSyncService stopped")

    def check_ntp_status(self) -> bool:
        """
        Check if NTP synchronization is active.

        Returns:
            True if NTP is synchronized
        """
        # Method 1: Check timedatectl on Linux
        if self._is_linux:
            try:
                result = subprocess.run(
                    ["timedatectl", "status"],
                    capture_output=True,
                    text=True,
                    timeout=5.0,
                )
                if result.returncode == 0:
                    output = result.stdout.lower()
                    # Check for NTP synchronized status
                    if "ntp synchronized: yes" in output or "system clock synchronized: yes" in output:
                        logger.debug("NTP synchronized (timedatectl)")
                        return True
            except (subprocess.TimeoutExpired, FileNotFoundError, Exception) as e:
                logger.debug(f"timedatectl check failed: {e}")

        # Method 2: Try connecting to NTP server
        return self._check_ntp_reachable()

    def _check_ntp_reachable(self) -> bool:
        """Check if NTP server is reachable."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(3.0)

            # Send minimal NTP request
            # NTP packet: 48 bytes, first byte = 0x1b (LI=0, VN=3, Mode=3)
            ntp_packet = b"\x1b" + 47 * b"\0"

            sock.sendto(ntp_packet, (self.NTP_SERVER, self.NTP_PORT))
            data, _ = sock.recvfrom(48)
            sock.close()

            if len(data) >= 48:
                logger.debug(f"NTP server {self.NTP_SERVER} reachable")
                return True

        except (TimeoutError, OSError, Exception) as e:
            logger.debug(f"NTP server unreachable: {e}")

        return False

    def update_gps_time(self, unix_time_us: int, boot_time_ms: int) -> None:
        """
        Update GPS time from MAVLink SYSTEM_TIME message.

        Args:
            unix_time_us: Unix timestamp in microseconds
            boot_time_ms: Time since boot in milliseconds
        """
        with self._lock:
            self._gps_time_us = unix_time_us
            self._gps_boot_ms = boot_time_ms
            self._last_gps_time_update = time.time()

            # Mark GPS time as available if valid
            if unix_time_us > 0:
                self._status = TimeSyncStatus(
                    synced=self._status.synced,
                    source=self._status.source,
                    offset_seconds=self._status.offset_seconds,
                    last_sync=self._status.last_sync,
                    gps_time_available=True,
                    ntp_reachable=self._status.ntp_reachable,
                )

                # Check time offset
                self._check_time_offset()

    def _check_time_offset(self) -> None:
        """Check offset between system time and GPS time."""
        if self._gps_time_us == 0:
            return

        # Convert GPS time to seconds
        gps_time_s = self._gps_time_us / 1_000_000.0
        system_time_s = time.time()

        offset = abs(system_time_s - gps_time_s)

        with self._lock:
            self._status = TimeSyncStatus(
                synced=self._status.synced,
                source=self._status.source,
                offset_seconds=offset,
                last_sync=self._status.last_sync,
                gps_time_available=self._status.gps_time_available,
                ntp_reachable=self._status.ntp_reachable,
            )

        # Log warning if offset too large
        if offset > self.OFFSET_WARNING_THRESHOLD:
            logger.warning(f"Time offset detected: {offset:.3f}s (System: {system_time_s:.3f}, GPS: {gps_time_s:.3f})")

    def force_sync_from_gps(self) -> bool:
        """
        Force system time to GPS time.

        Requires sudo privileges or CAP_SYS_TIME capability.

        Returns:
            True if sync successful
        """
        if self._gps_time_us == 0:
            logger.error("Cannot sync: No GPS time available")
            return False

        # Convert to datetime
        gps_time_s = self._gps_time_us / 1_000_000.0
        gps_datetime = datetime.fromtimestamp(gps_time_s, tz=timezone.utc)

        logger.info(f"Attempting to sync system time to GPS: {gps_datetime.isoformat()}")

        if not self._is_linux:
            logger.warning("Time sync only supported on Linux")
            return False

        try:
            # Format: "YYYY-MM-DD HH:MM:SS"
            time_str = gps_datetime.strftime("%Y-%m-%d %H:%M:%S")

            # Try using timedatectl first (preferred on systemd systems)
            result = subprocess.run(
                ["timedatectl", "set-time", time_str],
                capture_output=True,
                text=True,
                timeout=10.0,
            )

            if result.returncode == 0:
                logger.info(f"System time set to {time_str} via timedatectl")
                self._update_sync_status(TimeSyncSource.GPS)
                return True

            # Fallback to date command
            logger.debug(f"timedatectl failed: {result.stderr}, trying date command")

            # Format for date: "MMDDHHmmYYYY.SS"
            date_str = gps_datetime.strftime("%m%d%H%M%Y.%S")

            result = subprocess.run(
                ["date", date_str],
                capture_output=True,
                text=True,
                timeout=10.0,
            )

            if result.returncode == 0:
                logger.info("System time set to GPS time via date command")
                self._update_sync_status(TimeSyncSource.GPS)
                return True

            logger.error(f"Failed to set time: {result.stderr}")

        except subprocess.TimeoutExpired:
            logger.error("Time sync command timed out")
        except PermissionError:
            logger.error("Permission denied. Requires sudo or CAP_SYS_TIME capability.")
        except Exception as e:
            logger.error(f"Time sync error: {e}")

        return False

    def _update_sync_status(self, source: TimeSyncSource) -> None:
        """Update sync status and notify listeners."""
        with self._lock:
            self._status = TimeSyncStatus(
                synced=True,
                source=source,
                offset_seconds=0.0,
                last_sync=datetime.now(timezone.utc),
                gps_time_available=self._status.gps_time_available,
                ntp_reachable=self._status.ntp_reachable,
            )

            # Update state manager
            if self._state_manager:
                self._state_manager.update_state(time_synced=True)

            # Notify callback
            if self._on_sync_change:
                self._on_sync_change(self._status)

    def _check_sync_status(self) -> None:
        """Check and update synchronization status."""
        ntp_synced = self.check_ntp_status()
        ntp_reachable = self._check_ntp_reachable() if not ntp_synced else True

        with self._lock:
            old_synced = self._status.synced
            old_source = self._status.source

            if ntp_synced:
                self._status = TimeSyncStatus(
                    synced=True,
                    source=TimeSyncSource.NTP,
                    offset_seconds=0.0,
                    last_sync=datetime.now(timezone.utc),
                    gps_time_available=self._status.gps_time_available,
                    ntp_reachable=True,
                )
            elif self._status.gps_time_available and self._status.offset_seconds < self.OFFSET_WARNING_THRESHOLD:
                # GPS time is close enough to system time
                self._status = TimeSyncStatus(
                    synced=True,
                    source=TimeSyncSource.GPS,
                    offset_seconds=self._status.offset_seconds,
                    last_sync=datetime.now(timezone.utc),
                    gps_time_available=True,
                    ntp_reachable=ntp_reachable,
                )
            else:
                self._status = TimeSyncStatus(
                    synced=False,
                    source=TimeSyncSource.NONE,
                    offset_seconds=self._status.offset_seconds,
                    last_sync=self._status.last_sync,
                    gps_time_available=self._status.gps_time_available,
                    ntp_reachable=ntp_reachable,
                )

            # Update state manager
            if self._state_manager:
                self._state_manager.update_state(time_synced=self._status.synced)

            # Log status change
            if self._status.synced != old_synced or self._status.source != old_source:
                if self._status.synced:
                    logger.info(f"Time synchronized via {self._status.source.name}")
                else:
                    logger.warning("Time synchronization lost")

                # Notify callback
                if self._on_sync_change:
                    self._on_sync_change(self._status)

    def _monitor_loop(self) -> None:
        """Background monitoring loop."""
        last_ntp_check = 0.0
        last_gps_check = 0.0

        while self._running:
            try:
                current_time = time.time()

                # Check NTP periodically
                if current_time - last_ntp_check >= self.NTP_CHECK_INTERVAL:
                    self._check_sync_status()
                    last_ntp_check = current_time

                # Check GPS time offset periodically
                if current_time - last_gps_check >= self.GPS_CHECK_INTERVAL:
                    if self._gps_time_us > 0:
                        self._check_time_offset()
                    last_gps_check = current_time

                time.sleep(1.0)

            except Exception as e:
                logger.error(f"Time sync monitor error: {e}")
                time.sleep(5.0)

    def get_corrected_timestamp(self) -> datetime:
        """
        Get timestamp with offset correction applied.

        Returns:
            Corrected UTC timestamp
        """
        now = datetime.now(timezone.utc)

        with self._lock:
            # If we have GPS time and significant offset, correct it
            if self._status.gps_time_available and self._status.offset_seconds > 0.1:
                # This is a rough correction - for precise timing, use force_sync_from_gps
                # Determine sign of correction from stored GPS time
                if self._gps_time_us > 0:
                    gps_time = datetime.fromtimestamp(self._gps_time_us / 1_000_000.0, tz=timezone.utc)
                    # Interpolate: GPS time + elapsed since GPS update
                    elapsed = time.time() - self._last_gps_time_update
                    from datetime import timedelta

                    return gps_time + timedelta(seconds=elapsed)

        return now


# Global instance (optional singleton pattern)
_time_sync_service: TimeSyncService | None = None


def get_time_sync_service() -> TimeSyncService:
    """Get or create the global TimeSyncService instance."""
    global _time_sync_service
    if _time_sync_service is None:
        _time_sync_service = TimeSyncService()
    return _time_sync_service


def init_time_sync_service(state_manager: StateManager) -> TimeSyncService:
    """Initialize the global TimeSyncService with a state manager."""
    global _time_sync_service
    _time_sync_service = TimeSyncService(state_manager=state_manager)
    return _time_sync_service
