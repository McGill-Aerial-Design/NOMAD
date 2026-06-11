# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tailscale VPN status for NOMAD.

Polls ``tailscale status --json`` on a background thread and exposes the
fields the ``/network/status`` route and the GCS dashboard consume:
status / ip / hostname / peer_count. Auto-reconnects via ``tailscale up``
when the link drops.
"""

from __future__ import annotations

import json
import logging
import subprocess
import threading
from dataclasses import dataclass
from datetime import datetime
from enum import Enum

logger = logging.getLogger(__name__)


class TailscaleStatus(Enum):
    """Tailscale connection states."""

    CONNECTED = "connected"
    CONNECTING = "connecting"
    DISCONNECTED = "disconnected"
    NEEDS_AUTH = "needs_auth"
    NOT_INSTALLED = "not_installed"
    ERROR = "error"


@dataclass
class TailscaleInfo:
    """Current Tailscale connection info (the consumed surface only)."""

    status: TailscaleStatus
    ip_address: str | None = None
    hostname: str = "unknown"
    peer_count: int = 0
    latency_ms: float | None = None  # reserved; not measured today
    last_check: datetime | None = None


_STATUS_MAP = {
    "Running": TailscaleStatus.CONNECTED,
    "Starting": TailscaleStatus.CONNECTING,
    "NeedsLogin": TailscaleStatus.NEEDS_AUTH,
    "NeedsMachineAuth": TailscaleStatus.NEEDS_AUTH,
    "Stopped": TailscaleStatus.DISCONNECTED,
}


def parse_status_json(data: dict) -> TailscaleInfo:
    """Parse ``tailscale status --json`` output into the consumed fields."""
    self_info = data.get("Self", {})
    ip_v4 = next((ip for ip in self_info.get("TailscaleIPs", []) if ":" not in ip), None)
    return TailscaleInfo(
        status=_STATUS_MAP.get(data.get("BackendState", ""), TailscaleStatus.ERROR),
        ip_address=ip_v4,
        hostname=self_info.get("HostName", "unknown"),
        peer_count=len(data.get("Peer", {})),
        last_check=datetime.now(),
    )


class TailscaleManager:
    """Polls Tailscale status on a daemon thread; reconnects when down."""

    def __init__(self, check_interval: float = 10.0, auto_reconnect: bool = True) -> None:
        self._check_interval = check_interval
        self._auto_reconnect = auto_reconnect
        self._info = TailscaleInfo(status=TailscaleStatus.DISCONNECTED)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def info(self) -> TailscaleInfo:
        return self._info

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._monitor_loop, name="tailscale-monitor", daemon=True)
        self._thread.start()
        logger.info("TailscaleManager started (interval=%.0fs)", self._check_interval)

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        logger.info("TailscaleManager stopped")

    def _monitor_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._check_status()
                if self._auto_reconnect and self._info.status in (
                    TailscaleStatus.DISCONNECTED,
                    TailscaleStatus.ERROR,
                ):
                    self._reconnect()
            except Exception as e:  # noqa: BLE001 - monitor must never die
                logger.error("Tailscale monitor error: %s", e)
            self._stop_event.wait(self._check_interval)

    def _check_status(self) -> None:
        exit_code, stdout = _run(["tailscale", "status", "--json"])
        if exit_code == 127:
            self._info = TailscaleInfo(status=TailscaleStatus.NOT_INSTALLED, last_check=datetime.now())
            return
        if exit_code != 0:
            self._info = TailscaleInfo(status=TailscaleStatus.ERROR, last_check=datetime.now())
            return
        try:
            self._info = parse_status_json(json.loads(stdout))
        except (json.JSONDecodeError, TypeError) as e:
            logger.error("Failed to parse tailscale status JSON: %s", e)
            self._info = TailscaleInfo(status=TailscaleStatus.ERROR, last_check=datetime.now())

    def _reconnect(self) -> None:
        logger.info("Tailscale auto-reconnect triggered")
        exit_code, _ = _run(["tailscale", "up"], timeout=30.0)
        if exit_code != 0:
            logger.warning("tailscale up failed (exit %d)", exit_code)


def _run(cmd: list[str], timeout: float = 10.0) -> tuple[int, str]:
    """Run a command, returning (exit_code, stdout); 127 when not installed."""
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return result.returncode, result.stdout
    except FileNotFoundError:
        return 127, ""
    except Exception as e:  # noqa: BLE001 - probe failure is a soft error
        logger.debug("Command %s failed: %s", cmd[0], e)
        return 1, ""
