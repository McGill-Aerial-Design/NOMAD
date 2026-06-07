# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Tailscale Manager

Monitors and manages Tailscale VPN connection for secure 4G/LTE communication
between the Jetson Orin Nano (drone) and Ground Station.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import asyncio
import json
import logging
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any

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
class TailscalePeer:
    """Information about a connected Tailscale peer."""

    hostname: str
    ip_address: str
    online: bool
    last_seen: str | None = None
    os: str | None = None
    relay: str | None = None  # DERP relay if not direct


@dataclass
class TailscaleInfo:
    """Current Tailscale connection info."""

    status: TailscaleStatus
    ip_address: str | None = None
    ip_v6: str | None = None
    hostname: str = "unknown"
    dns_name: str | None = None
    backend_state: str | None = None
    peers: list[TailscalePeer] = field(default_factory=list)
    derp_relay: str | None = None  # DERP server if relayed
    exit_node: str | None = None
    exit_node_ip: str | None = None
    online: bool = False
    version: str | None = None
    last_check: datetime = field(default_factory=datetime.now)

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for API response."""
        return {
            "status": self.status.value,
            "ip_address": self.ip_address,
            "ip_v6": self.ip_v6,
            "hostname": self.hostname,
            "dns_name": self.dns_name,
            "backend_state": self.backend_state,
            "peer_count": len(self.peers),
            "peers": [
                {
                    "hostname": p.hostname,
                    "ip_address": p.ip_address,
                    "online": p.online,
                    "relay": p.relay,
                }
                for p in self.peers
            ],
            "derp_relay": self.derp_relay,
            "online": self.online,
            "version": self.version,
            "last_check": self.last_check.isoformat(),
        }


class TailscaleManager:
    """
    Manages Tailscale VPN connection for NOMAD.

    Features:
    - Monitor connection status via `tailscale status --json`
    - Auto-reconnect on disconnection
    - Provide health metrics for API
    - Callback on status changes

    Usage:
        manager = TailscaleManager(on_status_change=my_callback)
        await manager.start()
        info = manager.info
        await manager.stop()
    """

    def __init__(
        self,
        hostname: str = "nomad-jetson",
        check_interval: float = 10.0,
        auto_reconnect: bool = True,
        on_status_change: Callable[[TailscaleInfo], None] | None = None,
    ):
        """
        Initialize Tailscale manager.

        Args:
            hostname: Tailscale hostname for this device
            check_interval: Seconds between status checks
            auto_reconnect: Whether to auto-reconnect on disconnection
            on_status_change: Callback when status changes
        """
        self._hostname = hostname
        self._check_interval = check_interval
        self._auto_reconnect = auto_reconnect
        self._on_status_change = on_status_change

        self._info = TailscaleInfo(status=TailscaleStatus.DISCONNECTED)
        self._running = False
        self._task: asyncio.Task[None] | None = None
        self._prev_status: TailscaleStatus | None = None

    @property
    def info(self) -> TailscaleInfo:
        """Get current Tailscale info."""
        return self._info

    @property
    def is_connected(self) -> bool:
        """Check if Tailscale is connected."""
        return self._info.status == TailscaleStatus.CONNECTED

    @property
    def ip_address(self) -> str | None:
        """Get Tailscale IP address (100.x.x.x)."""
        return self._info.ip_address

    async def start(self) -> None:
        """Start monitoring Tailscale connection."""
        if self._running:
            logger.warning("TailscaleManager already running")
            return

        self._running = True
        logger.info(f"TailscaleManager starting (interval={self._check_interval}s)")

        # Do initial check
        await self._check_status()

        # Start monitoring task
        self._task = asyncio.create_task(self._monitor_loop())

    async def stop(self) -> None:
        """Stop monitoring."""
        if not self._running:
            return

        self._running = False

        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

        logger.info("TailscaleManager stopped")

    async def _monitor_loop(self) -> None:
        """Background monitoring loop."""
        while self._running:
            try:
                await self._check_status()

                # Auto-reconnect if needed
                if self._auto_reconnect and self._info.status in (TailscaleStatus.DISCONNECTED, TailscaleStatus.ERROR):
                    logger.info("Auto-reconnect triggered")
                    await self.reconnect()

                await asyncio.sleep(self._check_interval)

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Monitor loop error: {e}")
                await asyncio.sleep(self._check_interval)

    async def _check_status(self) -> None:
        """Query Tailscale CLI for current status."""
        try:
            # Check if tailscale is installed
            if not await self._is_tailscale_installed():
                self._update_info(TailscaleInfo(status=TailscaleStatus.NOT_INSTALLED))
                return

            # Get status JSON
            exit_code, stdout, stderr = await self._run_command(["tailscale", "status", "--json"])

            if exit_code != 0:
                logger.warning(f"tailscale status failed: {stderr}")
                self._update_info(TailscaleInfo(status=TailscaleStatus.ERROR))
                return

            # Parse JSON
            try:
                data = json.loads(stdout)
            except json.JSONDecodeError as e:
                logger.error(f"Failed to parse tailscale status JSON: {e}")
                self._update_info(TailscaleInfo(status=TailscaleStatus.ERROR))
                return

            # Build info from parsed data
            info = self._parse_status_json(data)
            self._update_info(info)

        except Exception as e:
            logger.error(f"Status check error: {e}")
            self._update_info(TailscaleInfo(status=TailscaleStatus.ERROR))

    def _parse_status_json(self, data: dict[str, Any]) -> TailscaleInfo:
        """Parse tailscale status --json output."""
        # Get backend state
        backend_state = data.get("BackendState", "")

        # Determine status from backend state
        status_map = {
            "Running": TailscaleStatus.CONNECTED,
            "Starting": TailscaleStatus.CONNECTING,
            "NeedsLogin": TailscaleStatus.NEEDS_AUTH,
            "NeedsMachineAuth": TailscaleStatus.NEEDS_AUTH,
            "Stopped": TailscaleStatus.DISCONNECTED,
        }
        status = status_map.get(backend_state, TailscaleStatus.ERROR)

        # Get self info
        self_info = data.get("Self", {})
        tailscale_ips = self_info.get("TailscaleIPs", [])

        ip_v4 = None
        ip_v6 = None
        for ip in tailscale_ips:
            if ":" in ip:
                ip_v6 = ip
            else:
                ip_v4 = ip

        # Parse peers
        peers = []
        peer_data = data.get("Peer", {})
        for peer_key, peer_info in peer_data.items():
            peer_ips = peer_info.get("TailscaleIPs", [])
            peer_ip = peer_ips[0] if peer_ips else None

            peers.append(
                TailscalePeer(
                    hostname=peer_info.get("HostName", "unknown"),
                    ip_address=peer_ip,
                    online=peer_info.get("Online", False),
                    last_seen=peer_info.get("LastSeen"),
                    os=peer_info.get("OS"),
                    relay=peer_info.get("Relay"),
                )
            )

        return TailscaleInfo(
            status=status,
            ip_address=ip_v4,
            ip_v6=ip_v6,
            hostname=self_info.get("HostName", "unknown"),
            dns_name=self_info.get("DNSName"),
            backend_state=backend_state,
            peers=peers,
            online=self_info.get("Online", False),
            version=data.get("Version"),
            last_check=datetime.now(),
        )

    def _update_info(self, info: TailscaleInfo) -> None:
        """Update info and trigger callback if status changed."""
        status_changed = self._prev_status != info.status
        self._info = info
        self._prev_status = info.status

        if status_changed:
            logger.info(f"Tailscale status: {info.status.value}")
            if self._on_status_change:
                try:
                    self._on_status_change(info)
                except Exception as e:
                    logger.error(f"Status change callback error: {e}")

    async def _is_tailscale_installed(self) -> bool:
        """Check if tailscale CLI is available."""
        try:
            exit_code, _, _ = await self._run_command(["tailscale", "version"])
            return exit_code == 0
        except FileNotFoundError:
            return False

    async def _run_command(self, cmd: list[str], timeout: float = 30.0) -> tuple[int, str, str]:
        """
        Run shell command and return (exit_code, stdout, stderr).

        Args:
            cmd: Command and arguments
            timeout: Command timeout in seconds

        Returns:
            Tuple of (exit_code, stdout, stderr)
        """
        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )

            stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=timeout)

            return (
                proc.returncode or 0,
                stdout.decode("utf-8", errors="replace"),
                stderr.decode("utf-8", errors="replace"),
            )

        except asyncio.TimeoutError:
            logger.warning(f"Command timed out: {' '.join(cmd)}")
            return (1, "", "Command timed out")
        except FileNotFoundError:
            return (127, "", f"Command not found: {cmd[0]}")
        except Exception as e:
            return (1, "", str(e))


# ============================================================
# Module-level singleton access
# ============================================================

_manager: TailscaleManager | None = None


def get_tailscale_manager() -> TailscaleManager | None:
    """Get the global TailscaleManager instance."""
    return _manager


def init_tailscale_manager(
    hostname: str = "nomad-jetson",
    check_interval: float = 10.0,
    auto_reconnect: bool = True,
    on_status_change: Callable[[TailscaleInfo], None] | None = None,
) -> TailscaleManager:
    """
    Initialize the global TailscaleManager instance.

    Args:
        hostname: Tailscale hostname for this device
        check_interval: Seconds between status checks
        auto_reconnect: Whether to auto-reconnect on disconnection
        on_status_change: Callback when status changes

    Returns:
        The initialized TailscaleManager instance
    """
    global _manager
    _manager = TailscaleManager(
        hostname=hostname,
        check_interval=check_interval,
        auto_reconnect=auto_reconnect,
        on_status_change=on_status_change,
    )
    return _manager
