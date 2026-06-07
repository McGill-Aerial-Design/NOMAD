# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Network Monitor

Monitors 4G/LTE modem and overall network health for the Jetson Orin Nano.
Provides connectivity metrics for the Edge Core API.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import asyncio
import logging
import os
import re
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any

logger = logging.getLogger(__name__)


class ConnectionType(Enum):
    """Network connection type."""

    LTE_4G = "4g_lte"
    LTE_5G = "5g"
    WIFI = "wifi"
    ETHERNET = "ethernet"
    UNKNOWN = "unknown"
    NONE = "none"


class SignalQuality(Enum):
    """Signal quality classification."""

    EXCELLENT = "excellent"  # RSRP >= -80 dBm
    GOOD = "good"  # RSRP >= -90 dBm
    FAIR = "fair"  # RSRP >= -100 dBm
    POOR = "poor"  # RSRP >= -110 dBm
    NO_SIGNAL = "no_signal"  # RSRP < -110 dBm or no connection


@dataclass
class ModemStatus:
    """4G/LTE modem status."""

    connected: bool = False
    signal_strength_dbm: int | None = None  # RSRP: -140 to -44 dBm
    signal_quality: SignalQuality = SignalQuality.NO_SIGNAL
    signal_percent: int = 0  # 0-100%
    carrier: str | None = None  # "AT&T", "Verizon", etc.
    technology: str | None = None  # "LTE", "5G NR", "HSPA+"
    ip_address: str | None = None
    interface: str | None = None  # "wwan0", "usb0"
    imei: str | None = None
    model: str | None = None

    # NetworkManager / connection-profile fields
    nm_connection_name: str | None = None  # e.g. "NOMAD-LTE"
    nm_connection_state: str | None = None  # "activated", "activating", ...
    apn: str | None = None

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for API response."""
        return {
            "connected": self.connected,
            "signal_strength_dbm": self.signal_strength_dbm,
            "signal_quality": self.signal_quality.value,
            "signal_percent": self.signal_percent,
            "carrier": self.carrier,
            "technology": self.technology,
            "ip_address": self.ip_address,
            "interface": self.interface,
            "imei": self.imei,
            "model": self.model,
            "nm_connection_name": self.nm_connection_name,
            "nm_connection_state": self.nm_connection_state,
            "apn": self.apn,
        }


@dataclass
class NetworkStatus:
    """Overall network status."""

    connection_type: ConnectionType = ConnectionType.NONE
    internet_reachable: bool = False
    modem: ModemStatus | None = None
    tailscale_reachable: bool = False
    latency_to_internet_ms: float | None = None
    latency_to_gcs_ms: float | None = None
    gcs_ip: str | None = None
    last_check: datetime = field(default_factory=datetime.now)

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for API response."""
        return {
            "connection_type": self.connection_type.value,
            "internet_reachable": self.internet_reachable,
            "modem": self.modem.to_dict() if self.modem else None,
            "tailscale_reachable": self.tailscale_reachable,
            "latency_to_internet_ms": self.latency_to_internet_ms,
            "latency_to_gcs_ms": self.latency_to_gcs_ms,
            "gcs_ip": self.gcs_ip,
            "last_check": self.last_check.isoformat(),
        }


def _rsrp_to_quality(rsrp: int | None) -> SignalQuality:
    """Convert RSRP (dBm) to signal quality classification."""
    if rsrp is None:
        return SignalQuality.NO_SIGNAL
    if rsrp >= -80:
        return SignalQuality.EXCELLENT
    if rsrp >= -90:
        return SignalQuality.GOOD
    if rsrp >= -100:
        return SignalQuality.FAIR
    if rsrp >= -110:
        return SignalQuality.POOR
    return SignalQuality.NO_SIGNAL


def _rsrp_to_percent(rsrp: int | None) -> int:
    """Convert RSRP (dBm) to percentage (0-100)."""
    if rsrp is None:
        return 0
    # RSRP range: -140 dBm (worst) to -44 dBm (best)
    # Map to 0-100%
    percent = int(((rsrp + 140) / 96) * 100)
    return max(0, min(100, percent))


class NetworkMonitor:
    """
    Monitors network connectivity for NOMAD.

    Features:
    - Monitor 4G/LTE modem signal strength
    - Check internet connectivity
    - Measure latency to Ground Station
    - Provide metrics for API

    Usage:
        monitor = NetworkMonitor(gcs_tailscale_ip="<gcs-ip>")
        await monitor.start()
        status = monitor.status
        await monitor.stop()
    """

    # NetworkManager connection profiles to look for, in priority order.
    # Includes both the canonical "NOMAD-LTE" name and the typical CDC-ECM
    # cellular-router profile ("LTE-ECM") used by USB tethered LTE modems
    # that expose themselves as an Ethernet device rather than a true
    # cellular modem (these never appear in mmcli).
    NOMAD_LTE_CONNECTION_CANDIDATES = ["NOMAD-LTE", "LTE-ECM"]
    # Substrings used to identify an LTE profile when none of the named
    # candidates exist (case-insensitive match against NM connection name).
    LTE_NAME_HINTS = ("lte", "ecm", "wwan", "cellular", "modem", "4g", "5g")

    def __init__(
        self,
        gcs_tailscale_ip: str | None = None,
        check_interval: float = 30.0,
        internet_check_host: str = "8.8.8.8",
        nm_connection_name: str | None = None,
    ):
        """
        Initialize network monitor.

        Args:
            gcs_tailscale_ip: Tailscale IP of Ground Control Station
            check_interval: Seconds between status checks
            internet_check_host: Host to ping for internet connectivity
            nm_connection_name: NetworkManager profile name owning the LTE
                modem (default ``NOMAD-LTE``; overridable via
                ``NOMAD_LTE_CONNECTION`` env var).
        """
        self._gcs_ip = gcs_tailscale_ip
        self._check_interval = check_interval
        self._internet_host = internet_check_host
        explicit = nm_connection_name or os.environ.get("NOMAD_LTE_CONNECTION")
        self._nm_conn_candidates = [explicit] if explicit else list(self.NOMAD_LTE_CONNECTION_CANDIDATES)

        self._status = NetworkStatus()
        self._running = False
        self._task: asyncio.Task[None] | None = None

    @property
    def status(self) -> NetworkStatus:
        """Get current network status."""
        return self._status

    @property
    def gcs_ip(self) -> str | None:
        """Get configured GCS IP."""
        return self._gcs_ip

    @gcs_ip.setter
    def gcs_ip(self, value: str | None) -> None:
        """Set GCS IP address."""
        self._gcs_ip = value

    async def start(self) -> None:
        """Start network monitoring."""
        if self._running:
            logger.warning("NetworkMonitor already running")
            return

        self._running = True
        logger.info(f"NetworkMonitor starting (interval={self._check_interval}s)")

        # Do initial check
        await self.check_connectivity()

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

        logger.info("NetworkMonitor stopped")

    async def check_connectivity(self) -> NetworkStatus:
        """Perform full connectivity check and update status."""
        status = NetworkStatus(last_check=datetime.now())

        # Check modem status
        status.modem = await self._check_modem_status()

        # Determine connection type
        status.connection_type = await self._determine_connection_type()

        # Check internet connectivity
        internet_latency = await self._ping_host(self._internet_host)
        status.internet_reachable = internet_latency is not None
        status.latency_to_internet_ms = internet_latency

        # Check GCS connectivity
        if self._gcs_ip:
            status.gcs_ip = self._gcs_ip
            gcs_latency = await self._ping_host(self._gcs_ip)
            status.tailscale_reachable = gcs_latency is not None
            status.latency_to_gcs_ms = gcs_latency

        self._status = status
        return status

    async def ping(self, host: str, count: int = 3) -> dict[str, Any]:
        """
        Ping arbitrary host and return results.

        Args:
            host: IP or hostname to ping
            count: Number of ping packets

        Returns:
            Dict with latency_ms, packets_sent, packets_received
        """
        latency = await self._ping_host(host, count)

        return {
            "host": host,
            "latency_ms": latency,
            "packets_sent": count,
            "packets_received": count if latency else 0,
            "reachable": latency is not None,
        }

    async def _monitor_loop(self) -> None:
        """Background monitoring loop."""
        while self._running:
            try:
                await self.check_connectivity()
                await asyncio.sleep(self._check_interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Monitor loop error: {e}")
                await asyncio.sleep(self._check_interval)

    async def _check_modem_status(self) -> ModemStatus | None:
        """
        Query 4G/LTE modem status.

        Combines two sources:
        - ``mmcli`` (ModemManager) for the modem-hardware view: model, carrier,
          technology, RSRP, IMEI.
        - ``nmcli`` (NetworkManager) for the connection-profile view:
          NOMAD-LTE state, bound interface and IPv4 address.

        Either source on its own is incomplete. ``mmcli`` doesn't always
        surface the interface/IP the kernel ended up using, and ``nmcli``
        doesn't expose RF signal strength. We merge them.

        Returns:
            ModemStatus or None if no modem and no LTE connection are found.
        """
        status = ModemStatus()
        got_anything = False

        # ---- Pass 1: NetworkManager (lightweight, works even without MM) ----
        # Try each configured candidate in order, fall back to fuzzy LTE match.
        try:
            nm_info = None
            for cand in self._nm_conn_candidates:
                nm_info = await self._query_nm_connection(cand)
                if nm_info:
                    break
            if nm_info is None:
                nm_info = await self._query_nm_connection_fuzzy()
            if nm_info:
                status.nm_connection_name = nm_info.get("name")
                status.nm_connection_state = nm_info.get("state")
                status.interface = nm_info.get("interface")
                status.ip_address = nm_info.get("ip4")
                status.apn = nm_info.get("apn")
                # NM "activated" => data session is up
                if nm_info.get("state") == "activated":
                    status.connected = True
                got_anything = True
        except Exception as e:
            logger.debug(f"nmcli query failed: {e}")

        # ---- Pass 2: ModemManager (probe every modem, not just /Modem/0) ----
        try:
            mm_info = await self._query_modemmanager()
            if mm_info:
                # Don't let a later mmcli "disconnected" flip an NM "activated"
                # — NM is authoritative for the data session.
                if not status.connected:
                    status.connected = mm_info.get("connected", False)
                status.model = mm_info.get("model") or status.model
                status.carrier = mm_info.get("carrier") or status.carrier
                status.technology = mm_info.get("technology") or status.technology
                status.imei = mm_info.get("imei") or status.imei
                status.signal_strength_dbm = mm_info.get("rsrp_dbm")
                if status.signal_strength_dbm is not None:
                    status.signal_quality = _rsrp_to_quality(status.signal_strength_dbm)
                    status.signal_percent = _rsrp_to_percent(status.signal_strength_dbm)
                # Fall back to mmcli-reported interface if NM didn't have one.
                if not status.interface:
                    status.interface = mm_info.get("interface")
                got_anything = True
        except FileNotFoundError:
            logger.debug("mmcli not found - ModemManager not installed")
        except Exception as e:
            logger.debug(f"mmcli query failed: {e}")

        # ---- Pass 3: derive interface/IP from `ip addr` if still missing ----
        if got_anything and (not status.interface or not status.ip_address):
            try:
                iface, ip4 = await self._guess_modem_interface()
                if not status.interface:
                    status.interface = iface
                if not status.ip_address:
                    status.ip_address = ip4
            except Exception as e:
                logger.debug(f"interface guess failed: {e}")

        return status if got_anything else None

    async def _query_nm_connection(self, conn_name: str) -> dict[str, Any] | None:
        """
        Look up a NetworkManager connection by name. Returns dict with
        state/interface/ip4/apn, or None if NetworkManager is missing or the
        connection doesn't exist.
        """
        # Confirm the profile exists at all.
        exit_code, stdout, _ = await self._run_command(
            ["nmcli", "-t", "-f", "NAME,TYPE,DEVICE,STATE", "connection", "show"]
        )
        if exit_code != 0:
            return None

        matched = None
        for line in stdout.splitlines():
            # Format: NAME:TYPE:DEVICE:STATE
            parts = line.split(":")
            if len(parts) < 4:
                continue
            name, ctype, device, state = parts[0], parts[1], parts[2], parts[3]
            if name == conn_name:
                matched = {
                    "name": name,
                    "type": ctype,
                    "device": device or None,
                    "state": state or None,
                }
                break

        if not matched:
            return None

        # Pull APN + ipv4 details from the connection profile.
        exit_code, stdout, _ = await self._run_command(
            [
                "nmcli",
                "-t",
                "-f",
                "gsm.apn,GENERAL.STATE,IP4.ADDRESS,GENERAL.DEVICES",
                "connection",
                "show",
                matched["name"],
            ]
        )
        result: dict[str, Any] = {
            "name": matched["name"],
            "state": matched["state"],
            "interface": matched.get("device"),
        }
        if exit_code == 0:
            for line in stdout.splitlines():
                if ":" not in line:
                    continue
                k, _, v = line.partition(":")
                v = v.strip()
                if not v:
                    continue
                key = k.strip().lower()
                if key == "gsm.apn":
                    result["apn"] = v
                elif key == "ip4.address[1]" or key == "ip4.address":
                    # nmcli prints "192.0.2.10/24"
                    result["ip4"] = v.split("/")[0]
                elif key == "general.devices":
                    result["interface"] = v
                elif key == "general.state":
                    # Overrides the brief state from `connection show`.
                    # Formats like "100 (activated)" — extract the word.
                    word = re.search(r"\((\w+)\)", v)
                    if word:
                        result["state"] = word.group(1)
                    else:
                        result["state"] = v
        return result

    async def _query_nm_connection_fuzzy(self) -> dict[str, Any] | None:
        """
        Locate an LTE NetworkManager profile by heuristic when no named
        candidate matched. Picks, in order:

        1. Any *activated* connection of type ``gsm``/``cdma``/``wwan``.
        2. Any *activated* connection whose name looks LTE-related (matches
           ``LTE_NAME_HINTS``). This catches USB tethered modems running in
           CDC-ECM mode, which appear as 802-3-ethernet to NM.
        3. Same as 2 but allowing inactive profiles, so a configured-but-
           down modem still surfaces in the UI.

        Returns the same dict shape as ``_query_nm_connection``.
        """
        exit_code, stdout, _ = await self._run_command(
            ["nmcli", "-t", "-f", "NAME,TYPE,DEVICE,STATE", "connection", "show"]
        )
        if exit_code != 0:
            return None

        # Walk the connections once; classify each into a bucket
        cellular_active: list[dict[str, Any]] = []
        named_active: list[dict[str, Any]] = []
        named_inactive: list[dict[str, Any]] = []

        for line in stdout.splitlines():
            parts = line.split(":")
            if len(parts) < 4:
                continue
            name, ctype, device, state = parts
            row = {
                "name": name,
                "type": ctype,
                "device": device or None,
                "state": state or None,
            }
            name_lower = name.lower()
            looks_lte = any(h in name_lower for h in self.LTE_NAME_HINTS)

            if state and ctype in ("gsm", "cdma", "wwan"):
                cellular_active.append(row)
            elif state and looks_lte:
                named_active.append(row)
            elif looks_lte:
                named_inactive.append(row)

        match = (
            (cellular_active[0] if cellular_active else None)
            or (named_active[0] if named_active else None)
            or (named_inactive[0] if named_inactive else None)
        )
        if match is None:
            return None

        # Re-use _query_nm_connection's IP/APN lookup for the resolved name.
        full = await self._query_nm_connection(match["name"])
        return full if full else match

    async def _query_modemmanager(self) -> dict[str, Any] | None:
        """
        Iterate over every modem ModemManager knows about, return aggregated
        info for the first one that reports any usable data.
        """
        exit_code, stdout, _ = await self._run_command(["mmcli", "-L"])
        if exit_code != 0 or "No modems" in stdout:
            return None

        modem_idxs = re.findall(r"/Modem/(\d+)", stdout)
        if not modem_idxs:
            return None

        for idx in modem_idxs:
            info = await self._read_modem(idx)
            if info:
                return info
        return None

    async def _read_modem(self, idx: str) -> dict[str, Any] | None:
        """Pull a single modem's full picture out of mmcli."""
        exit_code, stdout, _ = await self._run_command(["mmcli", "-m", idx])
        if exit_code != 0:
            return None

        info: dict[str, Any] = {}

        # State — mmcli writes "state: 'connected'" but versions vary.
        state_match = re.search(r"state:\s*'?(\w+)'?", stdout, re.IGNORECASE)
        if state_match:
            info["connected"] = state_match.group(1).lower() == "connected"

        for key_re, dest in (
            (r"model:\s*(.+)", "model"),
            (r"access tech(?:nologies)?:\s*(.+)", "technology"),
            (r"operator name:\s*(.+)", "carrier"),
            (r"\bimei:\s*(.+)", "imei"),
            (r"primary port:\s*(\S+)", "interface"),
        ):
            m = re.search(key_re, stdout, re.IGNORECASE)
            if m:
                val = m.group(1).strip()
                # mmcli often pads with "--" for unset values
                if val and val not in ("--", "unknown"):
                    info[dest] = val

        # Signal — try --signal-get first (works on most recent mmcli versions)
        exit_code, sig_out, _ = await self._run_command(["mmcli", "-m", idx, "--signal-get"])
        rsrp = None
        if exit_code == 0:
            m = re.search(r"rsrp:\s*([-\d.]+)\s*dBm", sig_out)
            if m:
                rsrp = int(float(m.group(1)))

        # Fall back to the generic "signal quality" percent from mmcli -m
        if rsrp is None:
            m = re.search(r"signal quality:\s*(\d+)%", stdout, re.IGNORECASE)
            if m:
                # Reverse-map the percent into a rough dBm; better than nothing.
                pct = int(m.group(1))
                rsrp = int(round(pct / 100.0 * 96 - 140))

        if rsrp is not None:
            info["rsrp_dbm"] = rsrp

        # If we couldn't infer the kernel interface, try the bearer info.
        if "interface" not in info:
            exit_code, b_out, _ = await self._run_command(["mmcli", "-m", idx, "--bearer", "0"])
            if exit_code == 0:
                m = re.search(r"interface:\s*(\S+)", b_out, re.IGNORECASE)
                if m and m.group(1) not in ("--", "unknown"):
                    info["interface"] = m.group(1).strip()

        return info if info else None

    async def _guess_modem_interface(self) -> tuple[str | None, str | None]:
        """
        Last-ditch fallback when neither ModemManager nor NetworkManager
        gave us an interface name. Walk ``ip -4 addr`` and pick the first
        wwan*/usb*/cdc-wdm* interface that has an IPv4.
        """
        exit_code, stdout, _ = await self._run_command(["ip", "-4", "-o", "addr", "show"])
        if exit_code != 0:
            return None, None
        for line in stdout.splitlines():
            # "3: wwan0    inet 10.50.10.2/30 ..."
            parts = line.split()
            if len(parts) < 4:
                continue
            iface = parts[1]
            if not (
                iface.startswith("wwan")
                or iface.startswith("usb")
                or iface.startswith("cdc")
                or iface.startswith("ppp")
            ):
                continue
            ip4 = parts[3].split("/")[0]
            return iface, ip4
        return None, None

    async def _determine_connection_type(self) -> ConnectionType:
        """Determine primary network connection type."""
        try:
            # Check for wwan/mobile interface
            exit_code, stdout, _ = await self._run_command(["ip", "link", "show"])

            if exit_code == 0:
                if "wwan" in stdout or "usb0" in stdout:
                    # Check if 5G or LTE
                    if self._status.modem and self._status.modem.technology:
                        if "5g" in self._status.modem.technology.lower():
                            return ConnectionType.LTE_5G
                    return ConnectionType.LTE_4G

                if "wlan" in stdout:
                    return ConnectionType.WIFI

                if "eth" in stdout:
                    return ConnectionType.ETHERNET

            return ConnectionType.UNKNOWN

        except Exception:
            return ConnectionType.UNKNOWN

    async def _ping_host(self, host: str, count: int = 3, timeout: float = 5.0) -> float | None:
        """
        Ping host and return average latency in ms.

        Args:
            host: IP or hostname to ping
            count: Number of ping packets
            timeout: Total timeout in seconds

        Returns:
            Average latency in ms, or None if unreachable
        """
        try:
            exit_code, stdout, stderr = await self._run_command(
                ["ping", "-c", str(count), "-W", "2", host],
                timeout=timeout,
            )

            if exit_code != 0:
                return None

            # Parse average latency from ping output
            # Example: "rtt min/avg/max/mdev = 10.5/15.2/20.1/3.2 ms"
            match = re.search(r"rtt.*=\s*[\d.]+/([\d.]+)/", stdout)
            if match:
                return float(match.group(1))

            # Alternative format: "time=XX ms"
            times = re.findall(r"time=([\d.]+)\s*ms", stdout)
            if times:
                return sum(float(t) for t in times) / len(times)

            return None

        except Exception as e:
            logger.debug(f"Ping error for {host}: {e}")
            return None

    async def _run_command(self, cmd: list[str], timeout: float = 10.0) -> tuple[int, str, str]:
        """Run shell command and return (exit_code, stdout, stderr)."""
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
            return (1, "", "Command timed out")
        except FileNotFoundError:
            return (127, "", f"Command not found: {cmd[0]}")
        except Exception as e:
            return (1, "", str(e))


# ============================================================
# Module-level singleton access
# ============================================================

_monitor: NetworkMonitor | None = None


def get_network_monitor() -> NetworkMonitor | None:
    """Get the global NetworkMonitor instance."""
    return _monitor


def init_network_monitor(
    gcs_tailscale_ip: str | None = None,
    check_interval: float = 30.0,
) -> NetworkMonitor:
    """
    Initialize the global NetworkMonitor instance.

    Args:
        gcs_tailscale_ip: Tailscale IP of Ground Control Station
        check_interval: Seconds between status checks

    Returns:
        The initialized NetworkMonitor instance
    """
    global _monitor
    _monitor = NetworkMonitor(
        gcs_tailscale_ip=gcs_tailscale_ip,
        check_interval=check_interval,
    )
    return _monitor
