# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""4G/LTE modem and connectivity monitor for NOMAD.

Polls modem state (NetworkManager + ModemManager) and internet/GCS
reachability on a background thread. Exposes the surface the
``/network/status`` route and the GCS dashboard consume:
``internet_reachable``, ``tailscale_reachable``, and ``modem`` (full
``ModemStatus.to_dict()`` — interface, ip_address, NM connection
name/state, APN, model, IMEI, signal).
"""

from __future__ import annotations

import logging
import os
import re
import subprocess
import threading
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any

logger = logging.getLogger(__name__)


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
    carrier: str | None = None
    technology: str | None = None  # "LTE", "5G NR", "HSPA+"
    ip_address: str | None = None
    interface: str | None = None  # "wwan0", "usb0"
    imei: str | None = None
    model: str | None = None
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
    """Overall network status (the consumed surface only)."""

    internet_reachable: bool = False
    tailscale_reachable: bool = False
    modem: ModemStatus | None = None
    gcs_ip: str | None = None
    last_check: datetime = field(default_factory=datetime.now)


def _rsrp_to_quality(rsrp: int | None) -> SignalQuality:
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
    if rsrp is None:
        return 0
    # RSRP range: -140 dBm (worst) to -44 dBm (best), mapped to 0-100%.
    return max(0, min(100, int(((rsrp + 140) / 96) * 100)))


class NetworkMonitor:
    """Polls modem + connectivity state on a daemon thread."""

    # NetworkManager connection profiles to look for, in priority order.
    # "LTE-ECM" covers USB tethered modems in CDC-ECM mode that appear as
    # Ethernet devices and never show up in mmcli.
    NOMAD_LTE_CONNECTION_CANDIDATES = ["NOMAD-LTE", "LTE-ECM"]
    # Substrings identifying an LTE profile when no named candidate exists.
    LTE_NAME_HINTS = ("lte", "ecm", "wwan", "cellular", "modem", "4g", "5g")

    def __init__(
        self,
        gcs_tailscale_ip: str | None = None,
        check_interval: float = 30.0,
        internet_check_host: str = "8.8.8.8",
        nm_connection_name: str | None = None,
    ) -> None:
        self._gcs_ip = gcs_tailscale_ip
        self._check_interval = check_interval
        self._internet_host = internet_check_host
        explicit = nm_connection_name or os.environ.get("NOMAD_LTE_CONNECTION")
        self._nm_conn_candidates = [explicit] if explicit else list(self.NOMAD_LTE_CONNECTION_CANDIDATES)
        self._status = NetworkStatus()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def status(self) -> NetworkStatus:
        return self._status

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._monitor_loop, name="network-monitor", daemon=True)
        self._thread.start()
        logger.info("NetworkMonitor started (interval=%.0fs)", self._check_interval)

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        logger.info("NetworkMonitor stopped")

    def _monitor_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.check_connectivity()
            except Exception as e:  # noqa: BLE001 - monitor must never die
                logger.error("Network monitor error: %s", e)
            self._stop_event.wait(self._check_interval)

    def check_connectivity(self) -> NetworkStatus:
        """Perform a full connectivity check and update ``status``."""
        status = NetworkStatus(last_check=datetime.now())
        status.modem = self._check_modem_status()
        status.internet_reachable = self._ping_host(self._internet_host) is not None
        if self._gcs_ip:
            status.gcs_ip = self._gcs_ip
            status.tailscale_reachable = self._ping_host(self._gcs_ip) is not None
        self._status = status
        return status

    def _check_modem_status(self) -> ModemStatus | None:
        """Merge the NetworkManager and ModemManager views of the modem.

        ``nmcli`` knows the connection profile (state, interface, IP, APN) but
        not RF signal; ``mmcli`` knows the hardware (model, carrier, RSRP,
        IMEI) but not always the kernel interface. Either alone is incomplete.
        """
        status = ModemStatus()
        got_anything = False

        nm_info = None
        for cand in self._nm_conn_candidates:
            nm_info = self._query_nm_connection(cand)
            if nm_info:
                break
        if nm_info is None:
            nm_info = self._query_nm_connection_fuzzy()
        if nm_info:
            status.nm_connection_name = nm_info.get("name")
            status.nm_connection_state = nm_info.get("state")
            status.interface = nm_info.get("interface")
            status.ip_address = nm_info.get("ip4")
            status.apn = nm_info.get("apn")
            # NM "activated" => data session is up.
            if nm_info.get("state") == "activated":
                status.connected = True
            got_anything = True

        mm_info = self._query_modemmanager()
        if mm_info:
            # NM is authoritative for the data session; don't let a stale
            # mmcli "disconnected" flip an NM "activated".
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
            if not status.interface:
                status.interface = mm_info.get("interface")
            got_anything = True

        if got_anything and (not status.interface or not status.ip_address):
            iface, ip4 = self._guess_modem_interface()
            status.interface = status.interface or iface
            status.ip_address = status.ip_address or ip4

        return status if got_anything else None

    def _query_nm_connection(self, conn_name: str) -> dict[str, Any] | None:
        """Look up a NetworkManager connection by name (state/interface/ip4/apn)."""
        exit_code, stdout = _run(["nmcli", "-t", "-f", "NAME,TYPE,DEVICE,STATE", "connection", "show"])
        if exit_code != 0:
            return None

        matched = None
        for line in stdout.splitlines():
            parts = line.split(":")  # NAME:TYPE:DEVICE:STATE
            if len(parts) >= 4 and parts[0] == conn_name:
                matched = {"name": parts[0], "state": parts[3] or None, "interface": parts[2] or None}
                break
        if not matched:
            return None

        exit_code, stdout = _run(
            ["nmcli", "-t", "-f", "gsm.apn,GENERAL.STATE,IP4.ADDRESS,GENERAL.DEVICES", "connection", "show", conn_name]
        )
        if exit_code == 0:
            for line in stdout.splitlines():
                key, _, value = line.partition(":")
                value = value.strip()
                if not value:
                    continue
                key = key.strip().lower()
                if key == "gsm.apn":
                    matched["apn"] = value
                elif key in ("ip4.address[1]", "ip4.address"):
                    matched["ip4"] = value.split("/")[0]  # "192.0.2.10/24"
                elif key == "general.devices":
                    matched["interface"] = value
                elif key == "general.state":
                    # Formats like "100 (activated)" — extract the word.
                    word = re.search(r"\((\w+)\)", value)
                    matched["state"] = word.group(1) if word else value
        return matched

    def _query_nm_connection_fuzzy(self) -> dict[str, Any] | None:
        """Locate an LTE profile heuristically when no named candidate matched.

        Preference order: an *activated* gsm/cdma/wwan connection; an activated
        connection whose name looks LTE-related (covers CDC-ECM modems that NM
        sees as ethernet); the same allowing inactive profiles, so a
        configured-but-down modem still surfaces in the UI.
        """
        exit_code, stdout = _run(["nmcli", "-t", "-f", "NAME,TYPE,DEVICE,STATE", "connection", "show"])
        if exit_code != 0:
            return None

        cellular_active: list[str] = []
        named_active: list[str] = []
        named_inactive: list[str] = []
        for line in stdout.splitlines():
            parts = line.split(":")
            if len(parts) < 4:
                continue
            name, ctype, _device, state = parts[:4]
            looks_lte = any(h in name.lower() for h in self.LTE_NAME_HINTS)
            if state and ctype in ("gsm", "cdma", "wwan"):
                cellular_active.append(name)
            elif state and looks_lte:
                named_active.append(name)
            elif looks_lte:
                named_inactive.append(name)

        for bucket in (cellular_active, named_active, named_inactive):
            if bucket:
                return self._query_nm_connection(bucket[0])
        return None

    def _query_modemmanager(self) -> dict[str, Any] | None:
        """Return info for the first ModemManager modem reporting usable data."""
        exit_code, stdout = _run(["mmcli", "-L"])
        if exit_code != 0 or "No modems" in stdout:
            return None
        for idx in re.findall(r"/Modem/(\d+)", stdout):
            info = self._read_modem(idx)
            if info:
                return info
        return None

    def _read_modem(self, idx: str) -> dict[str, Any] | None:
        """Pull a single modem's picture out of mmcli."""
        exit_code, stdout = _run(["mmcli", "-m", idx])
        if exit_code != 0:
            return None

        info: dict[str, Any] = {}
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
                if val and val not in ("--", "unknown"):  # mmcli pads unset values with "--"
                    info[dest] = val

        # Signal: prefer --signal-get RSRP; fall back to the generic percent.
        exit_code, sig_out = _run(["mmcli", "-m", idx, "--signal-get"])
        rsrp = None
        if exit_code == 0:
            m = re.search(r"rsrp:\s*([-\d.]+)\s*dBm", sig_out)
            if m:
                rsrp = int(float(m.group(1)))
        if rsrp is None:
            m = re.search(r"signal quality:\s*(\d+)%", stdout, re.IGNORECASE)
            if m:
                rsrp = int(round(int(m.group(1)) / 100.0 * 96 - 140))
        if rsrp is not None:
            info["rsrp_dbm"] = rsrp

        if "interface" not in info:
            exit_code, b_out = _run(["mmcli", "-m", idx, "--bearer", "0"])
            if exit_code == 0:
                m = re.search(r"interface:\s*(\S+)", b_out, re.IGNORECASE)
                if m and m.group(1) not in ("--", "unknown"):
                    info["interface"] = m.group(1).strip()

        return info or None

    def _guess_modem_interface(self) -> tuple[str | None, str | None]:
        """Fallback: first wwan*/usb*/cdc*/ppp* interface with an IPv4."""
        exit_code, stdout = _run(["ip", "-4", "-o", "addr", "show"])
        if exit_code != 0:
            return None, None
        for line in stdout.splitlines():
            parts = line.split()  # "3: wwan0    inet 10.50.10.2/30 ..."
            if len(parts) < 4:
                continue
            iface = parts[1]
            if iface.startswith(("wwan", "usb", "cdc", "ppp")):
                return iface, parts[3].split("/")[0]
        return None, None

    def _ping_host(self, host: str, count: int = 3, timeout: float = 10.0) -> float | None:
        """Ping host and return average latency in ms, or None if unreachable."""
        exit_code, stdout = _run(["ping", "-c", str(count), "-W", "2", host], timeout=timeout)
        if exit_code != 0:
            return None
        # "rtt min/avg/max/mdev = 10.5/15.2/20.1/3.2 ms"
        match = re.search(r"rtt.*=\s*[\d.]+/([\d.]+)/", stdout)
        if match:
            return float(match.group(1))
        times = re.findall(r"time=([\d.]+)\s*ms", stdout)
        if times:
            return sum(float(t) for t in times) / len(times)
        return None


def _run(cmd: list[str], timeout: float = 10.0) -> tuple[int, str]:
    """Run a command, returning (exit_code, stdout)."""
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return result.returncode, result.stdout
    except FileNotFoundError:
        return 127, ""
    except Exception as e:  # noqa: BLE001 - probe failure is a soft error
        logger.debug("Command %s failed: %s", cmd[0], e)
        return 1, ""
