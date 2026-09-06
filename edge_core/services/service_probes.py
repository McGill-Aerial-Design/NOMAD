# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Blocking service-state probes for the ``/api/services/status`` route.

Every function here blocks on ``subprocess.run`` or a filesystem check, so
callers must run it on a worker thread, never on the event loop.
"""

import os
import subprocess


def _systemd_active(unit: str, user: bool = False) -> tuple[bool, str]:
    """Return (is_active, reported_status) for a systemd unit."""
    cmd = ["systemctl", "is-active", unit]
    if user:
        cmd.insert(1, "--user")
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=2)
        status = r.stdout.strip() or "inactive"
        return r.returncode == 0, status
    except Exception:
        return False, "inactive"


def _pgrep_running(pattern: str) -> bool:
    """Return True when a process matching pattern is running."""
    try:
        r = subprocess.run(
            ["pgrep", "-f", pattern],
            capture_output=True,
            text=True,
            timeout=3,
        )
        return r.returncode == 0
    except Exception:
        return False


def probe_mavlink_router() -> dict:
    """Probe the mavlink-router service and the flight-controller device."""
    try:
        systemd_running, systemd_status = _systemd_active("mavlink-router")
        router_running = _pgrep_running("mavlink-routerd")
        fc_present = os.path.exists("/dev/ttyACM0")
        if not fc_present:
            return {
                "status": "no_flight_controller",
                "running": False,
                "flight_controller_present": False,
            }
        running = systemd_running or router_running
        return {
            "status": "active" if running else systemd_status,
            "running": running,
            "flight_controller_present": True,
        }
    except Exception as e:
        return {"status": "error", "error": str(e)}


def probe_mediamtx() -> dict:
    """Probe the mediamtx RTSP server."""
    try:
        systemd_running, systemd_status = _systemd_active("mediamtx")
        running = systemd_running or _pgrep_running("mediamtx")
        return {
            "status": "active" if running else systemd_status,
            "running": running,
        }
    except Exception as e:
        return {"status": "error", "error": str(e)}


def probe_novnc() -> dict:
    """Probe the noVNC desktop service (user unit, system unit, or VNC pair)."""
    try:
        user_running, user_status = _systemd_active("novnc", user=True)
        system_running, system_status = _systemd_active("novnc")
        websockify_running = _pgrep_running("[w]ebsockify.*6080")
        x11vnc_running = _pgrep_running("[x]11vnc.*-rfbport 5900")

        running = user_running or system_running or (websockify_running and x11vnc_running)
        status = "active" if running else (user_status if user_status != "inactive" else system_status)
        return {
            "status": status,
            "running": running,
            "url": "http://localhost:6080/vnc.html",
        }
    except Exception as e:
        return {"status": "error", "error": str(e)}
