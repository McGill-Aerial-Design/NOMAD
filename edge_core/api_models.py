# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Shared request/response models and command contracts for Edge Core API routes."""

from pydantic import BaseModel, field_validator

# Whitelist of allowed terminal commands.
#
# Every NOMAD service is now a per-service systemd unit (see
# infra/systemd/nomad-*.service) wrapping a script under scripts/services/.
# These whitelist entries delegate to `systemctl` or the `nomad` CLI so that
# the Mission Planner terminal panel cannot create out-of-band processes that
# fight the systemd-managed lifecycle.
COMMAND_WHITELIST: dict[str, str] = {
    # --- Service status ---
    "status_nomad": "systemctl is-active nomad-edge-core.service",
    "status_mediamtx": "systemctl is-active nomad-mediamtx.service",
    "status_mavlink": "systemctl is-active nomad-mavlink-router.service",
    "status_video": "systemctl is-active nomad-video-bridge.service",
    "status_isaac": "systemctl is-active nomad-isaac-ros-container.service",
    "status_zed": "systemctl is-active nomad-zed-wrapper.service",
    "status_ros_bridge": "systemctl is-active nomad-ros-http-bridge.service",
    "status_nvblox": "systemctl is-active nomad-nvblox.service",
    "status_novnc": "if systemctl --user is-active --quiet novnc 2>/dev/null || systemctl is-active --quiet novnc 2>/dev/null || (pgrep -f '[w]ebsockify.*6080' >/dev/null && pgrep -f '[x]11vnc.*-rfbport 5900' >/dev/null); then echo active; else echo inactive; fi",
    # --- Per-service start / stop / restart ---
    "start_nomad": "sudo -n systemctl start   nomad-edge-core.service 2>&1 && echo started || echo failed",
    "stop_nomad": "nohup bash -c 'sleep 2 && sudo -n systemctl stop nomad-edge-core.service' > /dev/null 2>&1 & echo 'stop scheduled'",
    "restart_edge_core": "nohup bash -c 'sleep 2 && sudo -n systemctl restart nomad-edge-core.service' > /dev/null 2>&1 & echo 'restart scheduled'",
    "start_mediamtx": "sudo -n systemctl start   nomad-mediamtx.service 2>&1 && echo started || echo failed",
    "stop_mediamtx": "sudo -n systemctl stop    nomad-mediamtx.service 2>&1 && echo stopped || echo failed",
    "restart_video": "sudo -n systemctl restart nomad-mediamtx.service && sudo -n systemctl restart nomad-video-bridge.service 2>&1 && echo restarted || echo failed",
    "start_mavlink": "sudo -n systemctl start   nomad-mavlink-router.service 2>&1 && echo started || echo failed",
    "stop_mavlink": "sudo -n systemctl stop    nomad-mavlink-router.service 2>&1 && echo stopped || echo failed",
    "restart_mavlink": "sudo -n systemctl restart nomad-mavlink-router.service 2>&1 && echo restarted || echo failed",
    "start_video_bridge": "sudo -n systemctl start   nomad-video-bridge.service 2>&1 && echo started || echo failed",
    "stop_video_bridge": "sudo -n systemctl stop    nomad-video-bridge.service 2>&1 && echo stopped || echo failed",
    "restart_video_bridge": "sudo -n systemctl restart nomad-video-bridge.service 2>&1 && echo restarted || echo failed",
    "start_isaac": "sudo -n systemctl start   nomad-isaac-ros-container.service nomad-zed-wrapper.service nomad-ros-http-bridge.service 2>&1 && echo started || echo failed",
    "stop_isaac": "sudo -n systemctl stop    nomad-isaac-ros-container.service nomad-zed-wrapper.service nomad-ros-http-bridge.service 2>&1 && echo stopped || echo failed",
    "restart_isaac": "nohup bash -c 'sleep 2 && sudo -n systemctl restart nomad-isaac-ros-container.service nomad-zed-wrapper.service nomad-ros-http-bridge.service' > /dev/null 2>&1 & echo 'restart scheduled'",
    # --- Compound operations ---
    "restart_all": "nohup bash -c 'sleep 2 && sudo -n systemctl restart nomad-edge-core.service nomad-mediamtx.service nomad-mavlink-router.service nomad-zed-wrapper.service nomad-ros-http-bridge.service' > /dev/null 2>&1 & echo 'restart scheduled'",
}

# ==================== Request/Response Models ====================


class TerminalCommandRequest(BaseModel):
    """Request model for terminal command execution."""

    command_name: str
    timeout: int = 10

    @field_validator("timeout")
    @classmethod
    def _clamp_terminal_run_timeout(cls, value: int) -> int:
        return max(1, min(int(value), 60))


class TerminalExecRequest(BaseModel):
    """Request model for arbitrary terminal command execution."""

    command: str
    timeout: int = 30
    cwd: str | None = None

    @field_validator("timeout")
    @classmethod
    def _clamp_terminal_exec_timeout(cls, value: int) -> int:
        return max(1, min(int(value), 120))


class TerminalCommandResponse(BaseModel):
    """Response model for terminal command."""

    success: bool
    stdout: str
    stderr: str
    return_code: int
    command_executed: str | None = None
    cwd: str | None = None
