"""Shared request/response models and command contracts for Edge Core API routes."""

from typing import Optional

from pydantic import BaseModel

# msgpack for efficient mesh deserialization (optional)
try:
    import msgpack  # noqa: F401

    MSGPACK_AVAILABLE = True
except ImportError:
    MSGPACK_AVAILABLE = False

# ==================== Request/Response Models ====================


class Task2HitRequest(BaseModel):
    """Request model for Task 2 target hit."""

    x: float
    y: float
    z: float


class Task1CapturesList(BaseModel):
    """Response model for list of Task 1 captures."""

    captures: list[str]
    count: int


# Whitelist of allowed terminal commands.
#
# Every NOMAD service is now a per-service systemd unit (see
# infra/systemd/nomad-*.service) wrapping a script under scripts/services/.
# These whitelist entries delegate to `systemctl` or the `nomad` CLI so that
# the Mission Planner terminal panel cannot create out-of-band processes that
# fight the systemd-managed lifecycle.
#
# Service<->unit map:
#   edge_core      -> nomad-edge-core.service
#   mediamtx       -> nomad-mediamtx.service
#   mavlink_router -> nomad-mavlink-router.service
#   video_bridge   -> nomad-video-bridge.service
#   isaac_ros      -> nomad-isaac-ros-container.service (+ zed_wrapper, ros_http_bridge)
#   nvblox         -> nomad-nvblox.service (opt-in)
#
# `sudo -n systemctl ...` requires a sudoers entry granting the `mad` user
# passwordless control of the nomad-*.service units; the setup script installs
# that. Where sudo isn't available we fall back to the `nomad` CLI which uses
# script-level process management.
COMMAND_WHITELIST: dict[str, str] = {
    # --- Service status ---
    "status_nomad":      "systemctl is-active nomad-edge-core.service",
    "status_mediamtx":   "systemctl is-active nomad-mediamtx.service",
    "status_mavlink":    "systemctl is-active nomad-mavlink-router.service",
    "status_video":      "systemctl is-active nomad-video-bridge.service",
    "status_isaac":      "systemctl is-active nomad-isaac-ros-container.service",
    "status_zed":        "systemctl is-active nomad-zed-wrapper.service",
    "status_ros_bridge": "systemctl is-active nomad-ros-http-bridge.service",
    "status_nvblox":     "systemctl is-active nomad-nvblox.service",
    "status_novnc": "if systemctl --user is-active --quiet novnc 2>/dev/null || systemctl is-active --quiet novnc 2>/dev/null || (pgrep -f '[w]ebsockify.*6080' >/dev/null && pgrep -f '[x]11vnc.*-rfbport 5900' >/dev/null); then echo active; else echo inactive; fi",

    # --- Per-service start / stop / restart (delegate to systemd) ---
    "start_nomad":         "sudo -n systemctl start   nomad-edge-core.service 2>&1 && echo started || echo failed",
    "stop_nomad":          "nohup bash -c 'sleep 2 && sudo -n systemctl stop nomad-edge-core.service' > /dev/null 2>&1 & echo 'stop scheduled'",
    "restart_edge_core":   "nohup bash -c 'sleep 2 && sudo -n systemctl restart nomad-edge-core.service' > /dev/null 2>&1 & echo 'restart scheduled'",
    "start_mediamtx":      "sudo -n systemctl start   nomad-mediamtx.service 2>&1 && echo started || echo failed",
    "stop_mediamtx":       "sudo -n systemctl stop    nomad-mediamtx.service 2>&1 && echo stopped || echo failed",
    "restart_video":       "sudo -n systemctl restart nomad-mediamtx.service && sudo -n systemctl restart nomad-video-bridge.service 2>&1 && echo restarted || echo failed",
    "start_mavlink":       "sudo -n systemctl start   nomad-mavlink-router.service 2>&1 && echo started || echo failed",
    "stop_mavlink":        "sudo -n systemctl stop    nomad-mavlink-router.service 2>&1 && echo stopped || echo failed",
    "restart_mavlink":     "sudo -n systemctl restart nomad-mavlink-router.service 2>&1 && echo restarted || echo failed",
    "start_video_bridge":  "sudo -n systemctl start   nomad-video-bridge.service 2>&1 && echo started || echo failed",
    "stop_video_bridge":   "sudo -n systemctl stop    nomad-video-bridge.service 2>&1 && echo stopped || echo failed",
    "restart_video_bridge":"sudo -n systemctl restart nomad-video-bridge.service 2>&1 && echo restarted || echo failed",
    "start_isaac":         "sudo -n systemctl start   nomad-isaac-ros-container.service nomad-zed-wrapper.service nomad-ros-http-bridge.service 2>&1 && echo started || echo failed",
    "stop_isaac":          "sudo -n systemctl stop    nomad-ros-http-bridge.service nomad-zed-wrapper.service nomad-isaac-ros-container.service 2>&1 && echo stopped || echo failed",
    "restart_isaac":       "sudo -n systemctl restart nomad-isaac-ros-container.service nomad-zed-wrapper.service nomad-ros-http-bridge.service 2>&1 && echo restarted || echo failed",
    "start_nvblox":        "sudo -n systemctl start   nomad-nvblox.service 2>&1 && echo started || echo failed",
    "stop_nvblox":         "sudo -n systemctl stop    nomad-nvblox.service 2>&1 && echo stopped || echo failed",
    "restart_nvblox":      "sudo -n systemctl restart nomad-nvblox.service 2>&1 && echo restarted || echo failed",

    # --- Individual ZED + ROS bridge control (per-unit) ---
    "start_zed":           "sudo -n systemctl start   nomad-zed-wrapper.service 2>&1 && echo started || echo failed",
    "stop_zed":            "sudo -n systemctl stop    nomad-zed-wrapper.service 2>&1 && echo stopped || echo failed",
    "restart_zed":         "sudo -n systemctl restart nomad-zed-wrapper.service 2>&1 && echo restarted || echo failed",
    "start_ros_bridge":    "sudo -n systemctl start   nomad-ros-http-bridge.service 2>&1 && echo started || echo failed",
    "stop_ros_bridge":     "sudo -n systemctl stop    nomad-ros-http-bridge.service 2>&1 && echo stopped || echo failed",
    "restart_ros_bridge":  "sudo -n systemctl restart nomad-ros-http-bridge.service 2>&1 && echo restarted || echo failed",

    # --- Bring everything up / down (autostart set per config/nomad.env) ---
    "start_all":           "sudo -n systemctl start   nomad.target 2>&1 && echo started || echo failed",
    "stop_all":            "sudo -n systemctl stop    nomad.target 2>&1 && echo stopped || echo failed",
    "restart_all":         "nohup bash -c 'sleep 2 && /home/mad/NOMAD/scripts/nomad restart all' > /dev/null 2>&1 & echo 'restart scheduled'",

    # --- noVNC (unchanged: not part of the NOMAD systemd target) ---
    "start_novnc": "if ss -ltn | grep -q ':6080 '; then echo 'already running'; else mkdir -p ~/nomad_logs; pgrep -x Xvfb >/dev/null || (screen=${NOVNC_GEOMETRY:-1920x1080}x24; nohup Xvfb :1 -screen 0 \"$screen\" -ac +extension RANDR > ~/nomad_logs/xvfb.log 2>&1 & sleep 1); pgrep -x openbox >/dev/null || (DISPLAY=:1 nohup openbox-session > ~/nomad_logs/openbox.log 2>&1 & sleep 1); command -v tint2 >/dev/null 2>&1 && (pgrep -x tint2 >/dev/null || (DISPLAY=:1 nohup tint2 > ~/nomad_logs/tint2.log 2>&1 & sleep 1)); if ! ss -ltn | grep -q ':5900 '; then x11vnc -display :1 -rfbport 5900 -localhost -forever -shared -repeat -xkb -noxdamage -noxfixes -noxrecord -bg -passwd ${NOVNC_VNC_PASSWORD:-skibidi123} -o ~/nomad_logs/x11vnc.log >/dev/null 2>&1; fi; for i in 1 2 3 4 5 6 7 8 9 10; do ss -ltn | grep -q ':5900 ' && break; sleep 1; done; if ! ss -ltn | grep -q ':5900 '; then echo failed; exit 1; fi; nohup websockify --heartbeat 30 --web /usr/share/novnc/ 6080 localhost:5900 > ~/nomad_logs/novnc.log 2>&1 & for i in 1 2 3 4 5 6 7 8 9 10; do ss -ltn | grep -q ':6080 ' && break; sleep 1; done; ss -ltn | grep -q ':6080 ' && echo started || (echo failed; exit 1); fi",
    "stop_novnc": "stopped=0; pkill -f '[w]ebsockify.*6080' 2>/dev/null && stopped=1; pkill -f '[x]11vnc.*-rfbport 5900' 2>/dev/null && stopped=1; pkill -x tint2 2>/dev/null && stopped=1; pkill -x openbox 2>/dev/null && stopped=1; pkill -f '[X]vfb :1' 2>/dev/null && stopped=1; [ $stopped -eq 1 ] && echo stopped || echo 'not running'",

    # --- System commands ---
    "reboot_jetson": "nohup bash -c 'sleep 2 && sudo -n reboot' > /dev/null 2>&1 & echo 'reboot scheduled'",
    "shutdown_jetson": "nohup bash -c 'sleep 2 && sudo -n shutdown -h now' > /dev/null 2>&1 & echo 'shutdown scheduled'",
    "check_disk": "df -h",
    "check_memory": "free -h",
    "check_processes": "ps aux | head -20",
    "tailscale_status": "tailscale status",
    "network_info": "ip addr show",
    "gpu_status": "tegrastats --interval 1000 --stop 2",
}


# ==================== Helper Functions ====================


class TerminalCommandRequest(BaseModel):
    """Request model for terminal command execution."""

    command_name: str  # Must be a key in COMMAND_WHITELIST
    timeout: int = 10


class TerminalExecRequest(BaseModel):
    """Request model for arbitrary terminal command execution."""

    command: str
    timeout: int = 30
    cwd: Optional[str] = None  # Working directory (persistent cd support)


class TerminalCommandResponse(BaseModel):
    """Response model for terminal command."""

    success: bool
    stdout: str
    stderr: str
    return_code: int
    command_executed: Optional[str] = None
    cwd: Optional[str] = None  # Current working directory after execution


class VIOUpdateRequest(BaseModel):
    """Request model for VIO pose update from ROS bridge."""

    timestamp: float
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    confidence: float = 1.0
    source: str = "external"
    # Raw ROS-frame pose (odom/map) for SLAM 3D visualization
    ros_x: float = 0.0
    ros_y: float = 0.0
    ros_z: float = 0.0
    # Raw ROS-frame orientation (same frame as mesh vertices)
    ros_roll: float = 0.0
    ros_pitch: float = 0.0
    ros_yaw: float = 0.0
    # Optional body attitude in ROS frame (camera attitude with gimbal pitch removed)
    body_roll: Optional[float] = None
    body_pitch: Optional[float] = None
    body_yaw: Optional[float] = None
    # Coordinate frame for ros_* pose fields: REP-103 odom frame (X-forward, Y-left, Z-up)
    frame_id: str = "ros_odom"


class VIOAreaSaveRequest(BaseModel):
    """Request model for saving a VIO relocalization area map."""

    file_path: str
    wait_for_completion: bool = True
    timeout_s: float = 30.0


class VIOAreaLoadRequest(BaseModel):
    """Request model for loading a VIO relocalization area map."""

    file_path: str


class NavVelocityRequest(BaseModel):
    """Request model for navigation velocity command from ROS nav2/nvblox."""

    timestamp: float
    vx: float  # Forward velocity (m/s)
    vy: float  # Lateral velocity (m/s)
    vz: float  # Vertical velocity (m/s)
    yaw_rate: float  # Yaw rate (rad/s)
    source: str = "nav2"


class NavPositionRequest(BaseModel):
    """Request model for navigation position target."""

    x: float  # North position (NED meters)
    y: float  # East position (NED meters)
    z: float  # Down position (NED meters)
    yaw: float  # Heading (radians)
    source: str = "nav2"


class GDriveUploadRequest(BaseModel):
    """Request model for generic Google Drive file upload."""

    local_path: str
    filename: str
    folder_id: Optional[str] = None


class GDriveUploadResponse(BaseModel):
    """Response model for Google Drive upload."""

    success: bool
    file_id: str = ""
    error: str = ""


