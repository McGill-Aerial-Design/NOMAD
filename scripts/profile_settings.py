# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Reviewed ownership of product-profile and deployment-local environment keys."""

PROFILE_KEYS = frozenset(
    {
        "NOMAD_PROFILE",
        "NOMAD_PROFILE_DESCRIPTION",
        "NOMAD_COMPUTE_PLACEMENT",
        "NOMAD_HAS_COMPANION",
        "NOMAD_HAS_PERCEPTION",
        "NOMAD_AUTOSTART_MAVLINK_ROUTER",
        "NOMAD_AUTOSTART_MEDIAMTX",
        "NOMAD_AUTOSTART_ISAAC_ROS_CONTAINER",
        "NOMAD_AUTOSTART_ROS_VEHICLE",
        "NOMAD_AUTOSTART_VIDEO_BRIDGE",
        "NOMAD_MAVLINK_ENDPOINT",
        "NOMAD_VIDEO_RTSP_URL",
        "NOMAD_INTEGRATED_FLIGHT",
        "NOMAD_SIM_MODE",
    }
)

DEPLOYMENT_KEYS = frozenset(
    {
        "NOMAD_REPO_ROOT",
        "NOMAD_LOG_DIR",
        "NOMAD_RUN_DIR",
        "NOMAD_DATA_DIR",
        "NOMAD_MISSION_LOG_DIR",
        "NOMAD_VENV",
        "GCS_IP",
        "GCS_EXTRA_IPS",
        "GCS_PORT_LTE",
        "GCS_PORT_LOCAL",
        "NOMAD_API_KEY",
        "NOMAD_CLIENT_CREDENTIALS_FILE",
        "NOMAD_AUDIT_DIRECTORY",
        "NOMAD_ACTUATORS_FILE",
        "NOMAD_CLIENT_CREDENTIAL",
        "NOMAD_CLIENT_ID",
        "NOMAD_RUNTIME_IPC_PORT",
        "MAVLINK_UART_DEV",
        "MAVLINK_UART_BAUD",
        "MAVLINK_ROUTER_EMPTY_CONF",
        "MAVLINK_ROUTER_LOG_FILE",
        "MAVLINK_ROUTER_PID_FILE",
        "MAVLINK_ROUTER_START_RETRY_DELAY",
        "RTSP_PORT",
        "MEDIAMTX_CONFIG",
        "MEDIAMTX_BIN",
        "ISAAC_CONTAINER_NAME",
        "ISAAC_IMAGE_NAME",
        "ISAAC_IMAGE_FALLBACK",
        "ISAAC_WORKSPACE",
        "ISAAC_ROS_DOMAIN_ID",
        "NOMAD_ROS_OBSERVATION_PORT",
        "NOMAD_ROS_EXPECTED_SYSTEM_ID",
        "NOMAD_ROS_PUBLISH_RATE_HZ",
        "NOMAD_ROS_ROOT",
        "NOMAD_ISAAC_WORKSPACE",
        "VIDEO_BRIDGE_STREAM_PATH",
        "VIDEO_BRIDGE_WIDTH",
        "VIDEO_BRIDGE_HEIGHT",
        "VIDEO_BRIDGE_FPS",
        "VIDEO_BRIDGE_BITRATE",
        "VIDEO_RELAY_HTTP_HOST",
        "VIDEO_RELAY_HTTP_PORT",
        "PYTHONUNBUFFERED",
        "VIDEO_BRIDGE_SOURCE_TOPIC",
        "NOMAD_VIDEO_RTSP_PUBLISH_URL",
        "NOMAD_VIDEO_FLIP_METHOD",
    }
)


def get_assignment_key(line: str) -> str:
    stripped = line.strip()
    if stripped.startswith("#") or "=" not in stripped:
        return ""
    return stripped.partition("=")[0].strip()


def prepare_environment(content: str, current: str, defaults: str, endpoint: str) -> bytes:
    """Apply owned settings and retain reviewed local assignments verbatim."""
    local_lines = {}
    for text in (defaults, current):
        for line in text.splitlines():
            key = get_assignment_key(line)
            if key in DEPLOYMENT_KEYS:
                local_lines[key] = line
    result = []
    for line in content.splitlines():
        key = get_assignment_key(line)
        if key and key not in PROFILE_KEYS:
            continue
        if key == "NOMAD_MAVLINK_ENDPOINT":
            line = f"NOMAD_MAVLINK_ENDPOINT={endpoint}"
        result.append(line)
    result.extend(["", "# Deployment-local settings (never saved into product profiles)."])
    result.extend(local_lines[key] for key in sorted(local_lines))
    return ("\n".join(result) + "\n").encode("utf-8")
