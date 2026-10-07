#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Provision a Jetson remotely for the NOMAD C++ companion baseline.

The script installs the host MAVLink router, loads the canonical companion
profile, and installs the current per-service systemd units. Optional ROS,
camera, estimator, and video workloads remain disabled until qualified.
"""

import os
import shlex
import sys

try:
    import paramiko
except ModuleNotFoundError:  # pragma: no cover - exercised only on setup hosts
    paramiko = None

REPOSITORY_URL = "https://github.com/McGill-Aerial-Design/NOMAD.git"
PROFILE_NAME = "onboard_companion"
OPTIONAL_AUTOSTART_FLAGS = (
    "NOMAD_AUTOSTART_MEDIAMTX",
    "NOMAD_AUTOSTART_ISAAC_ROS_CONTAINER",
    "NOMAD_AUTOSTART_ROS_VEHICLE",
    "NOMAD_AUTOSTART_VIDEO_BRIDGE",
)

# NOTE: Password should be set via an environment variable for security.
JETSON_IP = os.environ.get("JETSON_IP", "")
JETSON_USER = os.environ.get("JETSON_SSH_USER", "nomad")
JETSON_PASS = os.environ.get("JETSON_SSH_PASS", "")
JETSON_HOME = f"/home/{JETSON_USER}"
NOMAD_HOME = f"{JETSON_HOME}/NOMAD"
NOMAD_ENV = f"{NOMAD_HOME}/config/nomad.env"
GCS_IP = os.environ.get("GCS_IP", "")


def run_command(ssh, cmd: str, show_output: bool = True) -> tuple[str, str]:
    """Execute a remote command and return stdout and stderr."""
    print(f">>> {cmd}")
    _, stdout, stderr = ssh.exec_command(cmd)
    out = stdout.read().decode().strip()
    err = stderr.read().decode().strip()
    channel = getattr(stdout, "channel", None)
    exit_status = channel.recv_exit_status() if channel is not None else 0
    if show_output and out:
        print(out)
    if err:
        print(f"STDERR: {err}")
    if exit_status:
        raise RuntimeError(f"Remote command failed with status {exit_status}: {cmd}")
    return out, err


def sed_escape(value: str) -> str:
    """Escape a value used in a sed replacement expression."""
    return value.replace("\\", "\\\\").replace("&", "\\&").replace("|", "\\|")


def set_nomad_env_value(ssh, key: str, value: str) -> None:
    """Set one value in the remote runtime environment file."""
    env_file = shlex.quote(NOMAD_ENV)
    replacement = f"{key}={sed_escape(value)}"
    sed_expression = shlex.quote(f"s|^{key}=.*|{replacement}|")
    append_value = shlex.quote(replacement)
    cmd = (
        f"if grep -q '^{key}=' {env_file}; then "
        f"sed -i {sed_expression} {env_file}; "
        f"else printf '%s\\n' {append_value} >> {env_file}; fi"
    )
    run_command(ssh, cmd, show_output=False)


def configure_nomad_env(ssh) -> None:
    """Load the companion profile and apply device-specific transport values."""
    env_file = shlex.quote(NOMAD_ENV)
    repo_root = shlex.quote(NOMAD_HOME)
    run_command(
        ssh,
        f"cd {repo_root} && python3 scripts/profile.py load {PROFILE_NAME}",
    )

    values = {
        "NOMAD_REPO_ROOT": NOMAD_HOME,
        "NOMAD_LOG_DIR": f"{JETSON_HOME}/nomad_logs",
        "NOMAD_RUN_DIR": "/run/nomad",
        "NOMAD_DATA_DIR": f"{NOMAD_HOME}/data",
        "NOMAD_MISSION_LOG_DIR": f"{NOMAD_HOME}/data/mission_logs",
        "NOMAD_MAVLINK_ENDPOINT": "udpin:0.0.0.0:14550",
        "MEDIAMTX_CONFIG": f"{NOMAD_HOME}/infra/mediamtx.yml",
        "MEDIAMTX_BIN": f"{JETSON_HOME}/bin/mediamtx",
        "ISAAC_WORKSPACE": f"{JETSON_HOME}/workspaces/isaac_ros-dev",
        "GCS_IP": GCS_IP,
        "GCS_PORT_LTE": "14560",
        "GCS_PORT_LOCAL": "14550",
        "MAVLINK_UART_DEV": "/dev/ttyACM0",
        "MAVLINK_UART_BAUD": "921600",
        "NOMAD_AUTOSTART_MAVLINK_ROUTER": "true",
        "NOMAD_AUTOSTART_MEDIAMTX": "false",
        "NOMAD_AUTOSTART_ISAAC_ROS_CONTAINER": "false",
        "NOMAD_AUTOSTART_ROS_VEHICLE": "false",
        "NOMAD_AUTOSTART_VIDEO_BRIDGE": "false",
    }

    for key, value in values.items():
        set_nomad_env_value(ssh, key, value)

    print(f"Runtime configuration updated at {env_file} from {PROFILE_NAME}")


def validate_profile_and_optional_workloads(ssh) -> None:
    """Verify the active profile and fail closed for unqualified workloads."""
    repo_root = shlex.quote(NOMAD_HOME)
    env_file = shlex.quote(NOMAD_ENV)
    run_command(ssh, f"cd {repo_root} && python3 scripts/profile.py validate")

    flags = " ".join(OPTIONAL_AUTOSTART_FLAGS)
    command = (
        f"cd {repo_root} && for flag in {flags}; do "
        f'if ! grep -Eq "^$flag=(false|0)$" {env_file}; then '
        'echo "Unavailable: $flag is enabled before optional-provider qualification." >&2; '
        "exit 1; fi; done"
    )
    run_command(ssh, command)


def build_cpp_core(ssh) -> None:
    """Build the C++ core on the remote Jetson checkout."""
    repo_root = shlex.quote(NOMAD_HOME)
    run_command(
        ssh,
        f"cd {repo_root} && cmake -S . -B build/core -DCMAKE_BUILD_TYPE=Release && cmake --build build/core --parallel",
    )


def install_build_dependencies(ssh) -> None:
    """Install the small host toolchain needed for the C++ core build."""
    run_command(
        ssh,
        "sudo apt-get update && sudo apt-get install -y build-essential cmake ninja-build git",
    )


def install_mavlink_router(ssh) -> None:
    """Install mavlink-router when its daemon is not available."""
    out, _ = run_command(ssh, "command -v mavlink-routerd 2>/dev/null || echo 'NOT_INSTALLED'")
    if "NOT_INSTALLED" in out:
        print("mavlink-routerd not found. Installing mavlink-router...")
        run_command(ssh, "sudo apt-get update && sudo apt-get install -y mavlink-router")
    run_command(ssh, "sudo systemctl disable --now mavlink-router.service 2>/dev/null || true")


def install_nomad_services(ssh) -> None:
    """Install and start the current systemd service set."""
    repo_root = shlex.quote(NOMAD_HOME)
    run_command(ssh, f"cd {repo_root} && sudo bash infra/systemd/install.sh")
    run_command(ssh, "sudo systemctl start nomad.target")
    run_command(ssh, "sudo systemctl status nomad-mavlink-router --no-pager")


def update_checkout(ssh) -> None:
    """Clone or fast-forward the remote NOMAD checkout and its submodules."""
    nomad_home = shlex.quote(NOMAD_HOME)
    out, _ = run_command(ssh, f"test -d {nomad_home}/.git && echo 'FOUND' || echo 'NOT_FOUND'")
    if "NOT_FOUND" in out:
        print("NOMAD checkout not found. Cloning repository...")
        run_command(ssh, f"git clone {shlex.quote(REPOSITORY_URL)} {nomad_home}")
    else:
        print("NOMAD checkout exists. Updating...")
        run_command(ssh, f"cd {nomad_home} && git pull --ff-only")
    run_command(ssh, f"cd {nomad_home} && git submodule update --init --recursive")


def configure_firewall(ssh) -> None:
    """Allow retained administration, video, and MAVLink transport ports."""
    firewall_commands = [
        "sudo ufw allow from 100.0.0.0/8 to any port 22 proto tcp comment 'SSH-Tailscale'",
        "sudo ufw allow from 100.0.0.0/8 to any port 8554 proto tcp comment 'RTSP-Tailscale'",
        "sudo ufw allow from 100.0.0.0/8 to any port 14560 proto udp comment 'MAVLink-LTE-Tailscale'",
        "sudo ufw allow from 192.168.0.0/16 to any port 22 proto tcp comment 'SSH-Local'",
        "sudo ufw allow from 192.168.0.0/16 to any port 8554 proto tcp comment 'RTSP-Local'",
        "sudo ufw allow from 192.168.0.0/16 to any port 14550 proto udp comment 'MAVLink-Local'",
        "sudo ufw --force enable",
    ]
    for cmd in firewall_commands:
        run_command(ssh, cmd, show_output=False)
    print("Firewall configured for SSH, optional RTSP, and MAVLink access")


def print_summary() -> None:
    """Print operator-facing next steps for the provisioned baseline."""
    print("\n" + "=" * 50)
    print("Setup Complete!")
    print("=" * 50)
    print(f"""
Configuration:
  Jetson Tailscale IP: {JETSON_IP}
  Ground Station IP:   {GCS_IP or "auto-discovered by mavlink_router.sh"}
  Product profile:     {PROFILE_NAME}

Current services:
  MAVLink router:      enabled through nomad.target
  C++ ROS adapter:     unavailable (no qualified estimator/provider)
  MediaMTX/video:      unavailable (optional workload disabled)

Useful checks:
  ssh {JETSON_USER}@{JETSON_IP}
  cd {NOMAD_HOME}
  bash scripts/nomad status
  sudo systemctl status nomad.target --no-pager
""")


def validate_connection_settings() -> None:
    """Fail before network activity when required connection inputs are absent."""
    if not JETSON_IP:
        print("ERROR: JETSON_IP environment variable not set!")
        print("Set it before running: export JETSON_IP='<host>'")
        sys.exit(1)

    if not JETSON_PASS:
        print("ERROR: JETSON_SSH_PASS environment variable not set!")
        print("Set it before running: export JETSON_SSH_PASS='your-password'")
        sys.exit(1)

    if paramiko is None:
        print("ERROR: Paramiko is required for remote setup.")
        print("Install Paramiko on the setup host before running this script.")
        sys.exit(1)


def connect_ssh():
    """Connect with the operator's existing known-hosts trust store."""
    print(f"Connecting to {JETSON_USER}@{JETSON_IP}...")
    ssh = paramiko.SSHClient()
    ssh.load_system_host_keys()
    ssh.set_missing_host_key_policy(paramiko.RejectPolicy())

    try:
        ssh.connect(JETSON_IP, username=JETSON_USER, password=JETSON_PASS, timeout=10)
        print("✓ Connected successfully!")
    except Exception as exc:  # noqa: BLE001
        print(f"✗ Connection failed: {exc}")
        sys.exit(1)
    return ssh


def provision_jetson(ssh) -> None:
    """Run the ordered remote provisioning steps."""
    print("\n--- System Information ---")
    run_command(ssh, "hostname")
    run_command(ssh, "uname -a")
    run_command(ssh, "tailscale ip -4")

    print("\n--- Checking NOMAD Setup ---")
    update_checkout(ssh)

    print("\n--- Host Dependencies ---")
    install_build_dependencies(ssh)
    run_command(ssh, "python3 --version")
    install_mavlink_router(ssh)

    print("\n--- Runtime Configuration ---")
    configure_nomad_env(ssh)
    validate_profile_and_optional_workloads(ssh)
    build_cpp_core(ssh)

    print("\n--- Firewall ---")
    configure_firewall(ssh)

    print("\n--- NOMAD Services ---")
    install_nomad_services(ssh)
    print_summary()


def main() -> None:
    print("=" * 50)
    print("NOMAD Jetson Remote Setup")
    print("=" * 50)
    validate_connection_settings()
    ssh = connect_ssh()
    try:
        provision_jetson(ssh)
    finally:
        ssh.close()
        print("Done!")


if __name__ == "__main__":
    main()
