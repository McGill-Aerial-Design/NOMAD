#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Jetson Remote Setup Script
Connects via SSH and configures the Jetson Orin Nano
"""

import os
import shlex
import sys

import paramiko

# Jetson Configuration
# NOTE: Password should be set via environment variable for security
JETSON_IP = os.environ.get("JETSON_IP", "")
JETSON_USER = os.environ.get("JETSON_SSH_USER", "nomad")
JETSON_PASS = os.environ.get("JETSON_SSH_PASS", "")
JETSON_HOME = f"/home/{JETSON_USER}"  # User home directory on Jetson
NOMAD_HOME = f"{JETSON_HOME}/NOMAD"  # NOMAD repository path
NOMAD_ENV = f"{NOMAD_HOME}/config/nomad.env"

GCS_IP = os.environ.get("GCS_IP", "")


def run_command(ssh, cmd, show_output=True):
    """Execute command and return output"""
    print(f">>> {cmd}")
    stdin, stdout, stderr = ssh.exec_command(cmd)
    out = stdout.read().decode().strip()
    err = stderr.read().decode().strip()
    if show_output and out:
        print(out)
    if err:
        print(f"STDERR: {err}")
    return out, err


def sed_escape(value: str) -> str:
    return value.replace("\\", "\\\\").replace("&", "\\&").replace("|", "\\|")


def set_nomad_env_value(ssh, key: str, value: str) -> None:
    env_file = shlex.quote(NOMAD_ENV)
    replacement = f"{key}={sed_escape(value)}"
    cmd = (
        f"if grep -q '^{key}=' {env_file}; then "
        f"sed -i 's|^{key}=.*|{replacement}|' {env_file}; "
        f"else printf '\\n{replacement}\\n' >> {env_file}; fi"
    )
    run_command(ssh, cmd, show_output=False)


def configure_nomad_env(ssh) -> None:
    env_file = shlex.quote(NOMAD_ENV)
    repo_root = shlex.quote(NOMAD_HOME)
    run_command(
        ssh,
        f"cd {repo_root} && mkdir -p config && ([ -f {env_file} ] || cp config/nomad.env.example {env_file})",
    )

    values = {
        "NOMAD_REPO_ROOT": NOMAD_HOME,
        "NOMAD_LOG_DIR": f"{JETSON_HOME}/nomad_logs",
        "NOMAD_DATA_DIR": f"{NOMAD_HOME}/data",
        "NOMAD_MISSION_LOG_DIR": f"{NOMAD_HOME}/data/mission_logs",
        "NOMAD_VENV": f"{NOMAD_HOME}/venv",
        "NOMAD_API_HOST": "0.0.0.0",
        "NOMAD_API_PORT": "8000",
        "NOMAD_API_URL": "http://localhost:8000",
        "GCS_IP": GCS_IP,
        "GCS_PORT_LTE": "14560",
        "GCS_PORT_LOCAL": "14550",
        "MAVLINK_UART_DEV": "/dev/ttyTHS1",
        "MAVLINK_UART_BAUD": "921600",
        "NOMAD_AUTOSTART_ISAAC_ROS_CONTAINER": "false",
        "NOMAD_ENABLE_NVBLOX_MESH": "false",
    }

    for key, value in values.items():
        set_nomad_env_value(ssh, key, value)

    print(f"Runtime configuration updated at {NOMAD_ENV}")


def main():
    print("=" * 50)
    print("NOMAD Jetson Remote Setup")
    print("=" * 50)

    # Validate connection settings.
    if not JETSON_IP:
        print("ERROR: JETSON_IP environment variable not set!")
        print("Set it before running: export JETSON_IP='<host>'")
        sys.exit(1)

    if not JETSON_PASS:
        print("ERROR: JETSON_SSH_PASS environment variable not set!")
        print("Set it before running: export JETSON_SSH_PASS='your-password'")
        sys.exit(1)

    print(f"Connecting to {JETSON_USER}@{JETSON_IP}...")

    # Connect
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        ssh.connect(JETSON_IP, username=JETSON_USER, password=JETSON_PASS, timeout=10)
        print("✓ Connected successfully!")
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        sys.exit(1)

    # Get system info
    print("\n--- System Information ---")
    run_command(ssh, "hostname")
    run_command(ssh, "uname -a")
    run_command(ssh, "tailscale ip -4")

    # Check if NOMAD directory exists
    print("\n--- Checking NOMAD Setup ---")
    out, _ = run_command(ssh, f"ls -la {NOMAD_HOME} 2>/dev/null || echo 'NOT_FOUND'")

    if "NOT_FOUND" in out:
        print("NOMAD not found. Cloning repository...")
        run_command(ssh, f"git clone https://github.com/McGill-Aerial-Design/NOMAD.git {NOMAD_HOME}")
    else:
        print("NOMAD directory exists. Updating...")
        run_command(ssh, f"cd {NOMAD_HOME} && git pull")

    # Check Python
    print("\n--- Python Environment ---")
    run_command(ssh, "python3 --version")

    # Check MAVLink Router
    print("\n--- MAVLink Router Status ---")
    out, _ = run_command(ssh, "which mavlink-router 2>/dev/null || echo 'NOT_INSTALLED'")
    if "NOT_INSTALLED" in out:
        print("Installing mavlink-router...")
        run_command(ssh, "sudo apt-get update && sudo apt-get install -y mavlink-router")

    # Configure MAVLink Router to send to Ground Station
    print("\n--- Configuring MAVLink Router ---")
    mavlink_conf = f"""[General]
TcpServerPort=5760
ReportStats=true
MavlinkDialect=ardupilotmega

[UartEndpoint alpha]
Device=/dev/ttyTHS1
Baud=921600

[UdpEndpoint orchestrator]
Mode=Normal
Address=127.0.0.1
Port=14550

[UdpEndpoint groundstation]
Mode=Normal
Address={GCS_IP}
Port=14560
"""

    # Write config
    run_command(ssh, f"echo '{mavlink_conf}' | sudo tee /etc/mavlink-router/main.conf > /dev/null")
    print(f"MAVLink Router configured to send LTE telemetry to {GCS_IP}:14560")

    # Enable MAVLink Router service
    run_command(ssh, "sudo systemctl enable mavlink-router")
    run_command(ssh, "sudo systemctl restart mavlink-router")

    # Check Tailscale status
    print("\n--- Tailscale Status ---")
    run_command(ssh, "tailscale status")

    # Configure firewall
    print("\n--- Firewall Configuration ---")
    firewall_cmds = [
        "sudo ufw allow from 100.0.0.0/8 to any port 22 proto tcp comment 'SSH-Tailscale'",
        "sudo ufw allow from 100.0.0.0/8 to any port 8000 proto tcp comment 'API-Tailscale'",
        "sudo ufw allow from 100.0.0.0/8 to any port 8554 proto tcp comment 'RTSP-Tailscale'",
        "sudo ufw allow from 100.0.0.0/8 to any port 14560 proto udp comment 'MAVLink-LTE-Tailscale'",
        "sudo ufw --force enable",
    ]
    for cmd in firewall_cmds:
        run_command(ssh, cmd, show_output=False)
    print("Firewall configured for Tailscale access")

    print("\n--- Configuring Runtime Environment ---")
    configure_nomad_env(ssh)

    # Test Edge Core
    print("\n--- Testing Edge Core ---")
    out, _ = run_command(
        ssh, f"cd {NOMAD_HOME} && python3 -c 'from edge_core import main; print(\"Edge Core import OK\")' 2>&1"
    )

    # Summary
    print("\n" + "=" * 50)
    print("Setup Complete!")
    print("=" * 50)
    print(f"""
Configuration:
  Jetson Tailscale IP: {JETSON_IP}
  Ground Station IP:   {GCS_IP}

To start Edge Core manually:
  ssh {JETSON_USER}@{JETSON_IP}
  cd {NOMAD_HOME}
  python3 -m edge_core.main

To test from Windows:
  curl http://{JETSON_IP}:8000/health

Mission Planner:
  Main connection: UDP port 14600 (NOMAD plugin merged router)
  Plugin LTE input: UDP port 14560
  Plugin RadioMaster input: UDP port 14550
""")

    ssh.close()
    print("Done!")


if __name__ == "__main__":
    main()
