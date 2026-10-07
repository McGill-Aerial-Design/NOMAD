# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Structural checks for infrastructure wiring retained after Edge Core removal."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_tailscale_setup_exposes_only_retained_service_ports() -> None:
    setup = read("infra/tailscale/scripts/setup.sh")

    assert "Edge Core" not in setup
    assert "8000" not in setup
    assert "/health" not in setup
    assert "ufw allow in on tailscale0" not in setup
    assert "ufw allow from 100.0.0.0/8 to any port 22 proto tcp" in setup
    assert "ufw allow from 100.0.0.0/8 to any port 8554 proto tcp" in setup
    assert "ufw allow from 100.0.0.0/8 to any port 14560 proto udp" in setup


def test_tailscale_next_steps_follow_standalone_router_ownership() -> None:
    setup = read("infra/tailscale/scripts/setup.sh")
    assert "GCS_IP to the ground station's Tailscale IP in ignored config/nomad.env" in setup
    assert "retained when loading a product profile" in setup
    assert "infra/transport/ground_router/example.json" in setup
    assert "Links and Consumers" in setup
    assert "nomad-runtime owns commands" in setup
    assert "receive-only router telemetry" in setup
    assert "docs/operations.md" in setup
    assert "NOMAD plugin LTE input" not in setup
    assert "NOMAD plugin RadioMaster input" not in setup


def test_mavlink_router_attributes_local_core_to_cpp_owner() -> None:
    config = read("infra/transport/mavlink_router/main.conf")

    assert "Edge Core" not in config
    assert "The NOMAD C++ core listens on this port" in config
    assert "Do not install this file as a service configuration" in config
    assert "systemctl enable mavlink-router" not in config
    assert "[UdpEndpoint orchestrator]" in config
    assert "Address=127.0.0.1" in config
    assert "Port=14550" in config
