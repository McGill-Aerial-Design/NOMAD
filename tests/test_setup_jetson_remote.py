# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Focused contract tests for the remote Jetson provisioning helper."""

from __future__ import annotations

import io
from pathlib import Path

from scripts.setup import setup_jetson_remote as setup


class FakeSSH:
    """Capture remote commands without opening an SSH connection."""

    def __init__(self, output: str = "") -> None:
        self.commands: list[str] = []
        self.output = output

    def exec_command(self, command: str):
        self.commands.append(command)
        return None, io.BytesIO((self.output + "\n").encode()), io.BytesIO()


def test_script_has_no_retired_core_or_http_api_contract() -> None:
    source = (Path(__file__).resolve().parents[1] / "scripts/setup/setup_jetson_remote.py").read_text(encoding="utf-8")

    assert "edge_core" not in source.lower()
    assert "NOMAD_API_" not in source
    assert "8000" not in source
    assert "curl " not in source
    assert "/etc/mavlink-router/main.conf" not in source
    assert "AutoAddPolicy" not in source
    assert "RejectPolicy" in source


def test_configure_nomad_env_loads_canonical_profile_and_disables_optional_workloads(monkeypatch) -> None:
    monkeypatch.setattr(setup, "NOMAD_HOME", "/home/nomad/NOMAD")
    monkeypatch.setattr(setup, "NOMAD_ENV", "/home/nomad/NOMAD/config/nomad.env")
    monkeypatch.setattr(setup, "JETSON_HOME", "/home/nomad")
    monkeypatch.setattr(setup, "GCS_IP", "100.64.0.10")
    ssh = FakeSSH()

    setup.configure_nomad_env(ssh)

    assert ssh.commands[0] == "cd /home/nomad/NOMAD && python3 scripts/profile.py load onboard_companion"
    joined = "\n".join(ssh.commands)
    assert "NOMAD_MAVLINK_ENDPOINT=udpin:0.0.0.0:14550" in joined
    assert "NOMAD_VENV" not in joined
    assert "MAVLINK_UART_DEV=/dev/ttyACM0" in joined
    assert "GCS_IP=100.64.0.10" in joined
    for flag in (
        "NOMAD_AUTOSTART_MEDIAMTX=false",
        "NOMAD_AUTOSTART_ISAAC_ROS_CONTAINER=false",
        "NOMAD_AUTOSTART_ROS_VEHICLE=false",
        "NOMAD_AUTOSTART_VIDEO_BRIDGE=false",
    ):
        assert flag in joined


def test_install_mavlink_router_uses_daemon_name_and_installs_only_when_missing(monkeypatch) -> None:
    calls: list[str] = []

    def fake_run_command(_ssh, command: str, show_output: bool = True):
        calls.append(command)
        if len(calls) == 1:
            return "NOT_INSTALLED", ""
        return "", ""

    monkeypatch.setattr(setup, "run_command", fake_run_command)
    setup.install_mavlink_router(object())

    assert calls == [
        "command -v mavlink-routerd 2>/dev/null || echo 'NOT_INSTALLED'",
        "sudo apt-get update && sudo apt-get install -y mavlink-router",
        "sudo systemctl disable --now mavlink-router.service 2>/dev/null || true",
    ]


def test_install_mavlink_router_disables_distribution_service_when_present(monkeypatch) -> None:
    calls: list[str] = []

    def fake_run_command(_ssh, command: str, show_output: bool = True):
        calls.append(command)
        return "/usr/bin/mavlink-routerd", ""

    monkeypatch.setattr(setup, "run_command", fake_run_command)
    setup.install_mavlink_router(object())

    assert calls == [
        "command -v mavlink-routerd 2>/dev/null || echo 'NOT_INSTALLED'",
        "sudo systemctl disable --now mavlink-router.service 2>/dev/null || true",
    ]


def test_validation_and_build_use_profile_manager_and_cpp_toolchain(monkeypatch) -> None:
    calls: list[str] = []
    monkeypatch.setattr(setup, "NOMAD_HOME", "/home/nomad/NOMAD")
    monkeypatch.setattr(setup, "NOMAD_ENV", "/home/nomad/NOMAD/config/nomad.env")
    monkeypatch.setattr(setup, "run_command", lambda _ssh, command, **_: calls.append(command))

    setup.validate_profile_and_optional_workloads(object())
    setup.build_cpp_core(object())

    assert calls[0] == "cd /home/nomad/NOMAD && python3 scripts/profile.py validate"
    assert "Unavailable: $flag is enabled" in calls[1]
    assert "cmake -S . -B build/core -DCMAKE_BUILD_TYPE=Release" in calls[2]
    assert "cmake --build build/core --parallel" in calls[2]


def test_services_follow_systemd_inventory_and_start_only_current_target(monkeypatch) -> None:
    calls: list[str] = []
    monkeypatch.setattr(setup, "NOMAD_HOME", "/home/nomad/NOMAD")
    monkeypatch.setattr(setup, "run_command", lambda _ssh, command, **_: calls.append(command))

    setup.install_nomad_services(object())

    assert calls == [
        "cd /home/nomad/NOMAD && sudo bash infra/systemd/install.sh",
        "sudo systemctl start nomad.target",
        "sudo systemctl status nomad-mavlink-router --no-pager",
    ]


def test_firewall_exposes_retained_ports_without_deleted_http_api(monkeypatch) -> None:
    calls: list[str] = []
    monkeypatch.setattr(setup, "run_command", lambda _ssh, command, **_: calls.append(command))

    setup.configure_firewall(object())

    assert any("port 22" in command for command in calls)
    assert any("port 8554" in command for command in calls)
    assert any("port 14560" in command for command in calls)
    assert not any("8000" in command for command in calls)


def test_summary_marks_unqualified_optional_workloads_unavailable(capsys) -> None:
    setup.print_summary()

    output = capsys.readouterr().out
    assert "C++ ROS adapter:     unavailable" in output
    assert "MediaMTX/video:      unavailable" in output
    assert "qualified estimator/provider" in output
