# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for the shell probe wrapper shared by the network and Tailscale monitors.

Both monitors run probes from a background daemon thread that must survive a
missing binary or a hung command, so every failure has to come back as an exit
code rather than an exception.
"""

from __future__ import annotations

from types import SimpleNamespace

from infra.tailscale import shell


def test_run_command_returns_exit_code_and_stdout(monkeypatch):
    monkeypatch.setattr(shell.subprocess, "run", lambda *args, **kwargs: SimpleNamespace(returncode=0, stdout="hi"))

    assert shell.run_command(["echo", "hi"]) == (0, "hi")


def test_run_command_missing_binary_returns_127(monkeypatch):
    def missing_binary(*_args, **_kwargs):
        raise FileNotFoundError("nmcli")

    monkeypatch.setattr(shell.subprocess, "run", missing_binary)

    assert shell.run_command(["nmcli"]) == (127, "")


def test_run_command_failure_returns_1(monkeypatch):
    def failed_probe(*_args, **_kwargs):
        raise OSError("kaboom")

    monkeypatch.setattr(shell.subprocess, "run", failed_probe)

    assert shell.run_command(["ping"]) == (1, "")


def test_run_command_passes_timeout_and_command(monkeypatch):
    seen: dict = {}

    def record(cmd, **kwargs):
        seen["cmd"] = cmd
        seen.update(kwargs)
        return SimpleNamespace(returncode=0, stdout="")

    monkeypatch.setattr(shell.subprocess, "run", record)

    shell.run_command(["tailscale", "up"], timeout=30.0)

    assert seen["cmd"] == ["tailscale", "up"]
    assert seen["timeout"] == 30.0
