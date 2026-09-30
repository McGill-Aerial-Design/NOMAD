# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Contract tests for the MAVSDK connect/status smoke runner."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from scripts.dev import mavsdk_connectivity_smoke as smoke


@pytest.mark.parametrize("value", ["", "0", "256", "one"])
def test_system_id_rejects_invalid_values(value: str, monkeypatch) -> None:
    monkeypatch.setenv("NOMAD_MAVSDK_SYSTEM_ID", value)
    with pytest.raises(ValueError):
        smoke.get_system_id()


def test_run_smoke_passes_expected_identity_and_accepts_complete_status(monkeypatch, tmp_path: Path) -> None:
    binary = tmp_path / "smoke"
    binary.touch()
    completed = subprocess.CompletedProcess(
        [],
        0,
        "connected=true system=7 samples=3\nposition=45,-73 battery_v=12 gps_fix=3 flight_mode=1\n",
        "",
    )
    calls: list[list[str]] = []

    def fake_run(command: list[str], _timeout: float):
        calls.append(command)
        return completed, 1.25, 4096

    monkeypatch.setattr(smoke, "run_measured_process", fake_run)
    assert smoke.run_smoke(binary, "status", "udpin:0.0.0.0:14570", "7") == 0
    assert calls == [[str(binary), "status", "udpin:0.0.0.0:14570", "7"]]


def test_run_smoke_rejects_incomplete_success_output(monkeypatch, tmp_path: Path) -> None:
    binary = tmp_path / "smoke"
    binary.touch()
    result = subprocess.CompletedProcess([], 0, "connected=true system=1\n", "")
    monkeypatch.setattr(smoke, "run_measured_process", lambda *_args, **_kwargs: (result, 1.0, 2048))
    assert smoke.run_smoke(binary, "status", "udpin:0.0.0.0:14570", "1") == 2


@pytest.mark.parametrize("error", [subprocess.TimeoutExpired("smoke", 20), OSError("loader failed")])
def test_run_smoke_converts_process_failures_to_evidence_error(error: Exception, monkeypatch, tmp_path: Path) -> None:
    binary = tmp_path / "smoke"
    binary.touch()

    def fail(*_args, **_kwargs):
        raise error

    monkeypatch.setattr(smoke, "run_measured_process", fail)
    assert smoke.run_smoke(binary, "connect", "udpin:0.0.0.0:14570", "1") == 2


def test_run_measured_process_reports_current_child_memory() -> None:
    command = [sys.executable, "-c", "import time; value = bytearray(8_000_000); time.sleep(0.1); print(len(value))"]
    result, elapsed_seconds, peak_rss = smoke.run_measured_process(command, 2.0)

    assert result.returncode == 0
    assert result.stdout.strip() == "8000000"
    assert elapsed_seconds >= 0.1
    assert peak_rss is not None and peak_rss >= 8_000_000


def test_runtime_metric_reports_unavailable_memory(capsys) -> None:
    smoke.report_runtime_metric("connect", 1.25, None)
    output = capsys.readouterr().out
    assert "elapsed_seconds=1.250" in output
    assert "peak_process_tree_rss_bytes=unavailable" in output
