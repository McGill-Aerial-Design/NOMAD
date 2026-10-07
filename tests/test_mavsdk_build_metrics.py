# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for MAVSDK connectivity-smoke build metrics."""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

from scripts.dev import mavsdk_build_metrics as metrics


def test_directory_size_counts_regular_files(tmp_path: Path) -> None:
    (tmp_path / "one.bin").write_bytes(b"1234")
    nested = tmp_path / "nested"
    nested.mkdir()
    (nested / "two.bin").write_bytes(b"123456")
    assert metrics.directory_size(tmp_path) == 10


def test_collect_build_metrics_reports_executable_and_static_archives(tmp_path: Path) -> None:
    build_dir = tmp_path / "mavsdk-qualification"
    release = build_dir / "Release"
    release.mkdir(parents=True)
    executable = release / "nomad_mavsdk_connectivity_smoke.exe"
    executable.write_bytes(b"x" * 11)

    install_lib = build_dir / "mavsdk" / "third_party" / "install" / "lib"
    install_lib.mkdir(parents=True)
    (install_lib / "one.lib").write_bytes(b"a" * 7)
    (install_lib / "two.a").write_bytes(b"b" * 5)
    (install_lib / "ignore.txt").write_bytes(b"c" * 3)

    result = metrics.collect_build_metrics(build_dir)
    assert result["smoke_executable_bytes"] == 11
    assert result["selected_static_archives"] == 2
    assert result["selected_static_archives_bytes"] == 12
    assert result["build_tree_bytes"] == 26


def test_collect_build_metrics_fails_without_smoke_binary(tmp_path: Path) -> None:
    build_dir = tmp_path / "mavsdk-qualification"
    build_dir.mkdir()
    try:
        metrics.collect_build_metrics(build_dir)
    except FileNotFoundError as error:
        assert "smoke executable" in str(error)
    else:
        raise AssertionError("missing smoke binary must fail")


def test_run_build_measures_configure_and_target(monkeypatch, tmp_path: Path) -> None:
    commands: list[list[str]] = []
    readings = iter((10.0, 11.25, 20.0, 22.5))

    def run(command: list[str], check: bool) -> None:
        assert check is True
        commands.append(command)

    monkeypatch.setattr(subprocess, "run", run)
    monkeypatch.setattr(metrics.time, "perf_counter", lambda: next(readings))

    result = metrics.run_build(tmp_path)

    assert result == {"configure_seconds": 1.25, "target_build_seconds": 2.5}
    assert commands[0][:3] == ["cmake", "-S", str(metrics.ROOT)]
    assert commands[1][-2:] == ["--target", "nomad_mavsdk_connectivity_smoke"]


def test_run_build_propagates_phase_failure(monkeypatch, tmp_path: Path) -> None:
    failure = subprocess.CalledProcessError(1, ["cmake"])
    monkeypatch.setattr(subprocess, "run", lambda *_args, **_kwargs: (_ for _ in ()).throw(failure))
    try:
        metrics.run_build(tmp_path)
    except subprocess.CalledProcessError as error:
        assert error is failure
    else:
        raise AssertionError("failed build phase must fail evidence collection")


def test_write_github_summary_contains_all_metrics(tmp_path: Path) -> None:
    summary = tmp_path / "summary.md"
    values = {
        "build_tree_bytes": 100,
        "smoke_executable_bytes": 20,
        "selected_static_archives": 3,
        "selected_static_archives_bytes": 40,
    }
    metrics.write_github_summary(values, summary)
    text = summary.read_text(encoding="utf-8")
    assert "MAVSDK connectivity smoke build metrics" in text
    for key, value in values.items():
        assert f"`{key}`" in text
        assert str(value) in text


def test_main_emits_machine_readable_json(monkeypatch, capsys) -> None:
    values = {
        "build_tree_bytes": 100,
        "smoke_executable_bytes": 20,
        "selected_static_archives": 3,
        "selected_static_archives_bytes": 40,
    }
    monkeypatch.setattr(metrics, "parse_arguments", lambda: (False, None))
    monkeypatch.setattr(metrics, "collect_build_metrics", lambda: values)
    monkeypatch.delenv("GITHUB_STEP_SUMMARY", raising=False)
    assert metrics.main() == 0
    prefix = "mavsdk_connectivity_build_metrics="
    output = capsys.readouterr().out.strip()
    assert output.startswith(prefix)
    assert json.loads(output.removeprefix(prefix)) == values


def test_main_builds_and_writes_evidence(monkeypatch, tmp_path: Path, capsys) -> None:
    output = tmp_path / "evidence" / "metrics.json"
    values = {"build_tree_bytes": 100}
    monkeypatch.setattr(metrics, "parse_arguments", lambda: (True, output))
    monkeypatch.setattr(metrics, "run_build", lambda: {"configure_seconds": 1.0})
    monkeypatch.setattr(metrics, "collect_build_metrics", lambda: values.copy())
    monkeypatch.delenv("GITHUB_STEP_SUMMARY", raising=False)

    assert metrics.main() == 0
    assert json.loads(output.read_text(encoding="utf-8")) == {
        "build_tree_bytes": 100,
        "configure_seconds": 1.0,
    }
    assert "configure_seconds" in capsys.readouterr().out
