# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Structural checks for the hosted MAVSDK qualification gates."""

from pathlib import Path

import tomllib

ROOT = Path(__file__).resolve().parents[1]


def test_mavsdk_job_builds_checks_provenance_and_runs_peer_cases() -> None:
    workflow = (ROOT / ".github" / "workflows" / "test.yml").read_text(encoding="utf-8")
    qualification_job = workflow.split("  mavsdk-qualification:", maxsplit=1)[1]
    assert "os: [ubuntu-latest, windows-latest]" in qualification_job
    assert "mavsdk_build_metrics.py --build" in qualification_job
    assert "pixi run verify-mavsdk-provenance" in qualification_job
    assert "pixi run test-mavsdk-connectivity" in qualification_job
    assert "pixi run test-mavsdk-transport-qualification" in qualification_job
    assert "pixi run test-mavsdk-authority-wire" in qualification_job


def test_mavsdk_job_retains_build_metrics() -> None:
    workflow = (ROOT / ".github" / "workflows" / "test.yml").read_text(encoding="utf-8")
    qualification_job = workflow.split("  mavsdk-qualification:", maxsplit=1)[1]
    assert "mavsdk_build_metrics.py --build" in qualification_job
    assert "--output" in qualification_job
    assert "mavsdk-connectivity-metrics-${{ matrix.os }}" in qualification_job
    assert "actions/upload-artifact@v6" in qualification_job
    assert "if-no-files-found: error" in qualification_job


def test_ros_workflow_does_not_claim_runtime_qualification() -> None:
    workflow = (ROOT / ".github" / "workflows" / "ros-sim.yml").read_text(encoding="utf-8")
    assert "This is compile evidence only" in workflow


def test_sitl_workflow_retains_per_process_runtime_metrics() -> None:
    workflow = (ROOT / ".github" / "workflows" / "sitl.yml").read_text(encoding="utf-8")
    assert workflow.count("mavsdk-connectivity-runtime.txt") == 4
    assert workflow.count("name: mavsdk-connectivity-runtime") == 2
    assert workflow.count("if-no-files-found: error") == 2


def test_renamed_peer_tasks_keep_each_qualification_gate() -> None:
    tasks = tomllib.loads((ROOT / "pixi.toml").read_text(encoding="utf-8"))["tasks"]

    connectivity = tasks["test-mavsdk-connectivity"]
    transport = tasks["test-mavsdk-transport-qualification"]
    authority = tasks["test-mavsdk-authority-wire"]
    assert "mavsdk_connectivity_peer_fixture.py" in connectivity
    assert "tests/test_mavsdk_provenance.py" in connectivity
    assert "mavsdk_connection_fixture.py" in transport
    assert "tests/test_qualification_cli.py" in transport
    assert "mavsdk_authority_wire_fixture.py" in authority
    assert "mavsdk_authority_probe_fixture.py" in authority
