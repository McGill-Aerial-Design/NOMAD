# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Structural checks for the hosted MAVSDK qualification gates."""

from pathlib import Path

import pytest
import tomllib

ROOT = Path(__file__).resolve().parents[1]


def test_sitl_tools_use_one_configuration_and_build_without_installing() -> None:
    tasks = tomllib.loads((ROOT / "pixi.toml").read_text(encoding="utf-8"))["tasks"]
    command = tasks["build-sitl-tools"]
    assert command.count("cmake -S") == 1
    assert command.count("cmake --build") == 1
    assert "-DBUILD_TESTING=OFF" in command
    assert "--target nomad nomad-runtime nomad-qualification" in command
    assert "pixi run" not in command
    for forbidden in ("--install", "cpack", "deploy", "activate"):
        assert forbidden not in command


@pytest.mark.parametrize(
    "job_name, scenario_tasks",
    [
        (
            "sitl",
            (
                "core-sitl-status",
                "core-sitl-command-flow",
                "core-sitl-mission",
                "core-sitl-velocity-watchdog",
                "core-sitl-payload",
                "core-sitl-link-loss",
                "core-sitl-zero-delivery",
                "core-sitl-link-recovery",
                "core-sitl-gcs-heartbeat",
                "sitl-scenario",
                "sitl-fence",
                "core-sitl-geofence",
            ),
        ),
        (
            "quadplane-observation",
            tuple(
                "core-sitl-quadplane-" + name
                for name in (
                    "observe",
                    "vtol-takeoff",
                    "transition",
                    "route",
                    "recovery",
                    "transition-back",
                    "vtol-landing",
                )
            ),
        ),
    ],
)
def test_full_sitl_jobs_build_once_and_preserve_each_scenario_environment(job_name, scenario_tasks) -> None:
    yaml = pytest.importorskip("yaml")
    tasks = tomllib.loads((ROOT / "pixi.toml").read_text(encoding="utf-8"))["tasks"]
    workflow = yaml.safe_load((ROOT / ".github/workflows/sitl.yml").read_text(encoding="utf-8"))
    steps = workflow["jobs"][job_name]["steps"]
    builds = [i for i, step in enumerate(steps) if step.get("run") == "pixi run build-sitl-tools"]
    assert len(builds) == 1
    for name in scenario_tasks:
        indexes = [i for i, step in enumerate(steps) if step.get("run") == "pixi run --skip-deps " + name]
        assert len(indexes) == 1, f"missing or duplicated qualification scenario: {name}"
        assert builds[0] < indexes[0], f"tools must be built before {name}"
        task = tasks[name]
        assert task["depends-on"] == ["build-sitl-tools"]
        assert task["cmd"].startswith("python ") and "pixi run" not in task["cmd"]
        assert task["env"]["NOMAD_API_KEY"] == "nomad-dev-sitl-key"
        if job_name == "quadplane-observation":
            assert task["env"]["NOMAD_QUADPLANE_SITL_PORT"] == "14580"
    if job_name == "sitl":
        for name in ("core-sitl-zero-delivery", "core-sitl-link-recovery"):
            assert tasks[name]["env"]["NOMAD_CORE_SITL_PORT"] == "14572"
        assert tasks["sitl-scenario"]["env"]["NOMAD_SITL_OPERATOR"] == "tcp:127.0.0.1:5762"


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


def test_safety_changes_require_full_copter_and_quadplane_sitl() -> None:
    workflow = (ROOT / ".github" / "workflows" / "sitl.yml").read_text(encoding="utf-8")
    assert workflow.startswith("name: safety-qualification")
    assert "pull_request:" in workflow and "branches: [main]" in workflow
    assert "workflow_dispatch:" in workflow and "schedule:" in workflow
    assert "scripts/ci/sitl_scope.py" in workflow
    assert "needs.qualification-scope.outputs.required == 'true'" in workflow
    assert workflow.count("if: needs.qualification-scope.outputs.required == 'true'") == 2
    assert "name: Safety qualification gate" in workflow
    assert "needs: [qualification-scope, sitl, quadplane-observation]" in workflow
    assert "mavsdk-connectivity-smoke" not in workflow
    assert "mavsdk-connectivity-runtime.txt" in workflow


def test_connectivity_smoke_has_a_distinct_non_qualification_workflow() -> None:
    workflow = (ROOT / ".github" / "workflows" / "mavsdk-sitl-smoke.yml").read_text(encoding="utf-8")
    assert workflow.startswith("name: mavsdk-connectivity-smoke")
    assert "branches: [main]" in workflow
    assert "MAVSDK connect/status smoke" in workflow
    assert "SITL qualification" in workflow
    assert "name: Safety qualification gate" not in workflow


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
