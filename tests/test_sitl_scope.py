# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Path rules that decide when full Copter and QuadPlane SITL runs are required."""

from scripts.ci.sitl_scope import main, requires_full_sitl


def test_runtime_and_mission_planner_safety_changes_require_full_sitl() -> None:
    paths = [
        "src/runtime/runtime.cpp",
        "src/qualification/main.cpp",
        "tools/runtime/main.cpp",
        "include/nomad/vehicle.hpp",
        "tests/safety_test.cpp",
        "mission_planner/src/Geofence/BoundaryManager.cs",
        "mission_planner/src/Control/GimbalController.cs",
        "mission_planner/src/Plugin/NOMADPlugin.cs",
        "docker/docker-compose.quadplane.yml",
    ]

    assert requires_full_sitl(paths)


def test_sitl_workflow_changes_require_full_sitl() -> None:
    assert requires_full_sitl([".github/workflows/sitl.yml"])


def test_docs_and_unrelated_ground_tools_skip_full_sitl() -> None:
    paths = ["docs/architecture.md", "python/tools/plot.py", "mission_planner/src/Views/NOMADDashboardView.cs"]

    assert not requires_full_sitl(paths)


def test_unknown_git_base_fails_closed_to_full_sitl(monkeypatch, tmp_path) -> None:
    output = tmp_path / "github-output.txt"
    monkeypatch.setenv("EVENT_NAME", "pull_request")
    monkeypatch.setenv("PUSH_BEFORE", "")
    monkeypatch.setenv("BASE_BRANCH", "")
    monkeypatch.setenv("HEAD_SHA", "head")
    monkeypatch.setenv("GITHUB_OUTPUT", str(output))

    assert main() == 0
    assert output.read_text(encoding="utf-8") == "required=true\n"
