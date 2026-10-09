# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Path rules that decide when hosted Windows plugin and router checks run."""

from scripts.ci.csharp_scope import main, requires_windows_qualification


def test_mission_planner_router_and_qualification_changes_run_windows_checks() -> None:
    paths = [
        "mission_planner/src/Control/GimbalController.cs",
        "infra/transport/ground_router/Router.cs",
        "scripts/dev/release_process_qualification.py",
        "tests/test_mission_planner_command_boundary.py",
    ]

    assert requires_windows_qualification(paths)


def test_csharp_workflow_changes_run_windows_checks() -> None:
    assert requires_windows_qualification([".github/workflows/csharp.yml"])


def test_unrelated_changes_skip_windows_builds() -> None:
    paths = ["docs/architecture.md", "src/runtime/runtime.cpp", "python/tools/plot.py"]

    assert not requires_windows_qualification(paths)


def test_unknown_git_base_fails_closed_to_windows_qualification(monkeypatch, tmp_path) -> None:
    output = tmp_path / "github-output.txt"
    monkeypatch.setenv("EVENT_NAME", "pull_request")
    monkeypatch.setenv("PUSH_BEFORE", "")
    monkeypatch.setenv("BASE_BRANCH", "")
    monkeypatch.setenv("HEAD_SHA", "head")
    monkeypatch.setenv("GITHUB_OUTPUT", str(output))

    assert main() == 0
    assert output.read_text(encoding="utf-8") == "required=true\n"
