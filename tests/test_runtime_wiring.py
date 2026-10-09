# SPDX-License-Identifier: Apache-2.0
"""Current build/deployment boundaries and runnable documentation contracts."""

from __future__ import annotations

import re
from pathlib import Path

import pytest
import tomllib

ROOT = Path(__file__).resolve().parents[1]


def _load_toml(name: str) -> dict:
    return tomllib.loads((ROOT / name).read_text(encoding="utf-8"))


def test_build_and_package_tasks_keep_installation_explicit() -> None:
    tasks = _load_toml("pixi.toml")["tasks"]
    install_entrypoints = (
        "mission_planner/packaging/INSTALL.ps1",
        "infra/runtime/install_systemd.py",
        "Manage-NomadRuntime.ps1",
    )

    def command_text(name: str) -> str:
        task = tasks[name]
        return task["cmd"] if isinstance(task, dict) else task

    for name, task in tasks.items():
        if name != "test" and not name.startswith(("build-", "test-", "package-")):
            continue
        command = command_text(name)
        assert "cmake --install" not in command, name
        for entrypoint in install_entrypoints:
            assert entrypoint not in command, name

    stage_command = command_text("verify-core-staged-install")
    assert "cmake --install build/package --prefix build/package/stage" in stage_command
    assert tasks["verify-core-staged-install"]["depends-on"] == ["build-core-release"]

    package_command = command_text("package-core")
    assert package_command.startswith("cpack ")
    assert "cmake --build" not in package_command
    assert tasks["package-core"]["depends-on"] == ["build-core-release"]
    assert tasks["verify-core-package"]["depends-on"] == ["package-core"]
    assert "cmake --install" not in command_text("test-mavsdk-authority-wire")

    install_task = tasks["install-core"]
    assert install_task["args"] == ["prefix"]
    assert "--prefix {{ prefix }}" in install_task["cmd"]
    assert install_task["depends-on"] == ["build-core-release"]

    plugin_builder = (ROOT / "scripts" / "build" / "build_plugin_windows.ps1").read_text(encoding="utf-8")
    assert "Deployment skipped" in plugin_builder
    assert "mission_planner/packaging/INSTALL.ps1" not in plugin_builder


def test_production_and_qualification_builds_select_distinct_targets() -> None:
    tasks = _load_toml("pixi.toml")["tasks"]

    production = tasks["build-core"]
    release = tasks["build-core-release"]
    qualification = tasks["build-qualification-cli"]
    assert "-DBUILD_TESTING=OFF" in production
    assert "--target nomad nomad-runtime" in production
    assert "-DBUILD_TESTING=OFF" in release
    assert "--target nomad nomad-runtime" in release
    assert "-DBUILD_TESTING=OFF" in qualification
    assert "--target nomad-qualification" in qualification


def test_first_party_pixi_run_guidance_references_existing_tasks() -> None:
    tasks = _load_toml("pixi.toml")["tasks"]
    caller_files = [ROOT / name for name in ("README.md", "CONTRIBUTING.md", "AGENTS.md", "pixi.toml")]
    caller_files.extend((ROOT / ".vscode").rglob("tasks.json"))
    caller_files.extend((ROOT / ".github").rglob("*.yml"))
    caller_files.extend((ROOT / "docs").rglob("*.md"))
    caller_files.extend((ROOT / "scripts").rglob("*.py"))
    caller_files.extend((ROOT / "scripts").rglob("*.ps1"))
    caller_files.extend((ROOT / "scripts").rglob("*.sh"))
    caller_files.extend((ROOT / "tests").rglob("*.py"))
    caller_files.extend((ROOT / "mission_planner").glob("README.md"))
    caller_files.extend((ROOT / "mission_planner" / "packaging").glob("README.md"))

    command_tools = {"python", "pre-commit"}
    for path in caller_files:
        text = path.read_text(encoding="utf-8")
        task_names = re.findall(
            r"\bpixi run(?:\s+--(?:frozen|quiet))*\s+([A-Za-z0-9_][A-Za-z0-9_-]*)",
            text,
        )
        for task_name in task_names:
            if task_name in command_tools:
                continue
            assert task_name in tasks, f"{path.relative_to(ROOT)} references missing Pixi task {task_name}"


def test_compose_feeds_sitl_the_parameter_files_its_entrypoint_requires() -> None:
    """ArduPilot streams position/attitude/status only once asked, and the core
    never asks, so the stack must seed the SERIAL0 group rates itself."""
    yaml = pytest.importorskip("yaml")
    compose = yaml.safe_load((ROOT / "docker" / "docker-compose.dev.yml").read_text(encoding="utf-8"))
    sitl = compose["services"]["sitl"]
    entrypoint = (ROOT / "docker" / "sitl-entrypoint.sh").read_text(encoding="utf-8")

    mounts = [mount.split(":")[1] for mount in sitl["volumes"]]
    for variable in ("SITL_FENCE_DEFAULTS", "SITL_STREAM_DEFAULTS"):
        assert f'--add-param-file="${{{variable}}}"' in entrypoint, f"entrypoint does not apply {variable}"
        container_path = sitl["environment"][variable]
        assert container_path in mounts, f"{container_path} is not mounted into the container"

    stream_params = (ROOT / "docker" / "sitl-streams.parm").read_text(encoding="utf-8")
    for parameter in ("SR0_POSITION", "SR0_EXT_STAT", "SR0_EXTRA1", "SR0_EXTRA2"):
        assert re.search(rf"^{parameter} [1-9]", stream_params, re.MULTILINE), parameter


def test_ci_script_entrypoints_exist() -> None:
    for workflow in (ROOT / ".github/workflows").glob("*.yml"):
        for script in re.findall(
            r"(?:python(?:3)?|pwsh -File) (scripts/[A-Za-z0-9_./-]+\.(?:py|ps1))", workflow.read_text()
        ):
            assert (ROOT / script).is_file(), f"{workflow.name}: missing runner {script}"
