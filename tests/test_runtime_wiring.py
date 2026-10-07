# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Regression checks for the post-Edge-Core build and simulation wiring."""

from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

import pytest
import tomllib

ROOT = Path(__file__).resolve().parents[1]


def _load_toml(name: str) -> dict:
    return tomllib.loads((ROOT / name).read_text(encoding="utf-8"))


def _load_jsonc(name: str) -> dict:
    text = (ROOT / name).read_text(encoding="utf-8")
    text = re.sub(r"(?m)^[ \t]*//[^\r\n]*(?:\r?\n|$)", "", text)
    return json.loads(text)


def _render_sitl_outputs(template: str, *, ros: bool) -> str:
    ros_output = "--out udp:nomad_vehicle_node:14552" if ros else ""
    return template.replace("${NOMAD_SITL_ROS_OUTPUT:-}", ros_output)


def _build_wheel_in_copy(tmp_path: Path) -> Path:
    source = tmp_path / "source"
    source.mkdir()
    for name in ("pyproject.toml", "README.md", "LICENSE", "NOTICE"):
        shutil.copy2(ROOT / name, source / name)
    shutil.copytree(ROOT / "python", source / "python")
    shutil.copytree(ROOT / "infra", source / "infra")

    result = subprocess.run(
        [sys.executable, "-c", "import setuptools.build_meta as b; print(b.build_wheel('dist'))"],
        cwd=source,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stderr
    wheels = list((source / "dist").glob("*.whl"))
    assert len(wheels) == 1
    return wheels[0]


def test_distribution_packages_live_python_tools_without_edge_core(tmp_path: Path) -> None:
    from infra.tailscale import tailscale_manager
    from python.tools import simple_video_bridge

    metadata = _load_toml("pyproject.toml")
    project = metadata["project"]
    packages = metadata["tool"]["setuptools"]["packages"]["find"]["include"]

    assert project["name"] == "nomad-tools"
    assert project["dependencies"] == ["numpy>=1.26"]
    assert "scripts" not in project
    assert "python.tools" in packages
    assert "infra.tailscale" in packages
    assert (ROOT / "python" / "__init__.py").is_file()
    assert (ROOT / "python" / "tools" / "simple_video_bridge.py").is_file()
    assert callable(simple_video_bridge.main)
    assert hasattr(tailscale_manager, "TailscaleManager")
    assert not (ROOT / "edge_core").exists()

    wheel = _build_wheel_in_copy(tmp_path)
    with zipfile.ZipFile(wheel) as archive:
        names = archive.namelist()
    assert "python/tools/simple_video_bridge.py" in names
    assert "infra/tailscale/tailscale_manager.py" in names
    assert not any("edge_core" in name for name in names)
    assert not any(name.endswith("entry_points.txt") for name in names)


def test_removed_aliases_and_workflows_are_current() -> None:
    pixi = _load_toml("pixi.toml")
    tasks = pixi["tasks"]

    removed_aliases = {
        "dev",
        "dev-build",
        "test-fast",
        "build-core-mavsdk",
        "mavsdk-phase-a-smoke",
        "test-mavsdk-phase-a",
        "check-mavsdk-phase-a",
        "build-mavsdk-phase-b",
        "test-mavsdk-phase-b",
        "build-core-qualification",
        "verify-core-install",
    }
    assert removed_aliases.isdisjoint(tasks)
    assert "test-api" not in tasks
    assert "sitl-gimbal" not in tasks

    for workflow_path in (ROOT / ".github" / "workflows").glob("*.yml"):
        workflow = workflow_path.read_text(encoding="utf-8")
        assert "Dockerfile.dev" not in workflow
        assert "nomad-edge-dev" not in workflow
        if workflow_path.name == "docker.yml":
            assert "on: workflow_dispatch" in workflow
            assert "packages: write" not in workflow
        for task_name in re.findall(r"pixi run (?:(?:--frozen|--quiet)\s+)*([A-Za-z0-9_][A-Za-z0-9_-]*)", workflow):
            if task_name not in {"python", "pre-commit"}:
                assert task_name in tasks, f"{workflow_path.name} references missing Pixi task {task_name}"


def test_vscode_tasks_and_dockerfile_suggestions_are_current() -> None:
    pixi_tasks = _load_toml("pixi.toml")["tasks"]
    vscode_tasks = _load_jsonc(".vscode/tasks.json")["tasks"]
    vscode_task_text = json.dumps(vscode_tasks)

    assert "sim-ros-perception-up" not in pixi_tasks
    assert not any(name.startswith("sim-gazebo-") for name in pixi_tasks)
    assert "sim-ros-perception-up" not in vscode_task_text
    assert "sim-gazebo-" not in vscode_task_text

    for task in vscode_tasks:
        for task_name in re.findall(r"\bpixi run\s+([A-Za-z0-9_-]+)", task.get("command", "")):
            assert task_name in pixi_tasks, f"VS Code task {task['label']} references missing Pixi task {task_name}"

    dockerfiles = _load_jsonc(".vscode/settings.json")["docker.dockerfileSearchList"]
    for dockerfile in dockerfiles:
        assert (ROOT / dockerfile).is_file(), f"VS Code Dockerfile suggestion does not exist: {dockerfile}"


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
    caller_files.append(ROOT / "examples" / "README.md")

    excluded_history = {ROOT / "docs" / "migration.md"}
    command_tools = {"python", "pre-commit"}
    for path in caller_files:
        if path in excluded_history:
            continue
        text = path.read_text(encoding="utf-8")
        task_names = re.findall(
            r"\bpixi run(?:\s+--(?:frozen|quiet))*\s+([A-Za-z0-9_][A-Za-z0-9_-]*)",
            text,
        )
        for task_name in task_names:
            if task_name in command_tools:
                continue
            assert task_name in tasks, f"{path.relative_to(ROOT)} references missing Pixi task {task_name}"


def test_build_and_package_tasks_keep_installation_explicit() -> None:
    tasks = _load_toml("pixi.toml")["tasks"]
    install_entrypoints = ("mission_planner/packaging/INSTALL.ps1", "infra/systemd/install.sh", "setup_jetson")

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


def test_mavlink_and_image_dependencies_match_current_consumers() -> None:
    metadata = _load_toml("pyproject.toml")["project"]
    assert metadata["name"] == "nomad-tools"
    assert "pymavlink>=2.4" in metadata["optional-dependencies"]["dev"]

    pixi_lock = (ROOT / "pixi.lock").read_text(encoding="utf-8")
    sitl_image = (ROOT / "docker" / "Dockerfile.sitl-plane").read_text(encoding="utf-8")
    ros_image = (ROOT / "docker" / "Dockerfile.sim-ros").read_text(encoding="utf-8")
    jetson_image = (ROOT / "docker" / "Dockerfile.jetson").read_text(encoding="utf-8")
    isaac_image = (ROOT / "docker" / "Dockerfile.sim-isaac").read_text(encoding="utf-8")

    assert "pymavlink" in sitl_image
    assert "pymavlink" in ros_image
    assert "pymavlink" not in jetson_image
    assert "pymavlink" not in isaac_image
    assert "name: pymavlink" in pixi_lock
    assert "pytest" in ros_image
    assert "'numpy<2'" in jetson_image
    assert "'numpy<2'" in ros_image
    for image in (sitl_image, ros_image, jetson_image, isaac_image):
        assert "transforms3d" not in image


def test_mission_planner_packages_only_current_video_dependencies() -> None:
    project = (ROOT / "mission_planner" / "src" / "NOMADPlugin.csproj").read_text(encoding="utf-8")
    source = "\n".join(path.read_text(encoding="utf-8") for path in (ROOT / "mission_planner" / "src").rglob("*.cs"))
    installer = (ROOT / "mission_planner" / "packaging" / "INSTALL.ps1").read_text(encoding="utf-8")
    workflow = (ROOT / ".github" / "workflows" / "csharp.yml").read_text(encoding="utf-8")
    release_workflow = (ROOT / ".github" / "workflows" / "release.yml").read_text(encoding="utf-8")

    retired_references = (
        "OpenTK",
        "HelixToolkit",
        "LibVLCSharp",
        "PresentationCore",
        "PresentationFramework",
        "WindowsBase",
        "WindowsFormsIntegration",
        "System.Xaml",
    )
    for retired_reference in retired_references:
        assert retired_reference not in project
        assert retired_reference not in source

    assert '<Reference Include="System.Memory">' in project
    assert "GStreamer" in source
    assert '<Reference Include="SkiaSharp">' not in project
    assert '<Reference Include="MissionPlanner.Drawing">' not in project
    assert "--component', 'plugin'" in installer
    assert "ValidateSet('verify', 'stage', 'adopt', 'activate'" in installer
    assert "& $Python @toolArguments" in installer
    assert "Copy-Item" not in installer
    assert "path: mission_planner/src/bin/Release/NOMADPlugin.dll" in workflow
    assert "Copy-Item mission_planner/src/bin/Release/NOMADPlugin.dll $stage" in release_workflow
    assert "Copy-Item mission_planner/packaging/libvlc-windows" not in release_workflow

    managed_libs = ROOT / "mission_planner" / "third_party" / "libvlc"
    assert not managed_libs.exists() or not list(managed_libs.glob("*.dll"))
    for script in ("copy-libvlc.ps1", "copy-managed-libs.ps1", "fetch-libvlc.ps1"):
        assert not (ROOT / "mission_planner" / "packaging" / script).exists()


def test_unconsumed_opencv_cuda_setup_entrypoint_is_removed() -> None:
    assert not (ROOT / "scripts" / "setup" / "install_opencv_cuda_container.sh").exists()


def test_compose_has_valid_sitl_only_and_ros_output_paths() -> None:
    yaml = pytest.importorskip("yaml")
    compose_path = ROOT / "docker" / "docker-compose.dev.yml"
    compose = yaml.safe_load(compose_path.read_text(encoding="utf-8"))
    services = compose["services"]

    assert "edge_core" not in services
    for name, service in services.items():
        for dependency in service.get("depends_on", []):
            assert dependency in services, f"{name}: missing dependency {dependency}"
        if "build" in service:
            build = service["build"]
            context = (compose_path.parent / build["context"]).resolve()
            assert (context / build["dockerfile"]).is_file(), name
    sitl = services["sitl"]
    template = sitl["environment"]["SITL_UDP_OUTPUT_ADDRESS"]
    sitl_only = _render_sitl_outputs(template, ros=False)
    ros_stack = _render_sitl_outputs(template, ros=True)
    assert "nomad_vehicle_node" not in sitl_only
    assert "host.docker.internal:14570" in sitl_only
    assert "host.docker.internal:14572" in sitl_only
    assert "nomad_vehicle_node:14552" in ros_stack
    destinations = re.findall(r"udp:[^ ]+", sitl_only)
    assert len(destinations) == len(set(destinations)), destinations
    task = _load_toml("pixi.toml")["tasks"]["sim-ros-up"]
    assert task["env"]["NOMAD_SITL_ROS_OUTPUT"] in ros_stack

    dockerfile = (ROOT / "docker" / "Dockerfile.sim-ros").read_text(encoding="utf-8")
    assert "COPY python/ /opt/nomad/python/" in dockerfile
    assert "COPY edge_core/" not in dockerfile


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
