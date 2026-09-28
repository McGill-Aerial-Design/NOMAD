# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Regression checks for the post-Edge-Core build and simulation wiring."""

from __future__ import annotations

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


def test_dev_tasks_and_workflows_reference_existing_tasks() -> None:
    pixi = _load_toml("pixi.toml")
    tasks = pixi["tasks"]

    assert tasks["dev"] == "pixi run build-core"
    assert tasks["dev-build"] == "pixi run build-core"
    assert "test-api" not in tasks
    assert "sitl-gimbal" not in tasks

    for workflow_name in ("test.yml", "docker.yml", "sitl.yml", "ros-sim.yml"):
        workflow = (ROOT / ".github" / "workflows" / workflow_name).read_text(encoding="utf-8")
        assert "Dockerfile.dev" not in workflow
        assert "nomad-edge-dev" not in workflow
        if workflow_name == "docker.yml":
            assert "on: workflow_dispatch" in workflow
            assert "packages: write" not in workflow
        for task_name in re.findall(r"pixi run (?:(?:--frozen|--quiet)\s+)*([A-Za-z0-9_][A-Za-z0-9_-]*)", workflow):
            assert task_name in tasks, f"{workflow_name} references missing Pixi task {task_name}"


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
