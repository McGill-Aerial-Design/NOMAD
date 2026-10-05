# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Execute shell dispatch with missing or broken deployment configuration."""

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def shell_cli(tmp_path: Path):
    bash = shutil.which("bash")
    git_bash = Path(os.environ.get("ProgramFiles", "")) / "Git/bin/bash.exe"
    if os.name == "nt" and git_bash.is_file():
        bash = str(git_bash)
    if bash is None:
        pytest.skip("Bash is unavailable")
    probe = subprocess.run([bash, "--version"], capture_output=True, timeout=5)
    if probe.returncode != 0:
        pytest.skip("Bash is unavailable")
    scripts = tmp_path / "scripts"
    (scripts / "lib").mkdir(parents=True)
    shutil.copyfile(ROOT / "scripts/nomad", scripts / "nomad")
    shutil.copyfile(ROOT / "scripts/lib/common.sh", scripts / "lib/common.sh")
    # Isolate dispatch from the independently tested Python profile manager.
    (scripts / "nomad-profile").write_text('printf "PROFILE_DISPATCH:%s\\n" "$*"\n', encoding="utf-8")
    return bash, scripts / "nomad", tmp_path / "nomad.env"


@pytest.mark.parametrize("broken", [False, True])
@pytest.mark.parametrize(
    "args",
    [["--help"], ["list"], ["profile", "list"], ["profile", "show"], ["profile", "load", "groundstation_minimal"]],
)
def test_recovery_dispatch_does_not_source_active_env(shell_cli, broken: bool, args: list[str]) -> None:
    bash, script, active = shell_cli
    if broken:
        active.write_text("exit 93\n", encoding="utf-8")
    env = dict(os.environ, NOMAD_ENV_FILE=active.as_posix())
    result = subprocess.run([bash, script.as_posix(), *args], env=env, capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert "Config not found" not in result.stderr
    if args[0] == "profile":
        assert f"PROFILE_DISPATCH:{' '.join(args[1:])}" in result.stdout
    elif args[0] == "--help":
        assert "onboard_companion|groundstation_gpu|groundstation_minimal" in result.stdout
        assert "sim|drone|dev" not in result.stdout
    else:
        assert "Aircraft-side mavlink-routerd" in result.stdout


def test_config_dispatch_requires_active_env(shell_cli) -> None:
    bash, script, active = shell_cli
    result = subprocess.run(
        [bash, script.as_posix(), "config"],
        env=dict(os.environ, NOMAD_ENV_FILE=active.as_posix()),
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 1
    assert "Config not found" in result.stderr


@pytest.fixture
def real_profile_cli(shell_cli):
    bash, script, _ = shell_cli
    scripts = script.parent
    for name in (
        "nomad-profile",
        "profile.py",
        "profile_application.py",
        "profile_mission_planner.py",
        "profile_settings.py",
    ):
        shutil.copyfile(ROOT / "scripts" / name, scripts / name)
    config = scripts.parent / "config"
    shutil.copytree(ROOT / "config/profiles", config / "profiles")
    shutil.copyfile(ROOT / "config/nomad.env.example", config / "nomad.env.example")
    binaries = scripts.parent / "bin"
    binaries.mkdir()
    python3 = binaries / "python3"
    python3.write_text('#!/bin/bash\nexec "$NOMAD_TEST_PYTHON" "$@"\n', encoding="utf-8")
    python3.chmod(0o755)
    env = dict(
        os.environ,
        NOMAD_TEST_PYTHON=Path(sys.executable).as_posix(),
        NOMAD_MP_CONFIG=(scripts.parent / "mp.json").as_posix(),
        PATH=str(binaries) + os.pathsep + os.environ["PATH"],
    )
    env.pop("NOMAD_ENV_FILE", None)
    env.pop("NOMAD_REPO_ROOT", None)
    return bash, script, config / "nomad.env", env


@pytest.mark.parametrize("broken", [False, True])
@pytest.mark.parametrize("action", ["list", "show", "load"])
def test_real_profile_recovery_never_executes_active_env(real_profile_cli, broken: bool, action: str) -> None:
    bash, script, active, env = real_profile_cli
    marker = active.parent / "must-not-execute"
    if broken:
        active.write_text(f'touch "{marker.as_posix()}"\nexit 93\nBROKEN="unterminated\n', encoding="utf-8")
    args = ["profile", action] + (["groundstation_minimal"] if action == "load" else [])
    result = subprocess.run([bash, script.as_posix(), *args], env=env, capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert not marker.exists(), "Recovery command sourced executable active configuration"
    if action == "list":
        assert "groundstation_minimal" in result.stdout
    elif action == "show":
        assert ("Active profile:" if broken else "No active config") in result.stdout
    else:
        assert "[OK] Profile load completed: groundstation_minimal" in result.stdout
        content = active.read_text(encoding="utf-8")
        assert "NOMAD_PROFILE=groundstation_minimal" in content
        assert "BROKEN=" not in content


def test_real_profile_show_reports_unreadable_env(real_profile_cli) -> None:
    bash, script, active, env = real_profile_cli
    active.write_bytes(b"\xff\xfe")
    result = subprocess.run(
        [bash, script.as_posix(), "profile", "show"], env=env, capture_output=True, text=True, timeout=10
    )
    assert result.returncode == 0, result.stderr
    assert "Active config is unreadable" in result.stdout
    assert active.read_bytes() == b"\xff\xfe"
