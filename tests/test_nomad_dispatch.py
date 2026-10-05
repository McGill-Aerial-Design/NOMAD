# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Execute shell dispatch with missing or broken deployment configuration."""

import os
import shutil
import subprocess
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
    "args", [["--help"], ["list"], ["profile", "list"], ["profile", "load", "groundstation_minimal"]]
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
