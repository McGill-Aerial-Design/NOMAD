# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Execute watchdog rendering and status parsing without installing host services."""

import json
import os
import shlex
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def bash() -> str:
    executable = shutil.which("bash")
    git_bash = Path(os.environ.get("ProgramFiles", "")) / "Git/bin/bash.exe"
    if os.name == "nt" and git_bash.is_file():
        executable = str(git_bash)
    if executable is None:
        pytest.skip("Bash is unavailable")
    if subprocess.run([executable, "--version"], capture_output=True, timeout=5).returncode:
        pytest.skip("Bash is unavailable")
    return executable


def run_function(bash: str, source: Path, command: str, **environment: str) -> str:
    result = subprocess.run(
        [bash, "-c", 'source "$1"; ' + command, "watchdog-test", source.as_posix()],
        env=dict(os.environ, **environment),
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, result.stderr
    return result.stdout


@pytest.mark.parametrize("repo_environment", ["", "/incorrect/infra"])
def test_rendered_execstart_is_actual_repository_watchdog(bash: str, repo_environment: str) -> None:
    source = ROOT / "infra/tailscale/scripts/setup.sh"
    rendered = run_function(bash, source, "render_watchdog_service", NOMAD_REPO_ROOT=repo_environment)
    exec_start = next(
        line.removeprefix("ExecStart=") for line in rendered.splitlines() if line.startswith("ExecStart=")
    )
    watchdog = Path(shlex.split(exec_start)[0].replace("%%", "%").replace("$$", "$"))
    # Git Bash emits /c/... paths, which the native Python Path cannot resolve.
    expected = run_function(bash, source, 'printf "%s" "$(cd "$(dirname "$1")" && pwd -P)/watchdog.sh"')
    assert watchdog.as_posix() == expected
    assert "infra/infra" not in exec_start
    assert (source.parent / "watchdog.sh").is_file()


def test_renderer_works_without_git_or_inherited_repo_path(bash: str, tmp_path: Path) -> None:
    checkout = tmp_path / "checkout with spaces % & $"
    shutil.copytree(ROOT / "infra/tailscale", checkout / "infra/tailscale")
    source = checkout / "infra/tailscale/scripts/setup.sh"
    rendered = run_function(
        bash,
        source,
        'HOSTNAME="test-aircraft"; OPERATOR="test-user"; render_watchdog_service',
        NOMAD_REPO_ROOT="/incorrect/infra",
    )
    assert not (checkout / ".git").exists()
    assert 'Environment="TS_HOSTNAME=test-aircraft"' in rendered
    assert 'Environment="TS_OPERATOR=test-user"' in rendered
    assert "checkout with spaces %% & $$" in rendered
    assert 'ExecStart="' in rendered
    assert rendered.count("/infra/tailscale/scripts/watchdog.sh") == 1


@pytest.mark.parametrize("operator", ["", "test-user"])
def test_setup_authentication_uses_same_operator_as_watchdog(bash: str, operator: str) -> None:
    source = ROOT / "infra/tailscale/scripts/setup.sh"
    command = (
        'log() { :; }; sleep() { :; }; tailscale() { printf "ARG:%s\\n" "$@"; }; '
        'HOSTNAME="test-aircraft"; OPERATOR="$TEST_OPERATOR"; authenticate ""; render_watchdog_service'
    )
    output = run_function(bash, source, command, TEST_OPERATOR=operator)
    assert "ARG:--hostname=test-aircraft" in output
    assert f"ARG:--operator={operator}\n" in output
    assert f'Environment="TS_OPERATOR={operator}"' in output


@pytest.mark.parametrize(
    "state,expected", [("Running", "connected"), ("Stopped", "disconnected"), ("NeedsLogin", "needs_auth")]
)
@pytest.mark.parametrize("pretty", [False, True])
def test_watchdog_reads_compact_and_formatted_status(bash: str, state: str, expected: str, pretty: bool) -> None:
    status = json.dumps(
        {"BackendState": state}, indent=2 if pretty else None, separators=None if pretty else (",", ":")
    )
    command = 'tailscale() { printf "%s\\n" "$TEST_STATUS"; }; get_connection_status'
    output = run_function(bash, ROOT / "infra/tailscale/scripts/watchdog.sh", command, TEST_STATUS=status)
    assert output.strip() == expected


def test_watchdog_uses_explicit_service_preferences(bash: str) -> None:
    output = run_function(
        bash,
        ROOT / "infra/tailscale/scripts/watchdog.sh",
        'printf "%s:%s" "$HOSTNAME" "$OPERATOR"',
        TS_HOSTNAME="test-aircraft",
        TS_OPERATOR="test-user",
    )
    assert output == "test-aircraft:test-user"
