# SPDX-License-Identifier: Apache-2.0
"""Service provisioning verification without privileged installation or hardware."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from infra.runtime import install_systemd

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.skipif(os.name == "nt", reason="POSIX systemd paths")
def test_render_service_paths_and_configuration(tmp_path: Path) -> None:
    unit = install_systemd.render_unit(
        Path("/opt/nomad release/bin/nomad-runtime"), Path("/etc/nomad/runtime.json"), Path("/var/lib/nomad"), "nomad"
    )
    assert 'ExecStart="/opt/nomad release/bin/nomad-runtime" --config "/etc/nomad/runtime.json"' in unit
    assert 'ReadWritePaths="/var/lib/nomad"' in unit
    assert "RestartPreventExitStatus=78" in unit
    assert "RestartSec=5s" in unit and "StartLimitBurst=3" in unit
    assert "TimeoutStopSec=30s" in unit and "KillSignal=SIGTERM" in unit
    assert "Requires=" not in unit and "network-online" not in unit
    assert "NOMAD_API_KEY=" not in unit
    tool = shutil.which("systemd-analyze")
    if tool:
        syntax = install_systemd.render_unit(Path("/usr/bin/true"), tmp_path / "runtime.json", tmp_path, "root")
        path = tmp_path / "nomad-runtime.service"
        path.write_text(syntax, encoding="utf-8")
        result = subprocess.run([tool, "verify", str(path)], capture_output=True, text=True, timeout=10)
        assert result.returncode == 0, result.stderr


@pytest.mark.skipif(os.name == "nt", reason="POSIX systemd paths")
@pytest.mark.parametrize(
    "path", ["relative/bin", '/opt/inject"', "/opt/inject\nExecStart=/usr/bin/false", "/opt/$SECRET"]
)
def test_render_rejects_unit_injection(path: str) -> None:
    with pytest.raises(ValueError):
        install_systemd.render_unit(Path(path), Path("/etc/nomad.json"), Path("/var/lib/nomad"), "nomad")


@pytest.mark.skipif(os.name == "nt", reason="POSIX systemd paths")
def test_render_escapes_systemd_specifiers() -> None:
    unit = install_systemd.render_unit(
        Path("/opt/nomad%prod/bin/runtime"), Path("/etc/runtime.json"), Path("/var/lib/nomad"), "nomad"
    )
    assert "/opt/nomad%%prod/bin/runtime" in unit
    with pytest.raises(ValueError):
        install_systemd.render_unit(
            Path("/bin/runtime"), Path("/etc/runtime.json"), Path("/var/lib/nomad"), "bad\nUser=root"
        )


@pytest.mark.skipif(os.name != "nt", reason="Windows provisioning command generation")
def test_windows_plan_quotes_paths_and_bounds_recovery() -> None:
    script = ROOT / "infra/runtime/Manage-NomadRuntime.ps1"
    expression = f"& '{script}' -Action Plan -Executable 'C:\\NOMAD release\\bin\\nomad-runtime.exe' "
    expression += "-Config 'C:\\ProgramData\\NOMAD\\runtime.json' | ConvertTo-Json -Compress"
    result = subprocess.run(
        ["powershell", "-NoProfile", "-Command", expression], capture_output=True, text=True, timeout=10
    )
    assert result.returncode == 0, result.stderr
    plan = json.loads(result.stdout)
    assert (
        plan["BinaryPath"]
        == '"C:\\NOMAD release\\bin\\nomad-runtime.exe" --service --config "C:\\ProgramData\\NOMAD\\runtime.json"'
    )
    assert plan["Account"] == "NT AUTHORITY\\LocalService"
    assert plan["Start"] == "demand"
    assert plan["Recovery"] == "restart/5000/restart/30000/none/0"
    assert plan["NonCrashRecovery"] is False


def test_lifecycle_templates_contain_no_actuation_credentials() -> None:
    example = json.loads((ROOT / "infra/runtime/runtime.example.json").read_text(encoding="utf-8"))
    assert example["NOMAD_API_KEY"] == ""
    assert example["NOMAD_CLIENT_CREDENTIALS_FILE"] == ""
    assert "NOMAD_CLIENT_CREDENTIAL" not in example
    assert example["NOMAD_AUDIT_DIRECTORY"] == ""
