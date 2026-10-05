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


@pytest.mark.skipif(os.name != "nt", reason="Windows path qualification")
@pytest.mark.parametrize("path", [r"C:nomad-runtime.exe", r"\NOMAD\nomad-runtime.exe"])
def test_windows_plan_rejects_drive_relative_paths(path):
    result = subprocess.run(
        [
            "powershell",
            "-NoProfile",
            "-File",
            str(ROOT / "infra/runtime/Manage-NomadRuntime.ps1"),
            "-Action",
            "Plan",
            "-Executable",
            path,
            "-Config",
            r"C:\NOMAD\runtime.json",
        ],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode != 0 and "absolute paths" in result.stderr


def test_lifecycle_templates_contain_no_actuation_credentials() -> None:
    example = json.loads((ROOT / "infra/runtime/runtime.example.json").read_text(encoding="utf-8"))
    assert example["NOMAD_API_KEY"] == ""
    assert example["NOMAD_CLIENT_CREDENTIALS_FILE"] == ""
    assert "NOMAD_CLIENT_CREDENTIAL" not in example
    assert example["NOMAD_AUDIT_DIRECTORY"] == ""


def test_runtime_template_exposes_optional_actuator_path():
    example = json.loads((ROOT / "infra/runtime/runtime.example.json").read_text())
    assert example["NOMAD_ACTUATORS_FILE"] == ""


@pytest.mark.parametrize("key", ["NOMAD_AUDIT_DIRECTORY", "NOMAD_ACTUATORS_FILE"])
def test_service_rejects_state_outside_writable_tree(tmp_path, key):
    state = tmp_path / "state"
    state.mkdir()
    config = tmp_path / "runtime.json"
    settings = {"NOMAD_AUDIT_DIRECTORY": str(state / "audit"), "NOMAD_ACTUATORS_FILE": ""}
    settings[key] = str(tmp_path / "etc" / "state.json")
    config.write_text(json.dumps(settings))
    with pytest.raises(ValueError, match="beneath --state"):
        install_systemd.validate_state_paths(config, state)


def test_service_accepts_private_actuator_state_location(tmp_path):
    state = tmp_path / "state"
    state.mkdir()
    config = tmp_path / "runtime.json"
    config.write_text(
        json.dumps(
            {
                "NOMAD_AUDIT_DIRECTORY": str(state / "audit"),
                "NOMAD_ACTUATORS_FILE": str(state / "actuators" / "actuators.json"),
            }
        )
    )
    install_systemd.validate_state_paths(config, state)
    config.write_text(json.dumps({"NOMAD_AUDIT_DIRECTORY": str(state), "NOMAD_ACTUATORS_FILE": ""}))
    install_systemd.validate_state_paths(config, state)


@pytest.mark.skipif(os.name == "nt", reason="POSIX symbolic link fixture")
def test_service_rejects_linked_state_parent(tmp_path):
    state = tmp_path / "state"
    state.mkdir()
    (state / "linked").symlink_to(state, target_is_directory=True)
    config = tmp_path / "runtime.json"
    config.write_text(
        json.dumps(
            {
                "NOMAD_AUDIT_DIRECTORY": str(state / "audit"),
                "NOMAD_ACTUATORS_FILE": str(state / "linked" / "actuators.json"),
            }
        )
    )
    with pytest.raises(ValueError, match="symbolic links"):
        install_systemd.validate_state_paths(config, state)


def run_windows_protect(config):
    return subprocess.run(
        [
            "powershell",
            "-NoProfile",
            "-File",
            str(ROOT / "tests/runtime_protect_fixture.ps1"),
            "-Helper",
            str(ROOT / "infra/runtime/Manage-NomadRuntime.ps1"),
            "-Config",
            str(config),
        ],
        capture_output=True,
        text=True,
        timeout=10,
    )


@pytest.mark.skipif(os.name != "nt", reason="Windows service ACL provisioning")
@pytest.mark.parametrize("enabled", [False, True])
def test_windows_protect_includes_actuator_file_and_writable_parent(tmp_path, enabled):
    config, credentials, audit = (tmp_path / name for name in ("runtime.json", "clients.json", "audit"))
    credentials.write_text("{}")
    audit.mkdir()
    parent = tmp_path / "actuators"
    parent.mkdir()
    actuator = parent / "actuators.json"
    actuator.write_text('{"actuator_configs": []}')
    config.write_text(
        json.dumps(
            {
                "NOMAD_CLIENT_CREDENTIALS_FILE": str(credentials),
                "NOMAD_AUDIT_DIRECTORY": str(audit),
                "NOMAD_ACTUATORS_FILE": str(actuator) if enabled else "",
            }
        )
    )
    result = run_windows_protect(config)
    assert result.returncode == 0, result.stderr
    changes = {Path(change["Path"]): change for change in json.loads(result.stdout)}
    assert set(changes) == {config, credentials, audit} | ({parent, actuator} if enabled else set())
    for change in changes.values():
        assert change["Owner"] == "S-1-5-19" and change["Protected"]
        assert {rule["SID"] for rule in change["Rules"]} == {"S-1-5-19", "S-1-5-18", "S-1-5-32-544"}
        assert all(rule["Rights"] == "FullControl" for rule in change["Rules"])
    if enabled:
        assert all("ContainerInherit" in rule["Inheritance"] for rule in changes[parent]["Rules"])
        (parent / "unrelated.txt").write_text("must not change permissions")
        refused = run_windows_protect(config)
        assert refused.returncode != 0 and "dedicated actuator directory" in refused.stderr
        assert refused.stdout == ""
