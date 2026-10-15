# SPDX-License-Identifier: Apache-2.0
"""Service provisioning verification without privileged installation or hardware."""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import time
from pathlib import Path

import pytest

from infra.runtime import install_systemd

ROOT = Path(__file__).resolve().parents[1]
PLAN_PHASES = ("entry", "before_helper", "after_helper", "after_json", "done")


def plan_phase_facts(output: str | bytes | None) -> dict:
    """Project only fixed child milestones; never export captured process output."""
    text = output.decode("ascii", errors="replace") if isinstance(output, bytes) else output or ""
    phases, rejected = [], 0
    for line in text.splitlines():
        if not line.startswith("nomad_plan_phase:"):
            continue
        match = re.fullmatch(r"nomad_plan_phase:([a-z_]+):([0-9]{1,5})", line)
        expected = PLAN_PHASES[len(phases)] if len(phases) < len(PLAN_PHASES) else None
        if (
            match is None
            or match[1] != expected
            or int(match[2]) > 60000
            or (phases and int(match[2]) < phases[-1]["elapsed_ms"])
        ):
            rejected += 1
            continue
        phases.append({"phase": match[1], "elapsed_ms": int(match[2])})
    status = "complete" if len(phases) == len(PLAN_PHASES) else "partial" if phases else "missing"
    return {"phase_status": "invalid" if rejected else status, "phases": phases, "rejected_candidates": rejected}


def run_windows_plan(script: Path) -> subprocess.CompletedProcess:
    expression = """
$ErrorActionPreference = 'Stop'
$nomadPlanClock = [Diagnostics.Stopwatch]::StartNew()
function Write-NomadPlanPhase([string]$phase) {
    [Console]::Error.WriteLine(('nomad_plan_phase:{0}:{1}' -f $phase, $nomadPlanClock.ElapsedMilliseconds))
}
try {
    Write-NomadPlanPhase 'entry'
    Write-NomadPlanPhase 'before_helper'
"""
    expression += f"$nomadPlan = & '{script}' -Action Plan -Executable 'C:\\NOMAD release\\bin\\nomad-runtime.exe' "
    expression += "-Config 'C:\\ProgramData\\NOMAD\\runtime.json'\n"
    expression += """
    Write-NomadPlanPhase 'after_helper'
    $nomadPlanJson = $nomadPlan | ConvertTo-Json -Compress
    Write-NomadPlanPhase 'after_json'
    [Console]::Out.WriteLine($nomadPlanJson)
    Write-NomadPlanPhase 'done'
} catch { exit 1 }
"""
    started = time.monotonic_ns()
    timed_out = False
    result = None
    try:
        result = subprocess.run(
            ["powershell", "-NoProfile", "-Command", expression], capture_output=True, text=True, timeout=10
        )
        captured_error = result.stderr
    except subprocess.TimeoutExpired as error:
        captured_error = error.stderr
        timed_out = True
    facts = {
        "event": "runtime_service_plan_timing",
        "result": "timeout" if timed_out else "returned",
        "exit_code": result.returncode if result is not None else None,
        "parent_elapsed_ms": (time.monotonic_ns() - started) // 1_000_000,
        **plan_phase_facts(captured_error),
    }
    print(json.dumps(facts, sort_keys=True), flush=True)
    # Report outside the handler so pytest cannot print TimeoutExpired's private argv/buffers.
    if timed_out:
        pytest.fail("Windows service Plan exceeded its unchanged 10-second limit", pytrace=False)
    return result


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
    result = run_windows_plan(script)
    assert result.returncode == 0, "Windows service Plan failed; see fixed phase facts"
    plan = json.loads(result.stdout)
    assert (
        plan["BinaryPath"]
        == '"C:\\NOMAD release\\bin\\nomad-runtime.exe" --service --config "C:\\ProgramData\\NOMAD\\runtime.json"'
    )
    assert plan["Account"] == "NT AUTHORITY\\LocalService"
    assert plan["Start"] == "demand"
    assert plan["Recovery"] == "restart/5000/restart/30000/none/0"
    assert plan["NonCrashRecovery"] is False


@pytest.mark.parametrize("as_bytes", [False, True])
def test_plan_phase_projection_complete_and_partial(as_bytes):
    lines = [f"nomad_plan_phase:{phase}:{index}" for index, phase in enumerate(PLAN_PHASES)]
    for count in range(6):
        output = "PRIVATE_SENTINEL\n" + "\n".join(lines[:count])
        facts = plan_phase_facts(output.encode() if as_bytes else output)
        assert len(facts["phases"]) == count and facts["rejected_candidates"] == 0
        assert facts["phase_status"] == ("complete" if count == 5 else "partial" if count else "missing")
        assert "PRIVATE_SENTINEL" not in json.dumps(facts)
    assert plan_phase_facts(None)["phase_status"] == "missing"


@pytest.mark.parametrize(
    "output",
    [
        "nomad_plan_phase:PRIVATE_SENTINEL:0",
        "nomad_plan_phase:entry:PRIVATE_SENTINEL",
        "nomad_plan_phase:entry:-1",
        "nomad_plan_phase:entry:60001",
        "nomad_plan_phase:entry:0\nnomad_plan_phase:entry:0",
        "nomad_plan_phase:entry:2\nnomad_plan_phase:before_helper:1",
        "nomad_plan_phase:before_helper:0",
    ],
)
def test_plan_phase_projection_rejects_invalid_order_values_and_duplicates(output):
    facts = plan_phase_facts(output)
    assert facts["phase_status"] == "invalid" and facts["rejected_candidates"] > 0
    assert "PRIVATE_SENTINEL" not in json.dumps(facts)


def test_plan_timeout_preserves_limit_and_projects_without_exception_chain(monkeypatch, capsys):
    calls = []

    def timeout(command, **options):
        calls.append((command, options))
        raise subprocess.TimeoutExpired(
            ["PRIVATE_SENTINEL"], 10, output=b"PRIVATE_SENTINEL", stderr=b"nomad_plan_phase:entry:0\nPRIVATE_SENTINEL"
        )

    monkeypatch.setattr(subprocess, "run", timeout)
    with pytest.raises(pytest.fail.Exception) as failure:
        run_windows_plan(Path("PRIVATE_SENTINEL"))
    assert len(calls) == 1 and calls[0][0][:3] == ["powershell", "-NoProfile", "-Command"]
    assert calls[0][1] == {"capture_output": True, "text": True, "timeout": 10}
    assert failure.value.__context__ is None
    captured = capsys.readouterr()
    facts = json.loads(captured.out)
    assert facts["result"] == "timeout" and facts["phase_status"] == "partial"
    assert facts["phases"] == [{"phase": "entry", "elapsed_ms": 0}]
    assert "PRIVATE_SENTINEL" not in captured.out + captured.err + str(failure.value)


@pytest.mark.parametrize(
    "invalid", [None, "returncode", "BinaryPath", "Account", "Start", "Recovery", "NonCrashRecovery"]
)
def test_plan_instrumentation_retains_each_original_assertion(monkeypatch, capsys, invalid):
    plan = {
        "BinaryPath": (
            '"C:\\NOMAD release\\bin\\nomad-runtime.exe" --service --config "C:\\ProgramData\\NOMAD\\runtime.json"'
        ),
        "Account": "NT AUTHORITY\\LocalService",
        "Start": "demand",
        "Recovery": "restart/5000/restart/30000/none/0",
        "NonCrashRecovery": False,
    }
    if invalid not in (None, "returncode"):
        plan[invalid] = None
    stderr = "\n".join(f"nomad_plan_phase:{phase}:{index}" for index, phase in enumerate(PLAN_PHASES))

    def completed(command, **options):
        assert command[:3] == ["powershell", "-NoProfile", "-Command"] and options["timeout"] == 10
        return subprocess.CompletedProcess(command, 1 if invalid == "returncode" else 0, json.dumps(plan), stderr)

    monkeypatch.setattr(subprocess, "run", completed)
    if invalid is None:
        test_windows_plan_quotes_paths_and_bounds_recovery()
    else:
        with pytest.raises(AssertionError):
            test_windows_plan_quotes_paths_and_bounds_recovery()
    facts = json.loads(capsys.readouterr().out)
    assert facts["result"] == "returned" and facts["phase_status"] == "complete"


@pytest.mark.skipif(os.name != "nt", reason="Windows child failure status")
def test_plan_phase_commands_cannot_hide_helper_failure(tmp_path, capsys):
    script = tmp_path / "synthetic-plan.ps1"
    script.write_text("throw 'PRIVATE_SENTINEL'\n", encoding="utf-8")
    result = run_windows_plan(script)
    assert result.returncode != 0
    facts = json.loads(capsys.readouterr().out)
    assert facts["result"] == "returned" and facts["phase_status"] == "partial"
    assert [item["phase"] for item in facts["phases"]] == ["entry", "before_helper"]
    assert "PRIVATE_SENTINEL" not in json.dumps(facts) + result.stdout + result.stderr


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
