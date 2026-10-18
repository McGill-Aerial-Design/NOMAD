# SPDX-License-Identifier: Apache-2.0
"""Version preflight failures retain bounded facts without leaking child details."""

import hashlib
import importlib
import json
import os
import subprocess
import sys
import traceback
from pathlib import Path

import pytest

from scripts.release.lifecycle import Deployment


@pytest.fixture
def probe(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts/dev"))
    module = importlib.import_module("release_process_fixture")
    directory = tmp_path / "PRIVATE_DEPLOYMENT"
    directory.mkdir()
    config = directory / "config.json"
    config.write_text("synthetic configuration", encoding="utf-8")
    binary = directory / ("nomad-runtime.exe" if os.name == "nt" else "nomad-runtime")
    binary.write_bytes(b"synthetic executable bytes")
    adapter = module.RuntimeProcess(config, 1, directory)
    candidate = {"install_path": str(directory), "source_sha": "a" * 40}
    try:
        yield module, adapter, candidate, binary
    finally:
        adapter.close()


@pytest.mark.parametrize("partial", [None, b"\xffPRIVATE_CHILD_OUTPUT", b"0.0.0-fixture.A\nPRIVATE_CHILD_OUTPUT"])
def test_timeout_is_private_and_retains_closed_failure_facts(probe, monkeypatch, capsys, partial):
    module, adapter, candidate, binary = probe
    calls = []
    stderr = b"PRIVATE_CHILD_ERROR\x1b[31m"

    def timeout(command, **options):
        calls.append((command, options))
        raise subprocess.TimeoutExpired(command, options["timeout"], output=partial, stderr=stderr)

    monkeypatch.setattr(module.subprocess, "run", timeout)
    with pytest.raises(subprocess.TimeoutExpired) as caught:
        adapter.preflight(candidate)
    formatted = "".join(traceback.format_exception(caught.value))
    assert "PRIVATE_DEPLOYMENT" not in formatted and "PRIVATE_CHILD" not in formatted
    assert caught.value.cmd == ["<candidate-runtime>", "--version"]
    assert caught.value.timeout == 10 and caught.value.output is None and caught.value.stderr is None
    assert not isinstance(caught.value, RuntimeError)
    assert len(calls) == 1 and calls[0] == ([str(binary), "--version"], {"capture_output": True, "timeout": 10})
    captured = capsys.readouterr()
    record = json.loads(captured.err)
    assert set(record) == {
        "scope",
        "phase",
        "timeout_seconds",
        "elapsed_ms",
        "image_sha256",
        "stdout_bytes",
        "stdout_sha256",
        "stderr_bytes",
        "stderr_sha256",
        "expected_version_observed",
    }
    assert record["scope"] == "release-version-probe" and record["phase"] == "timeout"
    assert record["timeout_seconds"] == 10 and record["elapsed_ms"] >= 0
    assert record["image_sha256"] == hashlib.sha256(binary.read_bytes()).hexdigest()
    output = partial or b""
    assert record["stdout_bytes"] == len(output) and record["stdout_sha256"] == hashlib.sha256(output).hexdigest()
    assert record["stderr_bytes"] == len(stderr) and record["stderr_sha256"] == hashlib.sha256(stderr).hexdigest()
    assert record["expected_version_observed"] is (b"0.0.0-fixture.A" in output)
    assert "PRIVATE" not in captured.out + captured.err and "\x1b" not in captured.err
    assert adapter.child is None and adapter.candidate is None


@pytest.mark.parametrize("source,label", [("a", "A"), ("b", "B")])
def test_success_preserves_fixture_version_check_without_diagnostic_output(probe, monkeypatch, capsys, source, label):
    module, adapter, candidate, _ = probe
    candidate["source_sha"] = source * 40
    calls = []

    def success(command, **options):
        calls.append(options)
        return subprocess.CompletedProcess(command, 0, ("0.0.0-fixture." + label).encode(), b"")

    monkeypatch.setattr(module.subprocess, "run", success)
    adapter.preflight(candidate)
    assert calls == [{"capture_output": True, "timeout": 10}]
    assert capsys.readouterr().err == ""


@pytest.mark.parametrize("returncode,output", [(3, b"0.0.0-fixture.A"), (0, b"0.0.0-fixture.B")])
def test_failed_or_wrong_version_is_still_rejected(probe, monkeypatch, returncode, output):
    module, adapter, candidate, _ = probe
    monkeypatch.setattr(
        module.subprocess, "run", lambda command, **_: subprocess.CompletedProcess(command, returncode, output, b"")
    )
    with pytest.raises(AssertionError):
        adapter.preflight(candidate)
    assert adapter.child is None and adapter.candidate is None


@pytest.mark.parametrize("failure", [OSError, ValueError, RuntimeError])
def test_diagnostic_delivery_failure_does_not_mask_timeout(probe, monkeypatch, failure):
    module, adapter, candidate, _ = probe

    def timeout(command, **options):
        raise subprocess.TimeoutExpired(command, options["timeout"])

    def reject_diagnostic(*_args, **_options):
        raise failure("synthetic diagnostic failure")

    monkeypatch.setattr(module.subprocess, "run", timeout)
    monkeypatch.setattr(module, "print", reject_diagnostic, raising=False)
    with pytest.raises(subprocess.TimeoutExpired) as caught:
        adapter.preflight(candidate)
    assert caught.value.cmd == ["<candidate-runtime>", "--version"] and caught.value.timeout == 10


def test_timeout_precedes_transaction_or_process_mutation(probe, monkeypatch, tmp_path):
    module, adapter, candidate, _ = probe
    deployment = Deployment(tmp_path / "deployment", "core")
    state = deployment.status()
    before = dict(state)
    phases = []

    def timeout(command, **options):
        raise subprocess.TimeoutExpired(command, options["timeout"])

    with monkeypatch.context() as changes:
        changes.setattr(module.subprocess, "run", timeout)
        for phase in ["stop", "switch", "start", "health"]:
            changes.setattr(adapter, phase, lambda *_args, name=phase: phases.append(name))
        with pytest.raises(subprocess.TimeoutExpired):
            deployment.transition(state, candidate, adapter, "active")
    assert state == before and deployment.status() == before and not deployment.record.exists()
    assert phases == [] and adapter.child is None and adapter.candidate is None


def test_real_timed_child_is_retired_and_its_partial_output_is_private(probe, monkeypatch, capsys):
    module, adapter, candidate, _ = probe
    run, popen = subprocess.run, subprocess.Popen
    children, failures = [], []
    monkeypatch.setattr(adapter, "executable", lambda _: Path(sys.executable))

    def start(*args, **options):
        child = popen(*args, **options)
        children.append(child)
        return child

    def blocked_version(command, **options):
        assert command == [sys.executable, "--version"] and options == {"capture_output": True, "timeout": 10}
        script = "import time; print('0.0.0-fixture.A PRIVATE_CHILD_OUTPUT', flush=True); time.sleep(30)"
        try:
            return run([command[0], "-c", script], **options)
        except subprocess.TimeoutExpired as error:
            failures.append(error)
            raise

    monkeypatch.setattr(module.subprocess, "Popen", start)
    monkeypatch.setattr(module.subprocess, "run", blocked_version)
    with pytest.raises(subprocess.TimeoutExpired) as caught:
        adapter.preflight(candidate)
    assert len(children) == len(failures) == 1 and children[0].poll() is not None
    assert caught.value.cmd == ["<candidate-runtime>", "--version"] and caught.value.timeout == 10
    assert caught.value.output is None and caught.value.stderr is None
    captured = capsys.readouterr()
    assert "PRIVATE_CHILD_OUTPUT" not in captured.out + captured.err + str(caught.value)
    record = json.loads(captured.err)
    partial = failures[0].output or b""
    assert record["stdout_bytes"] == len(partial)
    assert record["stdout_sha256"] == hashlib.sha256(partial).hexdigest()
    assert record["image_sha256"] == hashlib.sha256(Path(sys.executable).read_bytes()).hexdigest()
