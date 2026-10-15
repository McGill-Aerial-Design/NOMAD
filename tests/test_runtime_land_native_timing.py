# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Native append facts stay bounded, associated, private and post-exit only."""

import importlib
import io
import json
import os
import runpy
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml


@pytest.fixture
def modules(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts/dev"))
    return importlib.import_module("runtime_land_fixture"), importlib.import_module("runtime_fixture_support")


def native_record():
    return {
        "event": "runtime_land_native_timing",
        "capacity": 8,
        "overflow": False,
        "records": [
            {
                "journal_event": event,
                "ordinal": ordinal,
                "lock_acquired_us": 1,
                "before_native_us": 2,
                "after_native_us": 3,
                "append_finish_us": 4,
                "native_success": True,
                "append_success": True,
            }
            for ordinal, event in ((6, "mutation_intent"), (7, "mutation_outcome"))
        ],
    }


def expected_rows():
    return [{"ordinal": 6, "event": "mutation_intent"}, {"ordinal": 7, "event": "mutation_outcome"}]


def read(land, record, exit_code=0, expected=None):
    payload = json.dumps(record).encode() + b"\n"
    return land.read_native_timing(io.BytesIO(payload), exit_code, expected_rows() if expected is None else expected)


def test_complete_native_pair_requires_confirmed_exit_and_durable_ordinal_association(modules):
    land, _ = modules
    result = read(land, native_record())
    assert result["status"] == "complete" and result["journal_match"] is True
    assert len(result["records"]) == 2 and result["rejected_candidates"] == 0
    for exit_code in (None, 74, False):
        assert read(land, native_record(), exit_code)["status"] == "partial"
    assert read(land, native_record(), expected=[{"ordinal": 5, "event": "mutation_intent"}])["status"] == "partial"
    assert land.read_native_timing(io.BytesIO(json.dumps(native_record()).encode()), 0)["status"] == "partial"


@pytest.mark.parametrize(
    "mutate",
    [
        lambda row: row.update(extra="PRIVATE_SENTINEL"),
        lambda row: row.update(capacity=True),
        lambda row: row.update(overflow="PRIVATE_SENTINEL"),
        lambda row: row["records"][0].update(path="PRIVATE_SENTINEL"),
        lambda row: row["records"][0].update(journal_event="PRIVATE_SENTINEL"),
        lambda row: row["records"][0].update(ordinal=True),
        lambda row: row["records"][0].update(ordinal=6.0),
        lambda row: row["records"][0].update(lock_acquired_us=-1),
        lambda row: row["records"][0].update(lock_acquired_us=True),
        lambda row: row["records"][0].update(before_native_us=5),
        lambda row: row["records"][0].update(after_native_us=None),
        lambda row: row["records"][0].update(native_success="PRIVATE_SENTINEL"),
        lambda row: row["records"][0].update(native_success=False),
        lambda row: row["records"][0].update(append_success="PRIVATE_SENTINEL"),
        lambda row: row["records"][1].update(ordinal=6),
    ],
)
def test_unknown_invalid_or_secret_values_reject_entire_candidate(modules, mutate):
    land, _ = modules
    record = native_record()
    mutate(record)
    result = read(land, record)
    assert result["status"] == "partial" and result["records"] == []
    assert "PRIVATE_SENTINEL" not in json.dumps(result)


def test_missing_truncated_duplicate_and_overflow_never_claim_complete(modules):
    land, _ = modules
    assert land.read_native_timing(io.BytesIO(b"unrelated PRIVATE_SENTINEL\n"), 0)["status"] == "unavailable"
    truncated = b'{"event":"runtime_land_native_timing"'
    assert land.read_native_timing(io.BytesIO(truncated), 0)["status"] == "partial"
    line = json.dumps(native_record()).encode() + b"\n"
    assert land.read_native_timing(io.BytesIO(line + line), 0, expected_rows())["records"] == []
    record = native_record()
    record["overflow"] = True
    assert read(land, record)["status"] == "partial"
    record["records"] = []
    assert read(land, record)["status"] == "partial"


@pytest.mark.parametrize(
    "original,replacement",
    [
        ('"overflow": false', '"overflow": true, "overflow": false'),
        ('"ordinal": 6', '"ordinal": 0, "ordinal": 6'),
    ],
)
def test_duplicate_json_keys_are_rejected_without_silent_overwrite(modules, original, replacement):
    land, _ = modules
    line = json.dumps(native_record()).replace(original, replacement)
    result = land.read_native_timing(io.BytesIO(line.encode()), 0, expected_rows())
    assert result["status"] == "partial" and result["records"] == []


def test_guard_and_native_failures_retain_null_or_actual_return_facts(modules):
    land, _ = modules
    for attempted in (False, True):
        record = native_record()
        record["records"] = record["records"][:1]
        record["records"][0].update(
            append_success=False,
            before_native_us=2 if attempted else None,
            after_native_us=3 if attempted else None,
            native_success=False if attempted else None,
        )
        result = read(land, record, exit_code=74)
        assert result["status"] == "partial" and len(result["records"]) == 1
        assert result["records"][0]["native_success"] is (False if attempted else None)


@pytest.mark.parametrize("enabled", [False, True])
def test_child_environment_clears_inherited_probe_and_only_explicitly_opts_in(modules, monkeypatch, tmp_path, enabled):
    _, support = modules
    captured = []
    monkeypatch.setenv("NOMAD_QUALIFICATION_AUDIT_TIMING", "1")
    storage = SimpleNamespace(name=str(tmp_path))
    monkeypatch.setattr(support.tempfile, "TemporaryDirectory", lambda **_: storage)
    monkeypatch.setattr(support, "write_fixture_credentials", lambda _: tmp_path / "synthetic")

    def popen(*_a, **kwargs):
        captured.append(kwargs["env"].get("NOMAD_QUALIFICATION_AUDIT_TIMING"))
        return SimpleNamespace()

    monkeypatch.setattr(support.subprocess, "Popen", popen)
    _, stdout, stderr = support.start_runtime(Path("synthetic"), 1, 2, audit_timing=enabled)
    stdout.close()
    stderr.close()
    assert captured == ["1" if enabled else None]


@pytest.mark.parametrize("flag", [None, "", "0", "true", "1x", "1"])
def test_actual_native_environment_and_last_owner_delivery(modules, flag):
    land, support = modules
    try:
        binary = support.find_runtime().with_name(
            "nomad_runtime_tests.exe" if os.name == "nt" else "nomad_runtime_tests"
        )
    except FileNotFoundError:
        pytest.skip("native runtime build unavailable")
    if not binary.is_file():
        pytest.skip("native runtime test binary unavailable")
    environment = dict(os.environ)
    environment.pop("NOMAD_QUALIFICATION_AUDIT_TIMING", None)
    if flag is not None:
        environment["NOMAD_QUALIFICATION_AUDIT_TIMING"] = flag
    result = subprocess.run(
        [str(binary), "--native-audit-environment"], env=environment, capture_output=True, timeout=10
    )
    assert result.returncode == 0
    lines = result.stderr.splitlines()
    before = lines.index(b"native_probe_before_final_release")
    after = lines.index(b"native_probe_after_final_release")
    candidates = [(index, json.loads(line)) for index, line in enumerate(lines) if line.startswith(b'{"capacity"')]
    assert len(candidates) == (1 if flag == "1" else 0)
    if flag == "1":
        assert before < candidates[0][0] < after
        assert land.project_native_timing(candidates[0][1]) is not None


def test_actual_land_entry_sanitizes_timeout_command_contents(modules, monkeypatch, capsys):
    _, support = modules
    peer_module = importlib.import_module("mavsdk_peer")

    class Peer:
        def __init__(self, *_a, **_kw):
            pass

        def start(self):
            pass

    def fail():
        raise subprocess.TimeoutExpired(["PRIVATE_SENTINEL", "udpin:private:12345"], 12)

    monkeypatch.setattr(peer_module, "VehiclePeer", Peer)
    monkeypatch.setattr(support, "find_runtime", fail)
    script = Path(__file__).resolve().parents[1] / "scripts/dev/runtime_land_fixture.py"
    with pytest.raises(SystemExit) as caught:
        runpy.run_path(str(script), run_name="__main__")
    captured = capsys.readouterr()
    assert caught.value.code == 1 and captured.err.strip() == "runtime LAND qualification failed"
    assert "PRIVATE_SENTINEL" not in captured.out + captured.err and "12345" not in captured.out + captured.err


def test_core_workflow_executes_native_controls_on_both_platforms():
    root = Path(__file__).resolve().parents[1]
    core = yaml.safe_load((root / ".github/workflows/test.yml").read_text(encoding="utf-8"))["jobs"]["core"]
    assert core["strategy"]["matrix"]["os"] == ["ubuntu-latest", "windows-latest"]
    step = next(
        step
        for step in core["steps"]
        if step.get("name") == "Qualify installed CLI parsing and LAND outcomes with built binaries"
    )
    assert (
        step["run"]
        == "pixi run python -m pytest tests/test_core_client_contract.py tests/test_runtime_land_native_timing.py -q"
    )
    assert "if" not in step and "continue-on-error" not in step
