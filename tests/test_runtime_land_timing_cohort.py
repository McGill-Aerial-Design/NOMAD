# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""The bounded LAND cohort preserves the existing case and stops at its first failure."""

import importlib
import json
import runpy
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def cohort(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "scripts/dev"))
    return importlib.import_module("runtime_land_timing_cohort")


def test_exact_twenty_original_case_calls_have_immediate_ordered_markers(cohort, monkeypatch):
    events = []
    monkeypatch.setattr(
        cohort, "print", lambda text, *, flush: events.append(("marker", json.loads(text), flush)), raising=False
    )

    def case(*args, **kwargs):
        sample = len(events) // 2 + 1
        assert events[-1] == ("marker", {"event": "runtime_land_timing_sample", "sample": sample}, True)
        events.append(("case", args, kwargs))

    monkeypatch.setattr(cohort, "run_case", case)
    assert cohort.main() == 0
    assert len(events) == 41
    for sample in range(20):
        assert events[sample * 2 + 1] == ("case", ("accepted-without-mode", "unknown", True), {"mode_delay": None})
    assert events[-1] == ("marker", {"event": "runtime_land_timing_cohort", "result": "passed", "samples": 20}, True)


def test_third_case_failure_preserves_same_exception_and_never_starts_fourth(cohort, monkeypatch, capsys):
    failure = RuntimeError("PRIVATE_SENTINEL")
    calls = []

    def case(*args, **kwargs):
        calls.append((args, kwargs))
        print(json.dumps({"event": "runtime_land_stage_timing", "checks": "failed" if len(calls) == 3 else "passed"}))
        if len(calls) == 3:
            raise failure

    monkeypatch.setattr(cohort, "run_case", case)
    with pytest.raises(RuntimeError) as caught:
        cohort.main()
    assert caught.value is failure and len(calls) == 3
    text = capsys.readouterr().out
    events = [json.loads(line) for line in text.splitlines()]
    assert [row["sample"] for row in events if row["event"] == "runtime_land_timing_sample"] == [1, 2, 3]
    assert [row["event"] for row in events] == ["runtime_land_timing_sample", "runtime_land_stage_timing"] * 3
    assert not any(row["event"] == "runtime_land_timing_cohort" for row in events)
    assert "PRIVATE_SENTINEL" not in text


def test_real_entry_reports_failed_exit_without_exception_contents(cohort, monkeypatch, capsys):
    fixture = importlib.import_module("runtime_land_fixture")
    calls = []

    def fail(*args, **kwargs):
        calls.append((args, kwargs))
        raise RuntimeError("PRIVATE_SENTINEL credential/proof/path/temporary-identity")

    monkeypatch.setattr(fixture, "run_case", fail)
    with pytest.raises(SystemExit) as caught:
        runpy.run_path(str(ROOT / "scripts/dev/runtime_land_timing_cohort.py"), run_name="__main__")
    assert caught.value.code == 1 and len(calls) == 1
    captured = capsys.readouterr()
    assert json.loads(captured.out) == {"event": "runtime_land_timing_sample", "sample": 1}
    assert json.loads(captured.err) == {"event": "runtime_land_timing_cohort", "result": "failed"}
    assert "PRIVATE_SENTINEL" not in captured.out + captured.err


def test_workflow_keeps_original_seven_cases_then_cohort_then_lifecycle():
    workflow = yaml.safe_load((ROOT / ".github/workflows/test.yml").read_text(encoding="utf-8"))
    core = workflow["jobs"]["core"]
    assert core["strategy"]["matrix"]["os"] == ["ubuntu-latest", "windows-latest"]
    steps = core["steps"]
    index = next(
        index
        for index, step in enumerate(steps)
        if step.get("run") == "pixi run python scripts/dev/runtime_land_fixture.py"
    )
    original, cohort, lifecycle = steps[index : index + 3]
    assert original == {
        "name": "Qualify bounded Copter LAND at a one-hertz MAVLink peer",
        "run": "pixi run python scripts/dev/runtime_land_fixture.py",
    }
    assert cohort == {
        "name": "Observe twenty fresh accepted-without-mode LAND cases",
        "run": "pixi run python scripts/dev/runtime_land_timing_cohort.py",
    }
    assert lifecycle["run"] == "pixi run python scripts/dev/runtime_lifecycle_qualification.py"
    assert all("continue-on-error" not in step and "if" not in step for step in (original, cohort, lifecycle))
