# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Regression tests for numerical ceilings on existing size debt."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.dev import line_report, source_size_caps


def _write_caps(root: Path, files: dict[str, int], functions: dict[str, int]) -> Path:
    path = root / "config/source_size_caps.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "baseline_commit": "bc7cf4e36690e0561a7af9370bdf46e4f9e91443",
                "measurement": "Tracked checkout at the audit commit.",
                "file_line_caps": {
                    name: {"max_lines": lines, "reason": "measured existing debt"} for name, lines in files.items()
                },
                "function_line_caps": {
                    name: {"max_lines": lines, "reason": "measured existing debt"} for name, lines in functions.items()
                },
            }
        ),
        encoding="utf-8",
    )
    return path.relative_to(root)


def _args(caps_path: Path, **overrides) -> SimpleNamespace:
    values = {
        "max_file_lines": 500,
        "max_function_lines": 40,
        "max_line_length": 120,
        "baseline_file": Path("config/file_size_baseline.txt"),
        "baseline_function_file": Path("config/function_size_baseline.txt"),
        "baseline_line_file": Path("config/line_length_baseline.txt"),
        "size_caps_file": caps_path,
        "fail_over_file_limit": False,
        "fail_over_function_limit": False,
        "fail_line_length": False,
        "fail_stale_baseline": False,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def _write(root: Path, relative: str, text: str) -> None:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def test_file_cap_rejects_growth_beyond_measured_baseline(tmp_path: Path):
    _write(tmp_path, "config/file_size_baseline.txt", "src/debt.cpp\n")
    _write(tmp_path, "config/function_size_baseline.txt", "")
    caps_path = _write_caps(tmp_path, {"src/debt.cpp": 501}, {})
    report = line_report.Report([line_report.FileSize(Path("src/debt.cpp"), 502)], [], [])

    result = line_report.enforce_report(report, _args(caps_path, fail_over_file_limit=True), tmp_path)

    assert result == 1


def test_function_cap_rejects_growth_beyond_measured_baseline(tmp_path: Path):
    _write(tmp_path, "config/file_size_baseline.txt", "")
    _write(tmp_path, "config/function_size_baseline.txt", "scripts/tool.py:big\n")
    caps_path = _write_caps(tmp_path, {}, {"scripts/tool.py:big": 41})
    report = line_report.Report([], [], [line_report.FunctionSize(Path("scripts/tool.py"), "big", 2, 42)])

    result = line_report.enforce_report(report, _args(caps_path, fail_over_function_limit=True), tmp_path)

    assert result == 1


def test_growth_check_allows_shrinking_but_still_oversized_debt():
    caps = {
        "file_line_caps": {"src/debt.cpp": {"max_lines": 510}},
        "function_line_caps": {"src/tool.py:big": {"max_lines": 45}},
    }

    messages = source_size_caps.find_growth_violations(
        [("src/debt.cpp", 505)],
        [("src/tool.py:big", 42)],
        {"src/debt.cpp"},
        {"src/tool.py:big"},
        caps,
    )

    assert messages == []


def test_cap_coverage_reports_missing_and_orphaned_entries():
    caps = {
        "file_line_caps": {"src/orphan.cpp": {"max_lines": 501}},
        "function_line_caps": {"src/tool.py:renamed": {"max_lines": 41}},
    }

    messages = source_size_caps.find_coverage_errors({"src/no-cap.cpp"}, {"src/tool.py:old_name"}, caps)

    assert "src/no-cap.cpp: missing numerical file cap" in messages
    assert "src/orphan.cpp: numerical file cap has no size-baseline entry" in messages
    assert "src/tool.py:old_name: missing numerical function cap" in messages
    assert "src/tool.py:renamed: numerical function cap has no size-baseline entry" in messages


@pytest.mark.parametrize(
    "entry",
    [
        {"max_lines": True, "reason": "bool is not a measured line count"},
        {"max_lines": 10, "reason": ""},
    ],
)
def test_invalid_cap_entries_are_rejected(tmp_path: Path, entry: dict[str, object]):
    path = _write_caps(tmp_path, {}, {})
    target = tmp_path / path
    data = json.loads(target.read_text(encoding="utf-8"))
    data["file_line_caps"]["src/debt.cpp"] = entry
    target.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(ValueError, match="invalid numerical cap entry"):
        source_size_caps.read_size_caps(path, tmp_path)


def test_windows_traversal_cap_paths_are_rejected(tmp_path: Path):
    path = _write_caps(tmp_path, {}, {})
    target = tmp_path / path
    data = json.loads(target.read_text(encoding="utf-8"))
    data["file_line_caps"][r"..\outside.cpp"] = {
        "max_lines": 500,
        "reason": "bad path",
    }
    target.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(ValueError, match="invalid relative path"):
        source_size_caps.read_size_caps(path, tmp_path)


@pytest.mark.parametrize("name", ["C:/outside.cpp", "src/./debt.cpp", "src//debt.cpp"])
def test_drive_qualified_and_noncanonical_cap_paths_are_rejected(tmp_path: Path, name: str):
    path = _write_caps(tmp_path, {}, {})
    target = tmp_path / path
    data = json.loads(target.read_text(encoding="utf-8"))
    data["file_line_caps"][name] = {"max_lines": 500, "reason": "bad path"}
    target.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(ValueError, match="invalid relative path"):
        source_size_caps.read_size_caps(path, tmp_path)


def test_boolean_schema_version_is_rejected(tmp_path: Path):
    path = _write_caps(tmp_path, {}, {})
    target = tmp_path / path
    data = json.loads(target.read_text(encoding="utf-8"))
    data["schema_version"] = True
    target.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(ValueError, match="unsupported source-size caps schema"):
        source_size_caps.read_size_caps(path, tmp_path)


def test_missing_baseline_identity_is_rejected(tmp_path: Path):
    path = _write_caps(tmp_path, {}, {})
    target = tmp_path / path
    data = json.loads(target.read_text(encoding="utf-8"))
    del data["baseline_commit"]
    target.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(ValueError, match="invalid source-size caps baseline commit"):
        source_size_caps.read_size_caps(path, tmp_path)
