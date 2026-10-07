# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Validate and apply numerical ceilings for existing source-size debt."""

from __future__ import annotations

import json
import re
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any


def read_size_caps(path: Path | None, root: Path) -> dict[str, dict[str, dict[str, Any]]]:
    """Read a versioned cap file and reject malformed or unsafe entries."""
    if path is None:
        raise ValueError("a numerical source-size caps file is required for this check")
    selected = path if path.is_absolute() else root / path
    if not selected.is_file():
        raise ValueError(f"numerical source-size caps file does not exist: {path}")
    data = json.loads(selected.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or type(data.get("schema_version")) is not int or data["schema_version"] != 1:
        raise ValueError(f"unsupported source-size caps schema in {path}")
    if not re.fullmatch(r"[0-9a-f]{40}", str(data.get("baseline_commit", ""))):
        raise ValueError(f"invalid source-size caps baseline commit in {path}")
    if not isinstance(data.get("measurement"), str) or not data["measurement"].strip():
        raise ValueError(f"invalid source-size caps measurement in {path}")
    caps = {
        "file_line_caps": data.get("file_line_caps"),
        "function_line_caps": data.get("function_line_caps"),
    }
    for kind, group in caps.items():
        _validate_group(path, kind, group)
    return caps


def _validate_group(path: Path, kind: str, group: object) -> None:
    if not isinstance(group, dict):
        raise ValueError(f"invalid {kind} mapping in {path}")
    for name, entry in group.items():
        if not isinstance(name, str) or not isinstance(entry, dict):
            raise ValueError(f"invalid numerical cap entry in {path}: {name}")
        if not _valid_relative_name(name, kind == "function_line_caps"):
            raise ValueError(f"invalid relative path in {path}: {name}")
        maximum = entry.get("max_lines")
        reason = entry.get("reason")
        if type(maximum) is not int or maximum <= 0 or not isinstance(reason, str) or not reason.strip():
            raise ValueError(f"invalid numerical cap entry in {path}: {name}")


def _valid_relative_name(name: str, function_entry: bool) -> bool:
    if "\\" in name:
        return False
    source, separator, symbol = name.partition(":")
    if function_entry and (not separator or not symbol.strip() or ":" in symbol):
        return False
    if not function_entry and separator:
        return False
    relative = PurePosixPath(source)
    windows = PureWindowsPath(source)
    return (
        bool(relative.parts)
        and relative.as_posix() == source
        and not relative.is_absolute()
        and not windows.drive
        and not windows.is_absolute()
        and ".." not in relative.parts
        and ".." not in windows.parts
    )


def find_coverage_errors(
    file_baseline: set[str], function_baseline: set[str], caps: dict[str, dict[str, dict[str, Any]]]
) -> list[str]:
    """Ensure every name-only exemption has one numeric cap and no cap is orphaned."""
    messages: list[str] = []
    for name in sorted(file_baseline - caps["file_line_caps"].keys()):
        messages.append(f"{name}: missing numerical file cap")
    for name in sorted(caps["file_line_caps"].keys() - file_baseline):
        messages.append(f"{name}: numerical file cap has no size-baseline entry")
    for name in sorted(function_baseline - caps["function_line_caps"].keys()):
        messages.append(f"{name}: missing numerical function cap")
    for name in sorted(caps["function_line_caps"].keys() - function_baseline):
        messages.append(f"{name}: numerical function cap has no size-baseline entry")
    return messages


def find_growth_violations(
    file_sizes: list[tuple[str, int]],
    function_sizes: list[tuple[str, int]],
    file_baseline: set[str],
    function_baseline: set[str],
    caps: dict[str, dict[str, dict[str, Any]]],
) -> list[str]:
    """Report measured debt that exceeds its reviewed numerical ceiling."""
    messages: list[str] = []
    messages.extend(_find_growth(file_sizes, file_baseline, caps["file_line_caps"], "file"))
    messages.extend(_find_growth(function_sizes, function_baseline, caps["function_line_caps"], "function"))
    return messages


def check_report_caps(
    path: Path | None,
    root: Path,
    file_baseline: set[str],
    function_baseline: set[str],
    file_sizes: list[tuple[str, int]],
    function_sizes: list[tuple[str, int]],
) -> list[str]:
    """Check baseline coverage and the selected report entries against caps."""
    caps = read_size_caps(path, root)
    messages = find_coverage_errors(file_baseline, function_baseline, caps)
    messages.extend(find_growth_violations(file_sizes, function_sizes, file_baseline, function_baseline, caps))
    return messages


def _find_growth(
    measured: list[tuple[str, int]],
    baseline: set[str],
    caps: dict[str, dict[str, Any]],
    kind: str,
) -> list[str]:
    messages = []
    for name, lines in measured:
        if name not in baseline:
            continue
        entry = caps.get(name)
        if entry is None:
            messages.append(f"{name}: missing numerical {kind} cap")
        elif lines > entry["max_lines"]:
            messages.append(f"{name}: {lines} lines exceeds numerical cap {entry['max_lines']}")
    return messages
