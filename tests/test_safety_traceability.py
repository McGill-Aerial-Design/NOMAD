# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Traceability CI gate for the consolidated safety case.

Parses the machine-checked ``cpp_traceability`` block in ``docs/safety.md`` and
asserts that every mapping resolves to real code and a real test. This keeps
requirement evidence honest after the Python transition block was retired with
the Python safety implementation (C++ cutover, 2026-09-05).
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
TRACEABILITY_DOC = REPO_ROOT / "docs" / "safety.md"

_BLOCK_RE = re.compile(r"```cpp_traceability\n(.*?)```", re.DOTALL)


def _parse_mappings() -> list[tuple[str, str, str]]:
    """Return (requirement, symbol ref, test ref) rows from the C++ block."""
    text = TRACEABILITY_DOC.read_text(encoding="utf-8")
    match = _BLOCK_RE.search(text)
    assert match, "no cpp_traceability block found in docs/safety.md"
    rows: list[tuple[str, str, str]] = []
    for line in match.group(1).splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [p.strip() for p in line.split("|")]
        assert len(parts) == 3, f"malformed traceability row: {line!r}"
        rows.append((parts[0], parts[1], parts[2]))
    return rows


MAPPINGS = _parse_mappings()


def test_block_is_non_empty():
    assert MAPPINGS, "cpp_traceability block parsed to zero rows"


def test_requirement_ids_are_well_formed():
    ids = [req for req, _, _ in MAPPINGS]
    # A requirement may appear on several rows (multiple symbols/tests prove
    # it), so only form is checked here — not uniqueness.
    for req in ids:
        assert re.fullmatch(r"SR-[A-Z]+-\d+", req), f"malformed requirement id: {req!r}"


def _read_ref(reference: str) -> tuple[Path, str]:
    file_part, separator, symbol = reference.partition(":")
    assert separator, f"reference must contain ':', got {reference!r}"
    assert symbol, f"reference has no symbol: {reference!r}"
    source_file = REPO_ROOT / file_part
    assert source_file.is_file(), f"source file not found: {file_part}"
    return source_file, symbol.lstrip(":")


@pytest.mark.parametrize("req,symbol_ref,_test_ref", MAPPINGS, ids=[m[0] for m in MAPPINGS])
def test_cpp_symbol_resolves(req: str, symbol_ref: str, _test_ref: str):
    """Every mapping must name a function present in its source file."""
    source_file, symbol = _read_ref(symbol_ref)
    text = source_file.read_text(encoding="utf-8")
    assert f"{symbol}(" in text or f"{symbol} (" in text, f"{req}: {symbol_ref} not found"


@pytest.mark.parametrize("req,_symbol_ref,test_ref", MAPPINGS, ids=[m[0] for m in MAPPINGS])
def test_cpp_proving_test_exists(req: str, _symbol_ref: str, test_ref: str):
    """Every mapping must name a test function present in its test file."""
    source_file, test_name = _read_ref(test_ref)
    text = source_file.read_text(encoding="utf-8")
    assert f"{test_name}(" in text or f"{test_name} (" in text, f"{req}: {test_ref} not found"
