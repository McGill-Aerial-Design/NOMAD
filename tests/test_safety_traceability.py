# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Traceability CI gate (DO-178C §6 bidirectional traceability).

Parses the normative ``traceability`` code block in
``docs/safety/traceability.md`` and asserts, for every requirement mapping, that
the referenced code symbol imports and the referenced test exists. This is what
keeps the requirement -> code -> test matrix honest: rename or delete a mapped
symbol or test without updating the table and this test fails.
"""

from __future__ import annotations

import ast
import importlib
import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
TRACEABILITY_DOC = REPO_ROOT / "docs" / "safety" / "traceability.md"

_BLOCK_RE = re.compile(r"```traceability\n(.*?)```", re.DOTALL)


def _parse_mappings() -> list[tuple[str, str, str]]:
    """Return (requirement, symbol_ref, test_ref) rows from the normative block."""
    text = TRACEABILITY_DOC.read_text(encoding="utf-8")
    match = _BLOCK_RE.search(text)
    assert match, "no ```traceability``` block found in docs/safety/traceability.md"
    rows: list[tuple[str, str, str]] = []
    for line in match.group(1).splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [p.strip() for p in line.split("|")]
        assert len(parts) == 3, f"malformed traceability row (expected 3 fields): {line!r}"
        rows.append((parts[0], parts[1], parts[2]))
    return rows


MAPPINGS = _parse_mappings()


def test_block_is_non_empty():
    assert MAPPINGS, "traceability block parsed to zero rows"


def test_requirement_ids_are_unique_and_well_formed():
    ids = [req for req, _, _ in MAPPINGS]
    assert len(ids) == len(set(ids)), f"duplicate requirement IDs: {ids}"
    for req in ids:
        assert re.fullmatch(r"SR-[A-Z]+-\d+", req), f"malformed requirement id: {req!r}"


@pytest.mark.parametrize("req,symbol_ref,_test_ref", MAPPINGS, ids=[m[0] for m in MAPPINGS])
def test_code_symbol_resolves(req: str, symbol_ref: str, _test_ref: str):
    """Every mapped code symbol must import (module:Dotted.Attr)."""
    module_path, _, dotted = symbol_ref.partition(":")
    assert dotted, f"{req}: symbol ref must be 'module:Symbol', got {symbol_ref!r}"
    module = importlib.import_module(module_path)
    obj = module
    for attr in dotted.split("."):
        assert hasattr(obj, attr), f"{req}: {symbol_ref} - '{attr}' not found on {obj!r}"
        obj = getattr(obj, attr)


@pytest.mark.parametrize("req,_symbol_ref,test_ref", MAPPINGS, ids=[m[0] for m in MAPPINGS])
def test_proving_test_exists(req: str, _symbol_ref: str, test_ref: str):
    """Every mapped test must exist as a def in the referenced file."""
    file_part, sep, test_name = test_ref.partition("::")
    assert sep and test_name, f"{req}: test ref must be 'file::test_name', got {test_ref!r}"
    test_file = REPO_ROOT / file_part
    assert test_file.is_file(), f"{req}: test file not found: {file_part}"
    tree = ast.parse(test_file.read_text(encoding="utf-8"), filename=str(test_file))
    defined = {node.name for node in ast.walk(tree) if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))}
    assert test_name in defined, f"{req}: test '{test_name}' not defined in {file_part}"
