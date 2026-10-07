# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Check source provenance and references, without claiming rule compliance."""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INVENTORY = ROOT / "docs" / "conops-requirements.md"
ID_PATTERN = re.compile(r"\bAE27-[A-Z][A-Z0-9]*-\d{3}\b")


def read_requirements() -> dict[str, list[str]]:
    requirements: dict[str, list[str]] = {}
    for line in INVENTORY.read_text(encoding="utf-8").splitlines():
        if not line.startswith("| AE27-"):
            continue
        cells = [cell.strip() for cell in line.strip("|").split("|")]
        assert len(cells) == 8, f"Expected eight traceability fields: {line}"
        requirement = cells[0]
        assert ID_PATTERN.fullmatch(requirement), f"Invalid ID: {requirement}"
        assert requirement not in requirements, f"Duplicate ID: {requirement}"
        assert all(cells), f"Empty field: {requirement}"
        requirements[requirement] = cells
    assert requirements, "Source requirement inventory is empty"
    return requirements


def test_requirements_have_source_status_owner_and_evidence():
    for requirement, cells in read_requirements().items():
        assert re.search(r"p\d", cells[1]), f"Missing source page: {requirement}"
        assert "/" in cells[2], f"Missing task/phase: {requirement}"
        assert re.fullmatch(r"[MSPER]/[CIU]", cells[3]), f"Invalid kind/status: {requirement}"
        assert cells[6] in {"H", "M", "L"}, f"Invalid safety relevance: {requirement}"
        assert re.search(r"\bG(?:[0-8]|-M)\b", cells[7]), f"Missing evidence gate: {requirement}"
        pages = re.findall(r"p(\d+)(?:-(\d+))?", cells[1])
        for first, last in pages:
            assert 1 <= int(first) <= int(last or first) <= 36, f"Invalid source page: {requirement}"


def test_canonical_requirement_references_resolve():
    requirements = read_requirements()
    documents = [*sorted((ROOT / "docs").glob("*.md")), ROOT / "PLAN.md", ROOT / "TODO.md"]
    for document in documents:
        for requirement in ID_PATTERN.findall(document.read_text(encoding="utf-8")):
            assert requirement in requirements, f"Unknown requirement {requirement} in {document.name}"


def test_unresolved_requirements_name_a_question():
    prd = (ROOT / "docs" / "prd.md").read_text(encoding="utf-8")
    questions = set(re.findall(r"^\| (Q\d{2}) \|", prd, re.MULTILINE))
    for requirement, cells in read_requirements().items():
        if cells[3].endswith("/U"):
            references = set(re.findall(r"\bQ\d{2}\b", cells[7]))
            assert references, f"Unresolved requirement has no question: {requirement}"
            assert references <= questions, f"Unknown question for {requirement}: {references - questions}"


def test_gap_rows_preserve_acceptance_fields_and_single_active_item():
    migration = (ROOT / "docs" / "migration.md").read_text(encoding="utf-8")
    gaps = [line for line in migration.splitlines() if line.startswith("| GAP-")]
    assert gaps, "No implementation gap matrix"
    for line in gaps:
        cells = [cell.strip() for cell in line.strip("|").split("|")]
        assert len(cells) == 8 and all(cells), f"Incomplete gap: {line}"
    ledger = (ROOT / "TODO.md").read_text(encoding="utf-8")
    assert len(re.findall(r"^- \[~\]", ledger, re.MULTILINE)) == 1, "Ledger must have one active item"
