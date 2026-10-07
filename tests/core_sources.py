# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Locate core C++ sources for gates that must scan the whole tree.

Gates that bind to a hand-maintained file list go blind the moment a source is
split, so they scan every core source instead. Shared here so each gate cannot
drift into checking a different set of files.
"""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SOURCE_DIRS = (REPO_ROOT / "src", REPO_ROOT / "include" / "nomad")
SOURCE_SUFFIXES = {".cpp", ".hpp"}


def core_source_files() -> list[Path]:
    """Return every core C++ source, failing loudly if the tree looks empty."""
    files = sorted(path for directory in SOURCE_DIRS for path in directory.rglob("*") if path.suffix in SOURCE_SUFFIXES)
    assert files, "core source directories are missing or empty"
    return files


def relative(path: Path) -> str:
    """Return a repository-relative POSIX path, for readable test ids."""
    return path.relative_to(REPO_ROOT).as_posix()
