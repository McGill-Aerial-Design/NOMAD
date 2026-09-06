# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Check the C++ command surface for forbidden failsafe controls."""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SOURCE_FILES = [
    REPO_ROOT / "src" / "vehicle" / "vehicle.cpp",
    REPO_ROOT / "src" / "main.cpp",
]
FORBIDDEN_TOKENS = ("FAILSAFE", "FS_", "BRD_SAFETY", "DISABLE_SAFETY")


def test_cpp_command_surface_has_no_failsafe_controls():
    for source_file in SOURCE_FILES:
        text = source_file.read_text(encoding="utf-8").upper()
        hits = [token for token in FORBIDDEN_TOKENS if token in text]
        assert not hits, f"{source_file} contains forbidden failsafe token(s): {hits}"
