# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Check the C++ command surface for forbidden failsafe controls.

The core must never disable or reconfigure ArduPilot failsafes, so the scan
covers every core source file rather than a hand-maintained list: splitting a
core file must not move code outside the check's reach.
"""

import pytest
from core_sources import core_source_files, relative

FORBIDDEN_TOKENS = ("FAILSAFE", "FS_", "BRD_SAFETY", "DISABLE_SAFETY")


@pytest.mark.parametrize("source_file", core_source_files(), ids=relative)
def test_cpp_command_surface_has_no_failsafe_controls(source_file):
    text = source_file.read_text(encoding="utf-8").upper()
    hits = [token for token in FORBIDDEN_TOKENS if token in text]
    assert not hits, f"{relative(source_file)} contains forbidden failsafe token(s): {hits}"
