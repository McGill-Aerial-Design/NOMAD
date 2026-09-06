# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Pytest wrapper for the SITL velocity loop-closure scenario.

Skipped by default: it requires a live ArduPilot SITL reachable on the
operator MAVLink endpoint (``NOMAD_SITL_OPERATOR``, e.g. ``tcp:127.0.0.1:5762``)
plus a built C++ core binary. See tests/sitl/README.md for how to run it
against the ``pixi run dev-up`` stack. Normal CI (no SITL) collects this as a
skip.
"""

from __future__ import annotations

import os
import sys

import pytest

# Import the sibling scenario module without requiring a package layout.
sys.path.insert(0, os.path.dirname(__file__))
from velocity_loop_closure import find_binary, get_sitl_port, run_scenario  # noqa: E402

_OPERATOR = os.environ.get("NOMAD_SITL_OPERATOR")
_BINARY = find_binary()


@pytest.mark.skipif(
    not (_OPERATOR and _BINARY),
    reason="set NOMAD_SITL_OPERATOR and run `pixi run build-core` to run against a live ArduPilot SITL",
)
def test_velocity_loop_closure():
    results = run_scenario(_OPERATOR, _BINARY, get_sitl_port())
    assert results["status"] == "PASS", results
