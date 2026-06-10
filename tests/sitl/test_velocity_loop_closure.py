# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Pytest wrapper for the SITL velocity loop-closure scenario.

Skipped by default: it requires a live ArduPilot SITL reachable on two MAVLink
endpoints (operator + controller), provided via ``NOMAD_SITL_OPERATOR`` and
``NOMAD_SITL_CONTROLLER``. See tests/sitl/README.md for how to run it against
the ``pixi run dev-up`` stack. Normal CI (no SITL) collects this as a skip.
"""

from __future__ import annotations

import os
import sys

import pytest

# Import the sibling scenario module without requiring a package layout.
sys.path.insert(0, os.path.dirname(__file__))
from velocity_loop_closure import run_scenario  # noqa: E402

_OPERATOR = os.environ.get("NOMAD_SITL_OPERATOR")
_CONTROLLER = os.environ.get("NOMAD_SITL_CONTROLLER")


@pytest.mark.skipif(
    not (_OPERATOR and _CONTROLLER),
    reason="set NOMAD_SITL_OPERATOR and NOMAD_SITL_CONTROLLER to run against a live ArduPilot SITL",
)
def test_velocity_loop_closure():
    results = run_scenario(_OPERATOR, _CONTROLLER)
    assert results["status"] == "PASS", results
