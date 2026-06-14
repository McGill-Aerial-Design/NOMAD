# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Pytest wrapper for the SITL gimbal mount-control scenario.

Skipped by default: it requires a live ArduPilot SITL reachable on the operator
MAVLink endpoint, provided via ``NOMAD_SITL_OPERATOR``. See tests/sitl/README.md
for how to run it against the ``pixi run dev-up`` stack. Normal CI (no SITL)
collects this as a skip.
"""

from __future__ import annotations

import os
import sys

import pytest

# Import the sibling scenario module without requiring a package layout.
sys.path.insert(0, os.path.dirname(__file__))
from gimbal_mount_control import MountNotActuated, run_scenario  # noqa: E402

_OPERATOR = os.environ.get("NOMAD_SITL_OPERATOR")


@pytest.mark.skipif(
    not _OPERATOR,
    reason="set NOMAD_SITL_OPERATOR to run against a live ArduPilot SITL",
)
def test_gimbal_mount_control():
    try:
        results = run_scenario(_OPERATOR)
    except MountNotActuated as exc:
        pytest.skip(str(exc))
    assert results["status"] == "PASS", results
