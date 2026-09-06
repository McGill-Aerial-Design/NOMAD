# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Regression for the silent docker-probe wiring break (plan F2): the probe
used to be attached to ApiRouteContext while IsaacModule looked it up on
AppContext — never found, so nvblox/bridge detection silently never ran."""

from __future__ import annotations

import pytest

pytest.importorskip("fastapi")


def test_isaac_module_probe_is_wired_after_wire_modules():
    import os

    from edge_core.api import create_app
    from edge_core.core import wire_modules
    from edge_core.services.state import StateManager

    os.environ["NOMAD_SIM_MODE"] = "true"
    app = create_app(StateManager.instance())
    registry = wire_modules(app)
    assert registry is not None and "isaac_mgmt" in registry.modules
    # The in-container process probe is a module-level helper now.
    from edge_core.api_routes.isaac import _docker_exec_pgrep

    assert _docker_exec_pgrep is not None
