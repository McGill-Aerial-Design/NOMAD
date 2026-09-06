# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Guard the transitional REST reference against route drift.

This test is deleted with the Python API during the C++ cutover. It is not a
contract for the target core.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from edge_core.api import create_app
from edge_core.core import wire_modules
from edge_core.services.state import StateManager

REPO_ROOT = Path(__file__).resolve().parents[1]
API_REFERENCE = REPO_ROOT / "docs" / "api_reference.md"

# Paths that FastAPI serves but does not list under openapi()["paths"].
_NOT_IN_SCHEMA = {"/docs", "/redoc", "/openapi.json"}


def _documented_paths() -> set[str]:
    """Concrete `/...` paths from api_reference.md *table rows* (skip prose)."""
    paths: set[str] = set()
    for line in API_REFERENCE.read_text(encoding="utf-8").splitlines():
        if not line.lstrip().startswith("|"):
            continue  # only Markdown table rows, not prose examples
        for token in re.findall(r"`([^`]+)`", line):
            token = token.strip().split("?")[0].strip()
            if not token.startswith("/"):
                continue
            if "*" in token or token in _NOT_IN_SCHEMA:
                continue
            paths.add(token)
    return paths


@pytest.fixture(scope="module")
def live_paths() -> set[str]:
    import os

    os.environ["NOMAD_SIM_MODE"] = "true"
    os.environ["NOMAD_ALLOW_INSECURE_REMOTE"] = "true"
    app = create_app(StateManager.instance())
    wire_modules(app)
    return set(app.openapi()["paths"].keys())


def test_documented_paths_exist_in_openapi(live_paths):
    documented = _documented_paths()
    assert documented, "no documented paths parsed — check api_reference.md format"
    missing = sorted(documented - live_paths)
    assert not missing, f"api_reference.md documents routes that do not exist: {missing}"
