# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Client <-> API contract gate (rearchitecture plan Phase 5).

The C# plugin's hand-written HTTP calls can silently drift from the FastAPI
routes. The hazard is drift, not hand-writing — so this test kills drift:
it extracts every route literal from the C# sources and asserts each exists in
the live ``app.openapi()`` schema. A client method calling a route that no
longer exists (or never did) fails CI here instead of failing in the field.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
CS_ROOT = REPO_ROOT / "mission_planner" / "src"

# Route literals in C# string (incl. $"..." interpolated) constants.
_ROUTE_RE = re.compile(r'"(/(?:api|health|network|status)(?:[/?][^"\s]*)?)"')

# Interpolation holes / path params are compared positionally, not by name.
_PARAM_RE = re.compile(r"\{[^}]*\}")


def _normalize(path: str) -> str:
    return _PARAM_RE.sub("{}", path.split("?")[0].rstrip("/"))


def _client_routes() -> dict[str, list[str]]:
    """Normalized route -> the C# files referencing it."""
    routes: dict[str, list[str]] = {}
    for cs_file in CS_ROOT.rglob("*.cs"):
        text = cs_file.read_text(encoding="utf-8", errors="replace")
        for match in _ROUTE_RE.finditer(text):
            normalized = _normalize(match.group(1))
            if normalized:
                routes.setdefault(normalized, []).append(cs_file.name)
    return routes


@pytest.fixture(scope="module")
def live_paths() -> set[str]:
    import os

    from edge_core.api import create_app
    from edge_core.core import wire_modules
    from edge_core.services.state import StateManager

    os.environ["NOMAD_SIM_MODE"] = "true"
    app = create_app(StateManager.instance())
    wire_modules(app)
    return {_normalize(p) for p in app.openapi()["paths"]}


# Known drift: C# UI still calls these routes, which were deliberately gutted
# from the Python baseline. Each entry is tracked debt (rearchitecture plan §6,
# opportunistic C# cleanup): when the panel/view is next touched, delete the
# caller (or re-add the route as a deployment module) and remove the row here.
# This ledger is two-way checked — adding NEW drift fails, and fixing an entry
# without removing its row also fails, so it cannot rot silently.
KNOWN_DRIFT = {
    "/api/admin/git-update": "EnhancedHealthDashboard update button; admin routes gutted",
    "/api/admin/upload-gdrive-token": "Settings uploads tab; admin routes gutted",
    "/api/detections": "SLAM3DView overlay poll; detections are per-deployment modules",
    "/api/isaac/launch-nvblox": "ServiceControlPanel nvblox button; device-only route gutted",
    "/api/isaac/stop-nvblox": "ServiceControlPanel nvblox button; device-only route gutted",
    "/api/isaac/logs": "DualLinkSender log fetch; route gutted",
    "/api/servo/status": "SLAM3DView servo poll; route gutted",
    "/api/slam/clear": "SLAM3DView / DualLinkSender; SLAM map routes gutted",
    "/api/slam/status": "ServiceControlPanel SLAM status; route gutted",
    "/api/tools/rviz2/start": "Rviz2View; tools routes gutted",
    "/api/tools/rviz2/stop": "Rviz2View; tools routes gutted",
}


def test_every_client_route_exists_in_api(live_paths):
    routes = _client_routes()
    assert routes, "no route literals parsed from the C# sources — check the regex"
    missing = {route: files for route, files in sorted(routes.items()) if route not in live_paths}

    new_drift = {route: files for route, files in missing.items() if route not in KNOWN_DRIFT}
    assert not new_drift, (
        f"C# client calls routes the API does not serve: {new_drift}. "
        "Either the route was removed/renamed (update the client) or the "
        "client method is dead (delete it). Do not extend KNOWN_DRIFT for new code."
    )

    fixed = sorted(set(KNOWN_DRIFT) - set(missing))
    assert not fixed, f"KNOWN_DRIFT entries no longer drift — remove them from the ledger: {fixed}"
