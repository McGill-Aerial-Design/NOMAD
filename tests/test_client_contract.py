# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Transitional client-to-API contract gate.

The C++ migration will replace this REST contract after the new client boundary
exists. Until then, it prevents the current plugin from drifting.

The hand-written HTTP client (the C# plugin) can silently drift from the
FastAPI routes. The hazard is drift, not hand-writing — so this test kills
drift: it extracts every route literal from the C# sources and asserts each
``(method, path)`` pair exists in the live ``app.openapi()`` schema. A client
calling a route (or verb) that no longer exists fails CI here instead of
failing in the field. (The Python ROS-HTTP bridge was the second client; it
was retired with the bridge on 2026-09-05.)
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

# Method call tokens preceding a route literal in the C# sources -> HTTP verb.
_CS_METHOD_VERB = {
    "GetAsync": "get",
    "GetLongRunAsync": "get",
    "GetStringAsync": "get",
    "TryGetAsync": "get",
    "PostAsync": "post",
    "PostLongRunAsync": "post",
    "PostJsonAsync": "post",
    "TryPostAsync": "post",
    "DeleteAsync": "delete",
}
_CS_METHOD_RE = re.compile(r"\b(" + "|".join(_CS_METHOD_VERB) + r")\s*\(")


def _normalize(path: str) -> str:
    return _PARAM_RE.sub("{}", path.split("?")[0].rstrip("/"))


def _client_routes() -> dict[tuple[str | None, str], list[str]]:
    """(verb-or-None, normalized route) -> the client files referencing it.

    The verb is the C# wrapper method nearest before the route literal
    (searched within the same statement window); ``None`` when no recognized
    call token precedes the literal (e.g. a bare constant).
    """
    routes: dict[tuple[str | None, str], list[str]] = {}
    for cs_file in CS_ROOT.rglob("*.cs"):
        text = cs_file.read_text(encoding="utf-8", errors="replace")
        for match in _ROUTE_RE.finditer(text):
            normalized = _normalize(match.group(1))
            if not normalized:
                continue
            window = text[max(0, match.start() - 160) : match.start()]
            tokens = _CS_METHOD_RE.findall(window)
            verb = _CS_METHOD_VERB[tokens[-1]] if tokens else None
            routes.setdefault((verb, normalized), []).append(cs_file.name)
    return routes


@pytest.fixture(scope="module")
def live_operations() -> set[tuple[str, str]]:
    """Every (method, normalized path) pair served by the live app."""
    import os

    from edge_core.api import create_app
    from edge_core.core import wire_modules
    from edge_core.services.state import StateManager

    os.environ["NOMAD_SIM_MODE"] = "true"
    app = create_app(StateManager.instance())
    wire_modules(app)
    return {(method, _normalize(path)) for path, ops in app.openapi()["paths"].items() for method in ops}


@pytest.fixture(scope="module")
def live_paths(live_operations) -> set[str]:
    return {path for _, path in live_operations}


# Known drift is kept empty during the transition. Remove this entire REST test
# with the Python API once the replacement client boundary is accepted.
KNOWN_DRIFT: dict[str, str] = {}  # burned down to zero (baseline-polish plan 4.1); keep it that way


def test_every_client_route_exists_in_api(live_operations, live_paths):
    routes = _client_routes()
    assert routes, "no route literals parsed from the C# sources — check the regex"

    by_key = sorted(routes.items(), key=lambda kv: (kv[0][1], kv[0][0] or ""))
    missing_paths = {(verb, path): files for (verb, path), files in by_key if path not in live_paths}
    new_drift = {key: files for key, files in missing_paths.items() if key[1] not in KNOWN_DRIFT}
    assert not new_drift, (
        f"C# client calls routes the API does not serve: {new_drift}. "
        "Either the route was removed/renamed (update the client) or the "
        "client method is dead (delete it). Do not extend KNOWN_DRIFT for new code."
    )

    missing = {path for _, path in missing_paths}
    fixed = sorted(set(KNOWN_DRIFT) - missing)
    assert not fixed, f"KNOWN_DRIFT entries no longer drift — remove them from the ledger: {fixed}"

    # Verb check: a path that exists must also serve the verb the client uses.
    verb_drift = {
        (verb, path): files
        for (verb, path), files in by_key
        if verb is not None and path in live_paths and (verb, path) not in live_operations
    }
    assert not verb_drift, (
        f"C# client uses an HTTP method the API does not serve on that path: {verb_drift}. "
        "Add the verb server-side or fix the client call."
    )
