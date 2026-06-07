# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Edge Core API endpoint smoke test.

Discovers every route from the live ``/openapi.json`` and exercises it:

* **GET** routes are invoked with the API key (placeholders substituted for
  ``{path}`` params). Reachable, non-crashing responses pass.
* **Mutating** routes (POST/PUT/PATCH/DELETE) are NOT executed (they may have
  side effects); instead they are probed *without* a key to confirm the auth
  middleware rejects them (401) before the handler runs.
* **Auth** is verified directly: exempt paths work keyless; a protected route
  returns 401 without the key and succeeds with it.

Usage:
    python scripts/dev/test_api_endpoints.py
    NOMAD_API_URL=http://localhost:8000 NOMAD_API_KEY=nomad-dev-key \
        python scripts/dev/test_api_endpoints.py

Exit code is non-zero if any check fails.
"""

from __future__ import annotations

import json
import os
import sys
import urllib.error
import urllib.request

BASE_URL = os.environ.get("NOMAD_API_URL", "http://localhost:8000").rstrip("/")
API_KEY = os.environ.get("NOMAD_API_KEY", "nomad-dev-key")
TIMEOUT = float(os.environ.get("NOMAD_API_TEST_TIMEOUT", "10"))

EXEMPT_PATHS = {"/", "/health", "/docs", "/redoc", "/openapi.json"}
# Path-param placeholders so parameterized GETs are still reachable.
PARAM_PLACEHOLDER = "1"

# Accept statuses that prove the route is wired + auth passed, even if the
# request data is incomplete or the backing service is absent in sim.
OK_GET = {200, 204, 400, 404, 405, 422, 501, 503}
# Mutating routes probed without a key must be rejected by auth.
OK_MUTATING_NOKEY = {401}


def _request(method: str, path: str, with_key: bool):
    url = f"{BASE_URL}{path}"
    headers = {"Accept": "application/json"}
    if with_key:
        headers["X-API-Key"] = API_KEY
    req = urllib.request.Request(url, method=method, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=TIMEOUT) as resp:
            return resp.status
    except urllib.error.HTTPError as e:
        return e.code
    except Exception as e:  # noqa: BLE001
        return f"ERR:{type(e).__name__}"


def _fill_params(path: str) -> str:
    out = []
    for seg in path.split("/"):
        out.append(PARAM_PLACEHOLDER if seg.startswith("{") and seg.endswith("}") else seg)
    return "/".join(out)


def _fetch_openapi() -> dict:
    return json.loads(_get_body("/openapi.json"))


def _get_body(path: str) -> str:
    with urllib.request.urlopen(f"{BASE_URL}{path}", timeout=TIMEOUT) as resp:
        return resp.read().decode("utf-8")


def main() -> int:
    print(f"NOMAD API endpoint test -> {BASE_URL}")
    try:
        spec = _fetch_openapi()
    except Exception as e:  # noqa: BLE001
        print(f"FATAL: could not fetch /openapi.json: {e}")
        print("Is Edge Core running? Try: pixi run dev-up")
        return 2

    paths = spec.get("paths", {})
    results: list[tuple[str, str, object, bool]] = []

    # --- explicit auth checks -------------------------------------------------
    health = _request("GET", "/health", with_key=False)
    results.append(("GET", "/health (keyless, exempt)", health, health == 200))

    # Pick a protected GET to verify auth enforcement both ways.
    protected = next(
        (p for p, ops in paths.items() if "get" in ops and (p.rstrip("/") or "/") not in EXEMPT_PATHS and "{" not in p),
        None,
    )
    if protected:
        no_key = _request("GET", protected, with_key=False)
        results.append(("GET", f"{protected} (keyless -> 401)", no_key, no_key == 401))
        with_key = _request("GET", protected, with_key=True)
        results.append(("GET", f"{protected} (keyed -> ok)", with_key, with_key in OK_GET))

    # --- enumerate every route ------------------------------------------------
    for path, ops in sorted(paths.items()):
        for method in ops:
            m = method.upper()
            if m not in {"GET", "POST", "PUT", "PATCH", "DELETE"}:
                continue
            call_path = _fill_params(path)
            if m == "GET":
                status = _request("GET", call_path, with_key=True)
                ok = status in OK_GET
            else:
                # Do not execute mutating handlers; confirm auth guards them.
                status = _request(m, call_path, with_key=False)
                ok = status in OK_MUTATING_NOKEY or path.rstrip("/") in EXEMPT_PATHS
            results.append((m, path, status, ok))

    # --- report ---------------------------------------------------------------
    width = max(len(p) for _, p, _, _ in results) + 2
    passed = 0
    for method, path, status, ok in results:
        flag = "PASS" if ok else "FAIL"
        if ok:
            passed += 1
        print(f"  [{flag}] {method:6} {path:<{width}} -> {status}")

    total = len(results)
    print(f"\n{passed}/{total} checks passed")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
