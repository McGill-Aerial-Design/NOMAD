# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Select hosted Windows plugin and router checks from changed paths."""

from __future__ import annotations

import fnmatch
import os
import subprocess

QUALIFICATION_PATH_PATTERNS = (
    "pixi.toml",
    "pixi.lock",
    "mission_planner/**",
    "infra/transport/ground_router/**",
    "scripts/build/**",
    "scripts/ci/csharp_scope.py",
    "scripts/dev/ground_router_smoke.py",
    "scripts/dev/ground_router_fixture_support.py",
    "scripts/dev/runtime_*support.py",
    "scripts/dev/runtime_lifecycle_fixture.py",
    "scripts/dev/release_process*",
    "scripts/dev/release_router_launcher.py",
    "scripts/dev/release_supervisor_fixture.py",
    "scripts/dev/mavsdk_peer*.py",
    "scripts/dev/verify_core_package.py",
    "scripts/release/**",
    ".github/workflows/csharp.yml",
    "tests/test_mission_planner_command_boundary.py",
)


def requires_windows_qualification(paths: list[str]) -> bool:
    """Return whether any change can affect plugin, router, or their checks."""
    return any(
        fnmatch.fnmatchcase(path.replace("\\", "/"), pattern)
        for path in paths
        for pattern in QUALIFICATION_PATH_PATTERNS
    )


def get_changed_paths(event_name: str, push_before: str, base_branch: str, head_sha: str) -> list[str]:
    if event_name == "pull_request":
        base = base_branch
    elif event_name == "push":
        base = push_before
    else:
        return []

    if not base or set(base) == {"0"}:
        raise RuntimeError("Base revision is unavailable")

    result = subprocess.run(
        ["git", "diff", "--name-only", "--no-renames", f"{base}...{head_sha}"],
        capture_output=True,
        check=False,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "Unable to inspect changed paths")
    return [path for path in result.stdout.splitlines() if path]


def main() -> int:
    event_name = os.environ.get("EVENT_NAME", "")
    try:
        paths = get_changed_paths(
            event_name,
            os.environ.get("PUSH_BEFORE", ""),
            os.environ.get("BASE_BRANCH", ""),
            os.environ.get("HEAD_SHA", ""),
        )
        required = event_name not in {"push", "pull_request"} or requires_windows_qualification(paths)
    except RuntimeError as error:
        print(f"Could not determine change scope: {error}; requiring Windows qualification.")
        required = True

    output_path = os.environ.get("GITHUB_OUTPUT")
    if output_path:
        with open(output_path, "a", encoding="utf-8") as output:
            output.write(f"required={'true' if required else 'false'}\n")
    print(f"Windows plugin and router qualification required: {str(required).lower()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
