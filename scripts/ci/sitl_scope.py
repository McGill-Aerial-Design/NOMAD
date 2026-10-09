# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Select full SITL qualification for safety-sensitive changes."""

from __future__ import annotations

import fnmatch
import os
import subprocess

SAFETY_PATH_PATTERNS = (
    ".github/workflows/sitl.yml",
    ".gitmodules",
    "CMakeLists.txt",
    "CMakePresets.json",
    "pixi.toml",
    "pixi.lock",
    "include/nomad/**",
    "src/**",
    "tools/nomad/**",
    "tools/qualification/**",
    "tools/runtime/**",
    "tests/*.cpp",
    "tests/support/**",
    "tests/vehicle/**",
    "tests/sitl/**",
    "tests/test_authority_sitl.py",
    "tests/test_mavsdk_*.py",
    "tests/test_quadplane_*.py",
    "scripts/ci/sitl_scope.py",
    "scripts/dev/core_sitl_*.py",
    "scripts/dev/mavsdk_*.py",
    "scripts/dev/check_mavsdk_provenance.py",
    "scripts/dev/runtime_*.py",
    "docker/**",
    "config/nomad.env.example",
    "config/actuators*.json",
    "third_party/MAVSDK",
    "third_party/MAVSDK/**",
    "third_party/ardupilot-mavlink",
    "third_party/ardupilot-mavlink/**",
    "mission_planner/src/Actuators/**",
    "mission_planner/src/Config/NOMADConfig*.cs",
    "mission_planner/src/Connectivity/**",
    "mission_planner/src/Control/**",
    "mission_planner/src/Geofence/**",
    "mission_planner/src/Input/**",
    "mission_planner/src/Plugin/**",
    "mission_planner/src/Views/NOMADBoundaryView*.cs",
    "mission_planner/tests/geometry/**",
    "mission_planner/tests/gimbal/**",
    "scripts/build/test_plugin_config_validation.ps1",
    "scripts/build/test_plugin_geometry.ps1",
    "scripts/build/test_plugin_gimbal.ps1",
)


def requires_full_sitl(paths: list[str]) -> bool:
    """Return whether changed source paths can affect vehicle behavior or qualification."""
    return any(
        fnmatch.fnmatchcase(path.replace("\\", "/"), pattern) for path in paths for pattern in SAFETY_PATH_PATTERNS
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
        required = event_name not in {"push", "pull_request"} or requires_full_sitl(paths)
    except RuntimeError as error:
        print(f"Could not determine change scope: {error}; requiring full SITL.")
        required = True

    output_path = os.environ.get("GITHUB_OUTPUT")
    if output_path:
        with open(output_path, "a", encoding="utf-8") as output:
            output.write(f"required={'true' if required else 'false'}\n")
    print(f"Full SITL qualification required: {str(required).lower()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
