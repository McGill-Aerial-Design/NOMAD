# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Verify the pinned MAVSDK source and dependency inventory."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

EXPECTED_REVISIONS = {
    "third_party/MAVSDK": "900fb0fe7fec74608f1911331218557915cc501a",
    "third_party/ardupilot-mavlink": "288b907c384a892c8519bfe271682424b1e1a3a0",
}
EXPECTED_DEPENDENCIES = {
    "cpp/third_party/CMakeLists.txt": 'MAVLINK_HASH "d6a7eeaf43319ce6da19a1973ca40180a4210643"',
    "cpp/third_party/asio/CMakeLists.txt": 'ASIO_GIT_TAG "asio-1-30-2"',
    "cpp/third_party/fmt/CMakeLists.txt": "GIT_TAG 12.1.0",
    "cpp/third_party/libevents/CMakeLists.txt": "GIT_TAG 840a88ea226d4eb0fd4c391ce860317422756435",
    "cpp/third_party/libmavlike/CMakeLists.txt": "GIT_TAG 90498b14262137ae10b633705810e81bdb85de9c",
    "cpp/third_party/liblzma/CMakeLists.txt": (
        "URL_HASH SHA256=135c90b934aee8fbc0d467de87a05cb70d627da36abe518c357a873709e5b7d6"
    ),
    "cpp/third_party/mavlink/mavlink.patch": '"PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"',
    "cpp/third_party/nlohmann_json/CMakeLists.txt": (
        "URL_HASH SHA256=4b92eb0c06d10683f7447ce9406cb97cd4b453be18d7279320f7b2f025c10187"
    ),
    "cpp/third_party/picosha2/CMakeLists.txt": "GIT_TAG 1bf940d8a03bb752604fbb366d47b97b50b9e6ce",
    "cpp/third_party/tinyxml2/CMakeLists.txt": "GIT_TAG 11.0.0",
}
EXPECTED_ARCHIVE_OPTIONS = {
    "cpp/third_party/liblzma/CMakeLists.txt": "DOWNLOAD_EXTRACT_TIMESTAMP TRUE",
    "cpp/third_party/nlohmann_json/CMakeLists.txt": "DOWNLOAD_EXTRACT_TIMESTAMP TRUE",
}
FORBIDDEN_DEPENDENCY_TEXT = {
    "cpp/third_party/liblzma/CMakeLists.txt": ("URL_MD5",),
    "cpp/third_party/picosha2/CMakeLists.txt": ("GIT_TAG cmake-install-support",),
}
FORBIDDEN_PATCH_ADDITIONS = {
    "cpp/third_party/mavlink/mavlink.patch": ("-m pip install", "pip-dependencies"),
}
# Git tree object IDs are used instead of hashing working-tree bytes. This keeps
# the check exact while remaining independent of checkout EOL conversion and
# other platform-specific worktree filters.
EXPECTED_LICENSE_BLOBS = {
    "Asio-Boost-1.0.txt": "36b7cd93cdfbac762f5be4c6ce276df2ea6305c2",
    "fmt-MIT.txt": "1cd1ef92696b65b2ecb0e560e622f0a0e869dcf3",
    "libevents-BSD-3-Clause.txt": "636cc2972ae0336bd65c0fe96de3a80e5814b6b9",
    "libmavlike-BSD-3-Clause.txt": "384550480ac2c493696a7fe3c4bad9ac7a6dafc0",
    "MAVLink-generator-and-output.txt": "6f1d4c61b0e2de896454cac2a3300e6d47371254",
    "MAVSDK-BSD-3-Clause.txt": "edd02bd724689dc54aa0168f92e79dafae104f6c",
    "nlohmann-json-MIT.txt": "a1dacc8dbbd907c4b622ff1f08e279c27465dcbc",
    "PicoSHA2-MIT.txt": "b6658bbc98a5b198a49fce48bc2aec7283bef62a",
    "tinyxml2-Zlib.txt": "85a6a36f0ddf7cda552b7a031a9682f2c062d722",
    "XZ-COPYING.txt": "f4406b9a20bfeeec8c56566b543664e52438e23f",
    "XZ-GPL-2.0.txt": "d159169d1050894d3ea3b98e1c965c4058208fe1",
    "XZ-GPL-3.0.txt": "f288702d2fa16d3cdf0035b15a9fcbc552cd88e7",
    "XZ-LGPL-2.1.txt": "e5ab03e1238af66de157fae2e6270d7e8f967f93",
}
NOTICE_COMPONENTS = (
    "MAVSDK",
    "Asio",
    "fmt",
    "libevents",
    "libmavlike",
    "MAVLink",
    "nlohmann JSON",
    "PicoSHA2",
    "tinyxml2",
    "liblzma",
)


def git_revision(path: str) -> str:
    result = subprocess.run(
        ["git", "rev-parse", f"HEAD:{path}"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def check_expected_text(path: Path, expected: str) -> None:
    if expected not in path.read_text(encoding="utf-8"):
        raise RuntimeError(f"{path.relative_to(ROOT)} no longer contains expected provenance: {expected}")


def check_forbidden_text(path: Path, forbidden: tuple[str, ...]) -> None:
    contents = path.read_text(encoding="utf-8")
    found = [value for value in forbidden if value in contents]
    if found:
        values = ", ".join(found)
        raise RuntimeError(f"{path.relative_to(ROOT)} contains mutable or weak provenance: {values}")


def patch_added_text(path: Path) -> str:
    lines = path.read_text(encoding="utf-8").splitlines()
    return "\n".join(line[1:] for line in lines if line.startswith("+") and not line.startswith("+++"))


def check_forbidden_patch_additions(path: Path, forbidden: tuple[str, ...]) -> None:
    additions = patch_added_text(path)
    found = [value for value in forbidden if value in additions]
    if found:
        values = ", ".join(found)
        raise RuntimeError(f"{path.relative_to(ROOT)} adds mutable or weak provenance: {values}")


def verify_license_bundle() -> None:
    license_root = ROOT / "licenses" / "mavsdk-phase-a"
    actual_names = {path.name for path in license_root.glob("*.txt")}
    expected_names = set(EXPECTED_LICENSE_BLOBS)
    if actual_names != expected_names:
        missing = sorted(expected_names - actual_names)
        unexpected = sorted(actual_names - expected_names)
        details = []
        if missing:
            details.append(f"missing: {', '.join(missing)}")
        if unexpected:
            details.append(f"unexpected: {', '.join(unexpected)}")
        raise RuntimeError(f"MAVSDK redistribution license inventory changed ({'; '.join(details)})")

    for name, expected in EXPECTED_LICENSE_BLOBS.items():
        path = license_root / name
        if not path.is_file():
            raise RuntimeError(f"missing MAVSDK redistribution license: {path.relative_to(ROOT)}")
        repo_path = f"licenses/mavsdk-phase-a/{name}"
        actual = git_revision(repo_path)
        if actual != expected:
            raise RuntimeError(
                f"MAVSDK redistribution license changed: {name}; expected blob {expected}, observed {actual}"
            )


def verify_provenance() -> None:
    for path, expected in EXPECTED_REVISIONS.items():
        actual = git_revision(path)
        if actual != expected:
            raise RuntimeError(f"{path} revision changed: expected {expected}, observed {actual}")

    mavsdk_root = ROOT / "third_party" / "MAVSDK"
    for relative, expected in EXPECTED_DEPENDENCIES.items():
        check_expected_text(mavsdk_root / relative, expected)
    for relative, expected in EXPECTED_ARCHIVE_OPTIONS.items():
        check_expected_text(mavsdk_root / relative, expected)
    for relative, forbidden in FORBIDDEN_DEPENDENCY_TEXT.items():
        check_forbidden_text(mavsdk_root / relative, forbidden)
    for relative, forbidden in FORBIDDEN_PATCH_ADDITIONS.items():
        check_forbidden_patch_additions(mavsdk_root / relative, forbidden)

    proto = subprocess.run(
        ["git", "rev-parse", "HEAD:proto"],
        cwd=mavsdk_root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if proto != "5c81ecfeb6110cf74ba75ae50b78a1b265c05670":
        raise RuntimeError(f"MAVSDK proto revision changed: {proto}")

    notice = (ROOT / "NOTICE").read_text(encoding="utf-8")
    missing = [component for component in NOTICE_COMPONENTS if component not in notice]
    if missing:
        raise RuntimeError(f"NOTICE is missing MAVSDK dependencies: {', '.join(missing)}")
    if "licenses/mavsdk-phase-a" not in notice:
        raise RuntimeError("NOTICE does not identify the MAVSDK redistribution license bundle")
    verify_license_bundle()


def main() -> int:
    try:
        verify_provenance()
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    print("MAVSDK provenance matches the reviewed inventory")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
