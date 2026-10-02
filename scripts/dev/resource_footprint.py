# SPDX-License-Identifier: Apache-2.0
"""Release artifacts and selected MAVSDK archives, without debug contamination."""

from __future__ import annotations

from pathlib import Path

from mavsdk_build_metrics import directory_size
from resource_composition import collect_composition, read_targets
from resource_metadata import ROOT


def release_binary(build: Path, name: str) -> Path:
    candidates = [build / "Release" / f"{name}.exe", build / f"{name}.exe", build / name]
    binary = next((item for item in candidates if item.is_file()), None)
    if binary is None:
        raise FileNotFoundError(f"Release {name} not found; build production targets first")
    return binary


def collect_footprint(build: Path) -> tuple[dict[str, int], dict]:
    stage = build / "stage"
    if not (stage / "bin").is_dir():
        raise FileNotFoundError("verified core staged installation is required")
    debug = [item for item in stage.rglob("*") if item.suffix.lower() in {".pdb", ".ilk", ".obj", ".debug"}]
    if debug:
        raise ValueError("debug artifacts contaminate the release staged installation")
    archives = sorted(set(build.glob("nomad-core*.zip")) | set(build.glob("nomad-core*.tar.gz")))
    if len(archives) != 2:
        raise ValueError("expected exactly one ZIP and one TGZ; use a clean package build")
    values = {
        "runtime_binary_bytes": release_binary(build, "nomad-runtime").stat().st_size,
        "cli_binary_bytes": release_binary(build, "nomad").stat().st_size,
        "core_stage_bytes": directory_size(stage),
        "package_zip_bytes": next(item.stat().st_size for item in archives if item.suffix == ".zip"),
        "package_tgz_bytes": next(item.stat().st_size for item in archives if item.suffix == ".gz"),
        "mavsdk_build_tree_bytes": directory_size(build / "mavsdk"),
        "dependency_stage_bytes": directory_size(build / "mavsdk/third_party/install"),
        "mavsdk_stage_bytes": directory_size(build / "mavsdk-stage"),
        "mavsdk_source_bytes": directory_size(ROOT / "third_party/MAVSDK"),
    }
    libraries = list_release_libraries(build)
    inventory = {item.relative_to(build).as_posix(): item.stat().st_size for item in sorted(libraries)}
    composition = collect_composition(build, inventory)
    values["built_static_library_bytes"] = sum(inventory.values())
    values["linked_static_library_bytes"] = sum(
        size for name, size in inventory.items() if Path(name).name in composition["runtime_link_archives"]
    )
    return values, composition


def list_release_libraries(build: Path) -> list[Path]:
    target = read_targets(build)["mavsdk"]
    sdk = [build / item["path"] for item in target["artifacts"] if Path(item["path"]).suffix in {".a", ".lib"}]
    dependencies = [
        item
        for item in (build / "mavsdk/third_party/install").rglob("*")
        if item.is_file() and item.suffix in {".a", ".lib"}
    ]
    return sorted(set(sdk + dependencies))
