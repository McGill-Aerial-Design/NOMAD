# SPDX-License-Identifier: Apache-2.0
"""Record an already completed isolated build; run supervision verifies this binding."""

import hashlib
import json
import subprocess
import sys
from pathlib import Path

PATCHES = Path(__file__).resolve().parent / "patches"


def patches(name):
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(PATCHES.glob(name + "-*.patch"))}


def record(workspace, name, binary):
    workspace = Path(workspace).resolve(strict=True)
    binary = Path(binary).resolve(strict=True)
    binary.relative_to(workspace)
    if not name.isidentifier():
        raise ValueError("Invalid source component name")
    source = workspace / "sources" / name
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    dirty = subprocess.check_output(["git", "-C", str(source), "status", "--porcelain"], text=True).strip()
    if dirty:
        raise ValueError("Upstream source has unrecorded changes")
    manifest = {
        "schema": 1,
        "source": revision,
        "binary": str(binary),
        "sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
        "patches": patches(name),
        "submodules": subprocess.check_output(
            ["git", "-C", str(source), "submodule", "status", "--recursive"], text=True
        ).splitlines(),
    }
    artifacts = [binary, *binary.parent.glob("*.so*")]
    manifest["artifacts"] = {}
    for artifact in artifacts:
        artifact = artifact.resolve(strict=True)
        relative = artifact.relative_to(workspace)
        manifest["artifacts"][str(relative)] = hashlib.sha256(artifact.read_bytes()).hexdigest()
    if name == "last_letter":
        manifest["dependencies"] = {
            dependency: subprocess.check_output(
                ["git", "-C", str(workspace / "sources" / dependency), "rev-parse", "HEAD"], text=True
            ).strip()
            for dependency in ("yaml_cpp", "data_tamer", "nlopt")
        }
    (workspace / "logs" / f"{name}-build.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def verify(workspace, name, revision, binary):
    workspace = Path(workspace).resolve(strict=True)
    binary = Path(binary).resolve(strict=True)
    manifest = json.loads((workspace / "logs" / f"{name}-build.json").read_text())
    if manifest.get("schema") != 1 or "artifacts" not in manifest:
        raise ValueError("Build manifest needs regeneration with the current build tool")
    if manifest["source"] != revision or Path(manifest["binary"]).resolve(strict=True) != binary:
        raise ValueError("Built source/executable binding mismatch")
    source = workspace / "sources" / name
    current = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    dirty = subprocess.check_output(["git", "-C", str(source), "status", "--porcelain"], text=True).strip()
    if current != revision or dirty:
        raise ValueError("Upstream source/model changed after the recorded build")
    if manifest.get("patches", {}) != patches(name):
        raise ValueError("NOMAD simulation patch changed after the recorded build; rebuild")
    submodules = subprocess.check_output(
        ["git", "-C", str(source), "submodule", "status", "--recursive"], text=True
    ).splitlines()
    if submodules != manifest["submodules"] or any(line.startswith(("+", "U")) for line in submodules):
        raise ValueError("Upstream submodule changed after the recorded build")
    if str(binary.relative_to(workspace)) not in manifest["artifacts"]:
        raise ValueError("Executable is absent from the artifact manifest")
    for relative, expected in manifest["artifacts"].items():
        artifact = (workspace / relative).resolve(strict=True)
        artifact.relative_to(workspace)
        if hashlib.sha256(artifact.read_bytes()).hexdigest() != expected:
            raise ValueError("Built executable/shared-library checksum mismatch")
    for dependency, expected in manifest.get("dependencies", {}).items():
        path = workspace / "sources" / dependency
        current = subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
        dirty = subprocess.check_output(["git", "-C", str(path), "status", "--porcelain"], text=True).strip()
        if current != expected or dirty:
            raise ValueError("Built dependency changed after the recorded build")
    return manifest


if __name__ == "__main__":
    record(sys.argv[1], sys.argv[2], sys.argv[3])
