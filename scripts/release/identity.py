# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Generate source-bound component identities and aggregate complete release sets."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import zipfile
from pathlib import Path

try:
    from .mission_planner_target import VERSION as MP_VERSION
except ImportError:
    from mission_planner_target import VERSION as MP_VERSION

try:
    from .manifest import EXPECTED, TAG, digest_file, load_manifest, read_package_identity, verify_package
except ImportError:
    from manifest import EXPECTED, TAG, digest_file, load_manifest, read_package_identity, verify_package

ROOT = Path(__file__).resolve().parents[2]


def git(*arguments: str) -> str:
    """Read repository provenance without accepting caller-provided source claims."""
    return subprocess.check_output(["git", *arguments], cwd=ROOT, text=True, stderr=subprocess.PIPE).strip()


def get_build_tag() -> str:
    """Use the same explicit CI ref or exact local tag as CMake."""
    if "GITHUB_REF_TYPE" in os.environ:
        return os.environ.get("GITHUB_REF_NAME", "") if os.environ["GITHUB_REF_TYPE"] == "tag" else ""
    try:
        value = git("describe", "--tags", "--exact-match", "HEAD")
        return value if TAG.fullmatch(value) else ""
    except subprocess.CalledProcessError:
        return ""


def get_identity(tag: str = "") -> dict:
    """Only an exact clean tag checkout can identify an official release."""
    source = git("rev-parse", "HEAD")
    mavsdk = git("ls-tree", "HEAD", "third_party/MAVSDK").split()[2]
    dirty = bool(git("status", "--porcelain", "--untracked-files=normal"))
    if not tag and not dirty:
        tag = get_build_tag()
    if tag:
        if not TAG.fullmatch(tag) or git("rev-parse", tag + "^{commit}") != source:
            raise ValueError("release tag must be vX.Y.Z at the checked-out source")
        if dirty:
            raise ValueError("official release requires an unchanged source checkout")
    return {
        "schema_version": 1,
        "release_version": tag or "dev-" + source,
        "source_sha": source,
        "mavsdk_sha": mavsdk,
        "official": bool(tag),
        "source_dirty": dirty,
    }


def component_identity(identity: dict, name: str, platform: str, architecture: str) -> dict:
    """Bind one independently supervised component to the release set."""
    if (name, platform, architecture) not in EXPECTED:
        raise ValueError("unsupported component platform")
    version = identity["release_version"][1:] if identity["official"] else "0.0.0-dev." + identity["source_sha"]
    result = dict(identity, name=name, platform=platform, architecture=architecture, version=version)
    result["protocol_versions"] = {"router_management": 1} if name == "router" else {"runtime_ipc": 1}
    if name == "plugin":
        result["mission_planner_target"] = MP_VERSION
    return result


def write_json(path: Path, value: dict) -> None:
    """Replace generated metadata atomically on its own filesystem."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def write_csharp(path: Path, identity: dict) -> None:
    """Emit build inputs, never operator state or credentials."""
    path.parent.mkdir(parents=True, exist_ok=True)
    numeric = identity["version"].split("-")[0]
    source = identity["source_sha"]
    path.write_text(
        "// Generated NOMAD release identity.\nusing System.Reflection;\n"
        f'[assembly: AssemblyVersion("{numeric}.0")]\n'
        f'[assembly: AssemblyInformationalVersion("{identity["version"]}+{source}")]\n'
        "namespace NOMAD.MissionPlanner { public static class NomadRelease {\n"
        f'public const string Version = "{identity["version"]}";\n'
        f'public const string SourceSha = "{source}";\n' + "} }\n",
        encoding="utf-8",
    )


def create_deployment_tools(directory: Path, identity: dict) -> dict:
    """Publish operator-reviewed tooling separately from component payloads."""
    filename = "NOMAD-deployment-tools-" + identity["release_version"] + ".zip"
    package = directory / filename
    paths = sorted((ROOT / "scripts/release").glob("*.py"))
    paths.extend([ROOT / "infra/runtime/install_systemd.py", ROOT / "infra/runtime/nomad-runtime.service.in"])
    paths.append(ROOT / "scripts/release/mission-planner-target.json")
    with zipfile.ZipFile(package, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("scripts/__init__.py", "")
        archive.writestr("scripts/release/__init__.py", "")
        for path in paths:
            archive.write(path, path.relative_to(ROOT).as_posix())
    return {"filename": filename, "sha256": digest_file(package), "source_sha": identity["source_sha"]}


def aggregate(directory: Path, output: Path, tag: str = "") -> dict:
    """Require the full reviewed set before generating publication metadata."""
    identity = get_identity(tag)
    entries = []
    for package in sorted(directory.iterdir()):
        if package.name.startswith("NOMAD-deployment-tools-") or not package.name.endswith((".zip", ".tar.gz")):
            continue
        embedded = read_package_identity(package)
        if any(embedded.get(field) != identity[field] for field in identity):
            raise ValueError("artifact source identity differs from aggregation checkout")
        entry = {
            field: embedded[field] for field in ("name", "version", "platform", "architecture", "protocol_versions")
        }
        if "mission_planner_target" in embedded:
            entry["mission_planner_target"] = embedded["mission_planner_target"]
        entry.update(filename=package.name, sha256=digest_file(package), required_files=embedded["required_files"])
        entries.append(entry)
    tools = create_deployment_tools(directory, identity)
    result = dict(identity, components=entries, deployment_tools=tools)
    write_json(output, result)
    manifest = load_manifest(output)
    for entry in manifest["components"]:
        verify_package(entry, directory / entry["filename"])
    checksum = output.parent / "SHA256SUMS"
    lines = [f"{entry['sha256']}  {entry['filename']}\n" for entry in entries]
    lines.append(f"{tools['sha256']}  {tools['filename']}\n")
    lines.append(f"{digest_file(output)}  {output.name}\n")
    checksum.write_text("".join(lines), encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("component", "aggregate"))
    parser.add_argument("--tag", default="")
    parser.add_argument("--component", choices=("core", "router", "plugin"))
    parser.add_argument("--platform", choices=("linux", "windows"))
    parser.add_argument("--architecture", default="x86_64", choices=("x86_64", "any"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--directory", type=Path)
    parser.add_argument("--csharp", type=Path)
    parser.add_argument("--required", nargs="+", default=[])
    arguments = parser.parse_args()
    if arguments.operation == "aggregate":
        aggregate(arguments.directory, arguments.output, arguments.tag)
        return
    identity = component_identity(
        get_identity(arguments.tag), arguments.component, arguments.platform, arguments.architecture
    )
    identity["required_files"] = ["package-identity.json", *arguments.required]
    write_json(arguments.output, identity)
    if arguments.csharp:
        write_csharp(arguments.csharp, identity)


if __name__ == "__main__":
    main()
