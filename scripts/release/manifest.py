# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Strict NOMAD release-set metadata and package correlation (not signatures)."""

from __future__ import annotations

import hashlib
import json
import re
import tarfile
import zipfile
from pathlib import Path, PurePosixPath

EXPECTED = {
    ("core", "linux", "x86_64"),
    ("core", "windows", "x86_64"),
    ("router", "windows", "x86_64"),
    ("plugin", "windows", "any"),
}
CORE_REQUIRED_FILES = (
    "include/nomad/vehicle/vehicle.hpp",
    "share/nomad/LICENSE",
    "share/nomad/NOTICE",
    "share/nomad/config/README.md",
    "share/nomad/config/nomad.env.example",
    "share/nomad/operations.md",
    "share/nomad/lifecycle/nomad-runtime.service.in",
    "share/nomad/lifecycle/install_systemd.py",
    "share/nomad/lifecycle/Manage-NomadRuntime.ps1",
    "share/nomad/lifecycle/runtime.example.json",
)
MAX_IDENTITY_BYTES = 64 * 1024


SHA = re.compile(r"[0-9a-f]{40}")
DIGEST = re.compile(r"[0-9a-f]{64}")
TAG = re.compile(r"v(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)")


def reject_duplicate_keys(pairs: list[tuple]) -> dict:
    """Ambiguous JSON is never deployment metadata."""
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: " + key)
        result[key] = value
    return result


def parse_json(data: str | bytes) -> dict:
    return json.loads(data, object_pairs_hook=reject_duplicate_keys)


def digest_file(path: Path) -> str:
    """Hash package bytes without executing package content."""
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def validate_relative(value: str) -> None:
    """Reject paths unsafe on either Windows or Linux."""
    if not isinstance(value, str) or not value or "\\" in value or ":" in value:
        raise ValueError("invalid package path")
    path = PurePosixPath(value)
    if any(
        part.endswith((" ", "."))
        or part.upper().split(".")[0]
        in {
            "CON",
            "PRN",
            "AUX",
            "NUL",
            *("COM" + str(n) for n in range(1, 10)),
            *("LPT" + str(n) for n in range(1, 10)),
        }
        for part in path.parts
    ):
        raise ValueError("unsafe Windows package path")
    if path.is_absolute() or ".." in path.parts or value != path.as_posix():
        raise ValueError("unsafe package path")


def validate_identity(value: dict) -> None:
    """Check shared source identity and honest official/dev version."""
    if not isinstance(value, dict) or type(value.get("schema_version")) is not int or value["schema_version"] != 1:
        raise ValueError("unsupported release manifest schema")
    if any(
        not isinstance(value.get(key), str) or not SHA.fullmatch(value[key]) for key in ("source_sha", "mavsdk_sha")
    ):
        raise ValueError("release requires full source and MAVSDK SHA")
    official = value.get("official")
    if type(official) is not bool:
        raise ValueError("official must be a boolean")
    version = value.get("release_version", "")
    expected = TAG.fullmatch(version) if isinstance(version, str) else None
    dirty = value.get("source_dirty", False)
    if type(dirty) is not bool or official and dirty:
        raise ValueError("official releases require clean source metadata")
    if official and not expected:
        raise ValueError("official release requires vX.Y.Z")
    if not official and version != "dev-" + value["source_sha"]:
        raise ValueError("development release must name its source SHA")


def validate_entry(entry: dict, manifest: dict) -> None:
    """Reject incomplete or unsupported component declarations."""
    if not isinstance(entry, dict):
        raise ValueError("component entry must be an object")
    if any(not isinstance(entry.get(key), str) for key in ("name", "platform", "architecture")):
        raise ValueError("component identity fields must be strings")
    key = (entry.get("name"), entry.get("platform"), entry.get("architecture"))
    if key not in EXPECTED:
        raise ValueError("unsupported component/platform")
    version = manifest["release_version"][1:] if manifest["official"] else "0.0.0-dev." + manifest["source_sha"]
    if entry.get("version") != version:
        raise ValueError("component version disagrees with release identity")
    filename = entry.get("filename", "")
    validate_relative(filename)
    if "/" in filename or not filename.endswith((".zip", ".tar.gz")):
        raise ValueError("package must be a ZIP or TGZ filename")
    if not DIGEST.fullmatch(str(entry.get("sha256", ""))):
        raise ValueError("package requires SHA-256")
    validate_required_files(entry)
    validate_protocols(entry)


def validate_required_files(entry: dict) -> None:
    required = entry.get("required_files")
    if (
        not isinstance(required, list)
        or not required
        or any(not isinstance(name, str) for name in required)
        or len(required) != len(set(required))
    ):
        raise ValueError("required_files must be a nonempty unique list")
    for name in required:
        validate_relative(name)
    essentials = {
        "plugin": ["NOMADPlugin.dll"],
        "router": ["nomad-link-router.exe", "Nomad.LinkRouter.dll"],
        "core": list(CORE_REQUIRED_FILES)
        + (
            ["bin/nomad.exe", "bin/nomad-runtime.exe"]
            if entry["platform"] == "windows"
            else ["bin/nomad", "bin/nomad-runtime"]
        ),
    }
    if not set(essentials[entry["name"]]).issubset(required):
        raise ValueError("manifest omits required component payload")
    if "package-identity.json" not in required:
        raise ValueError("package identity is required")


def validate_protocols(entry: dict) -> None:
    protocols = {"router_management": 1} if entry["name"] == "router" else {"runtime_ipc": 1}
    actual_protocols = entry.get("protocol_versions")
    if actual_protocols != protocols or any(type(value) is not int for value in actual_protocols.values()):
        raise ValueError("unsupported component protocol")
    if entry["name"] == "plugin" and entry.get("mission_planner_target") != "1.3.83":
        raise ValueError("unsupported Mission Planner target")


def load_manifest(path: Path) -> dict:
    """Read a complete release set; integrity never establishes publisher trust."""
    manifest = parse_json(Path(path).read_text(encoding="utf-8"))
    validate_identity(manifest)
    entries = manifest.get("components")
    if not isinstance(entries, list) or len(entries) != len(EXPECTED):
        raise ValueError("release set must contain all four required components")
    for entry in entries:
        validate_entry(entry, manifest)
    keys = {(entry["name"], entry["platform"], entry["architecture"]) for entry in entries}
    if keys != EXPECTED or len({entry["filename"] for entry in entries}) != len(entries):
        raise ValueError("duplicate or incomplete release set")
    tools = manifest.get("deployment_tools")
    if tools is not None:
        validate_tools(tools, manifest)
    return manifest


def validate_tools(tools: dict, manifest: dict) -> None:
    """Correlate the separate reviewed deployment CLI archive."""
    if not isinstance(tools, dict) or tools.get("source_sha") != manifest["source_sha"]:
        raise ValueError("deployment tooling source identity mismatch")
    validate_relative(tools.get("filename", ""))
    if "/" in tools["filename"] or not tools["filename"].endswith(".zip"):
        raise ValueError("invalid deployment tooling filename")
    if not isinstance(tools.get("sha256"), str) or not DIGEST.fullmatch(tools["sha256"]):
        raise ValueError("deployment tooling requires SHA-256")


def select_component(manifest: dict, component: str, platform: str, architecture: str = "x86_64") -> dict:
    """Select exactly one compatible payload."""
    found = [
        entry
        for entry in manifest["components"]
        if (entry["name"], entry["platform"], entry["architecture"]) == (component, platform, architecture)
    ]
    if len(found) != 1:
        raise ValueError("component is absent or unsupported on this platform")
    return found[0]


def read_package_identity(package: Path) -> dict:
    """Read one embedded identity and reject unsafe archive members first."""
    if zipfile.is_zipfile(package):
        with zipfile.ZipFile(package) as archive:
            members = archive.infolist()
            for member in members:
                validate_relative(member.filename.rstrip("/"))
                if member.external_attr >> 16 & 0o170000 == 0o120000:
                    raise ValueError("archive links are forbidden")
            names = [item.filename for item in members if PurePosixPath(item.filename).name == "package-identity.json"]
            if len(names) != 1:
                raise ValueError("package requires exactly one embedded identity")
            if archive.getinfo(names[0]).file_size > MAX_IDENTITY_BYTES:
                raise ValueError("embedded package identity exceeds 64 KiB")
            return parse_json(archive.read(names[0]))
    with tarfile.open(package, "r:gz") as archive:
        members = archive.getmembers()
        for member in members:
            validate_relative(member.name.rstrip("/"))
            if not member.isfile() and not member.isdir():
                raise ValueError("archive links and special files are forbidden")
        identities = [item for item in members if PurePosixPath(item.name).name == "package-identity.json"]
        if len(identities) != 1:
            raise ValueError("package requires exactly one embedded identity")
        if identities[0].size > MAX_IDENTITY_BYTES:
            raise ValueError("embedded package identity exceeds 64 KiB")
        stream = archive.extractfile(identities[0])
        if stream is None:
            raise ValueError("identity must be a regular file")
        return parse_json(stream.read(MAX_IDENTITY_BYTES + 1))


def verify_package(entry: dict, package: Path) -> dict:
    """Verify digest and identity before extracting or executing anything."""
    package = Path(package)
    if package.name != entry["filename"] or digest_file(package) != entry["sha256"]:
        raise ValueError("package filename or SHA-256 mismatch")
    identity = read_package_identity(package)
    validate_identity(identity)
    if identity.get("required_files") != entry["required_files"]:
        raise ValueError("embedded required contents disagree with manifest")
    verify_required_contents(package, entry["required_files"])
    for field in ("name", "version", "platform", "architecture", "protocol_versions", "mission_planner_target"):
        if identity.get(field) != entry.get(field):
            raise ValueError("embedded package identity mismatch: " + field)
    return identity


def verify_required_contents(package: Path, required: list[str]) -> None:
    """Check actual payload files and a single optional archive root."""
    if zipfile.is_zipfile(package):
        with zipfile.ZipFile(package) as archive:
            names = [item.filename for item in archive.infolist() if not item.is_dir()]
    else:
        with tarfile.open(package, "r:gz") as archive:
            names = [item.name for item in archive.getmembers() if item.isfile()]
    if len({name.casefold() for name in names}) != len(names):
        raise ValueError("duplicate archive entries")
    identity = next(name for name in names if PurePosixPath(name).name == "package-identity.json")
    prefix = identity[: -len("package-identity.json")]
    if any(prefix + name not in names for name in required):
        raise ValueError("package is missing required content")
    if any(not name.startswith(prefix) for name in names):
        raise ValueError("archive contains files outside component root")
