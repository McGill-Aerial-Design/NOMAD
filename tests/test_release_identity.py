# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Release-set correlation, completeness, and pre-execution rejection."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import zipfile
from pathlib import Path

import pytest

from scripts.release import identity, manifest

SOURCE = "a" * 40
MAVSDK = "b" * 40


def release_identity() -> dict:
    return {
        "schema_version": 1,
        "release_version": "dev-" + SOURCE,
        "source_sha": SOURCE,
        "mavsdk_sha": MAVSDK,
        "official": False,
    }


def create_package(directory: Path, name: str, platform: str, architecture: str) -> Path:
    embedded = identity.component_identity(release_identity(), name, platform, architecture)
    required = {
        "plugin": ["NOMADPlugin.dll"],
        "router": ["nomad-link-router.exe", "Nomad.LinkRouter.dll"],
        "core": list(manifest.CORE_REQUIRED_FILES)
        + (["bin/nomad.exe", "bin/nomad-runtime.exe"] if platform == "windows" else ["bin/nomad", "bin/nomad-runtime"]),
    }
    embedded["required_files"] = ["package-identity.json", *required[name]]
    package = directory / f"{name}-{platform}-{architecture}.zip"
    with zipfile.ZipFile(package, "w") as archive:
        archive.writestr("package-identity.json", json.dumps(embedded))
        for filename in required[name]:
            archive.writestr(filename, name + platform)
    return package


def create_release(directory: Path, monkeypatch) -> Path:
    monkeypatch.setattr(identity, "get_identity", lambda tag="": release_identity())
    for name, platform, architecture in sorted(manifest.EXPECTED):
        create_package(directory, name, platform, architecture)
    output = directory / "release-manifest.json"
    identity.aggregate(directory, output)
    return output


def test_complete_set_correlates_every_package_and_tools(tmp_path, monkeypatch):
    output = create_release(tmp_path, monkeypatch)
    loaded = manifest.load_manifest(output)
    assert len(loaded["components"]) == 4
    for entry in loaded["components"]:
        embedded = manifest.verify_package(entry, tmp_path / entry["filename"])
        assert embedded["source_sha"] == SOURCE
    tools = loaded["deployment_tools"]
    assert manifest.digest_file(tmp_path / tools["filename"]) == tools["sha256"]
    sums = (tmp_path / "SHA256SUMS").read_text()
    assert len(sums.splitlines()) == 6
    identity.aggregate(tmp_path, output)
    assert manifest.load_manifest(output) == loaded


def test_missing_component_cannot_be_published(tmp_path, monkeypatch):
    monkeypatch.setattr(identity, "get_identity", lambda tag="": release_identity())
    create_package(tmp_path, "plugin", "windows", "any")
    with pytest.raises(ValueError, match="all four"):
        identity.aggregate(tmp_path, tmp_path / "release-manifest.json")


def test_corrupt_package_fails_before_execution(tmp_path, monkeypatch):
    output = create_release(tmp_path, monkeypatch)
    entry = manifest.select_component(manifest.load_manifest(output), "core", "linux")
    package = tmp_path / entry["filename"]
    package.write_bytes(package.read_bytes() + b"corrupt")
    with pytest.raises(ValueError, match="SHA-256"):
        manifest.verify_package(entry, package)


def test_artifact_from_different_source_cannot_join_release(tmp_path, monkeypatch):
    output = create_release(tmp_path, monkeypatch)
    monkeypatch.setattr(identity, "get_identity", lambda tag="": dict(release_identity(), source_sha="c" * 40))
    with pytest.raises(ValueError, match="source identity"):
        identity.aggregate(tmp_path, output)


@pytest.mark.parametrize(
    "field,value", [("official", True), ("source_sha", "short"), ("release_version", "main"), ("schema_version", 2)]
)
def test_malformed_release_identity_is_rejected(tmp_path, monkeypatch, field, value):
    output = create_release(tmp_path, monkeypatch)
    document = json.loads(output.read_text())
    document[field] = value
    output.write_text(json.dumps(document))
    with pytest.raises(ValueError):
        manifest.load_manifest(output)


def test_unsupported_platform_protocol_and_target_are_rejected(tmp_path, monkeypatch):
    output = create_release(tmp_path, monkeypatch)
    document = json.loads(output.read_text())
    plugin = next(entry for entry in document["components"] if entry["name"] == "plugin")
    for field, value in [
        ("platform", "macos"),
        ("protocol_versions", {"runtime_ipc": 2}),
        ("mission_planner_target", "1.0"),
    ]:
        altered = json.loads(json.dumps(document))
        next(entry for entry in altered["components"] if entry["name"] == "plugin")[field] = value
        output.write_text(json.dumps(altered))
        with pytest.raises(ValueError):
            manifest.load_manifest(output)
    assert plugin["mission_planner_target"] == "1.3.83"


@pytest.mark.parametrize("filename", ["../escape", "/absolute", "C:/absolute", "directory\\escape"])
def test_unsafe_archive_member_is_rejected(tmp_path, filename):
    package = tmp_path / "unsafe.zip"
    with zipfile.ZipFile(package, "w") as archive:
        archive.writestr(filename, "unsafe")
    with pytest.raises(ValueError):
        manifest.read_package_identity(package)


def test_workflow_publication_requires_both_platforms_and_complete_aggregation():
    workflow = (Path(__file__).resolve().parents[1] / ".github/workflows/release.yml").read_text()
    assert "needs: [core, windows-clients]" in workflow
    assert "os: ubuntu-latest" in workflow and "os: windows-latest" in workflow
    assert "scripts/release/identity.py aggregate" in workflow
    assert "if: github.event_name == 'push' && startsWith(github.ref, 'refs/tags/')" in workflow
    assert "fail_on_unmatched_files: true" in workflow


def test_official_identity_requires_exact_clean_semantic_tag(monkeypatch):
    def fake_git(*arguments):
        if arguments == ("rev-parse", "HEAD") or arguments == ("rev-parse", "v2.3.4^{commit}"):
            return SOURCE
        if arguments[0] == "ls-tree":
            return f"160000 commit {MAVSDK}\tthird_party/MAVSDK"
        if arguments[0] == "status":
            return ""
        raise ValueError("unknown tag")

    monkeypatch.setattr(identity, "git", fake_git)
    official = identity.get_identity("v2.3.4")
    assert official["official"] is True
    assert identity.component_identity(official, "core", "linux", "x86_64")["version"] == "2.3.4"
    with pytest.raises(ValueError, match="vX.Y.Z"):
        identity.get_identity("vlatest")
    monkeypatch.setattr(
        identity, "git", lambda *arguments: " M CMakeLists.txt" if arguments[0] == "status" else fake_git(*arguments)
    )
    with pytest.raises(ValueError, match="unchanged"):
        identity.get_identity("v2.3.4")
    assert identity.get_identity()["source_dirty"] is True


def test_required_payload_must_actually_exist(tmp_path, monkeypatch):
    output = create_release(tmp_path, monkeypatch)
    document = manifest.load_manifest(output)
    entry = manifest.select_component(document, "plugin", "windows", "any")
    package = tmp_path / entry["filename"]
    embedded = manifest.read_package_identity(package)
    with zipfile.ZipFile(package, "w") as archive:
        archive.writestr("package-identity.json", json.dumps(embedded))
    entry["sha256"] = manifest.digest_file(package)
    with pytest.raises(ValueError, match="missing required"):
        manifest.verify_package(entry, package)


def test_case_colliding_archive_entries_are_rejected(tmp_path, monkeypatch):
    output = create_release(tmp_path, monkeypatch)
    entry = manifest.select_component(manifest.load_manifest(output), "plugin", "windows", "any")
    package = tmp_path / entry["filename"]
    with zipfile.ZipFile(package, "a") as archive:
        archive.writestr("nomadplugin.dll", "alias")
    entry["sha256"] = manifest.digest_file(package)
    with pytest.raises(ValueError, match="duplicate"):
        manifest.verify_package(entry, package)


def test_duplicate_json_keys_are_rejected(tmp_path):
    path = tmp_path / "manifest.json"
    path.write_text('{"schema_version": 1, "schema_version": 1}')
    with pytest.raises(ValueError, match="duplicate JSON"):
        manifest.load_manifest(path)


def test_embedded_identity_is_bounded_before_decompression(tmp_path):
    path = tmp_path / "oversize.zip"
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("package-identity.json", b" " * (manifest.MAX_IDENTITY_BYTES + 1))
    with pytest.raises(ValueError, match="64 KiB"):
        manifest.read_package_identity(path)


@pytest.mark.parametrize("value", [True, "1", None, 2])
def test_protocol_version_has_strict_integer_type(tmp_path, monkeypatch, value):
    output = create_release(tmp_path, monkeypatch)
    document = json.loads(output.read_text())
    next(entry for entry in document["components"] if entry["name"] == "plugin")["protocol_versions"] = {
        "runtime_ipc": value
    }
    output.write_text(json.dumps(document))
    with pytest.raises(ValueError, match="protocol"):
        manifest.load_manifest(output)


@pytest.mark.skipif(not shutil.which("cmake") or not shutil.which("git"), reason="CMake/Git identity check")
def test_cmake_uses_requested_tag_when_commit_has_multiple_tags(tmp_path):
    cmake = tmp_path / "cmake"
    cmake.mkdir()
    shutil.copyfile(identity.ROOT / "cmake/ReleaseIdentity.cmake", cmake / "ReleaseIdentity.cmake")
    (tmp_path / "third_party/MAVSDK").mkdir(parents=True)
    (tmp_path / ".gitignore").write_text("identity.txt\n")
    script = cmake / "read.cmake"
    script.write_text(
        'include("${CMAKE_CURRENT_LIST_DIR}/ReleaseIdentity.cmake")\n'
        'file(WRITE "${CMAKE_CURRENT_LIST_DIR}/../identity.txt" "${NOMAD_RELEASE_VERSION}")\n'
    )
    environment = dict(
        os.environ,
        GIT_AUTHOR_NAME="Fixture",
        GIT_COMMITTER_NAME="Fixture",
        GIT_AUTHOR_EMAIL="fixture@example.invalid",
        GIT_COMMITTER_EMAIL="fixture@example.invalid",
    )
    commands = [
        ["init", "--quiet"],
        ["add", "cmake", ".gitignore"],
        ["update-index", "--add", "--cacheinfo", "160000," + MAVSDK + ",third_party/MAVSDK"],
        ["commit", "--quiet", "-m", "Fixture"],
        ["tag", "v1.2.3"],
        ["tag", "v2.3.4"],
    ]
    for command in commands:
        subprocess.run(["git", "-C", str(tmp_path), *command], check=True, capture_output=True, env=environment)
    environment.update(GITHUB_REF_TYPE="tag", GITHUB_REF_NAME="v2.3.4")
    result = subprocess.run(["cmake", "-P", str(script)], capture_output=True, text=True, env=environment, timeout=10)
    assert result.returncode == 0, result.stderr
    assert (tmp_path / "identity.txt").read_text() == "v2.3.4"


@pytest.mark.skipif(not shutil.which("cmake"), reason="CMake source-archive check")
def test_source_archive_has_honest_nonrelease_identity(tmp_path):
    cmake = tmp_path / "cmake"
    cmake.mkdir()
    shutil.copyfile(identity.ROOT / "cmake/ReleaseIdentity.cmake", cmake / "ReleaseIdentity.cmake")
    script = cmake / "read.cmake"
    script.write_text(
        'include("${CMAKE_CURRENT_LIST_DIR}/ReleaseIdentity.cmake")\n'
        "if(NOMAD_OFFICIAL OR NOT NOMAD_SOURCE_ARCHIVE)\n"
        'message(FATAL_ERROR "Source archive impersonates a release")\nendif()\n'
        'file(WRITE "${CMAKE_CURRENT_LIST_DIR}/../identity.txt" "${NOMAD_COMPONENT_VERSION}")\n'
    )
    result = subprocess.run(["cmake", "-P", str(script)], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert (tmp_path / "identity.txt").read_text() == "0.0.0-development"
    for name in ("sim-ros", "jetson", "sim-isaac"):
        dockerfile = (identity.ROOT / "docker" / ("Dockerfile." + name)).read_text(encoding="utf-8")
        assert "COPY cmake/ /ws/src/nomad/cmake/" in dockerfile
    assert "NOMAD_FIXTURE_VERSION OR NOMAD_SOURCE_ARCHIVE" in (identity.ROOT / "CMakeLists.txt").read_text()
