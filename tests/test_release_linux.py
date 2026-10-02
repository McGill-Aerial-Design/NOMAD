# SPDX-License-Identifier: Apache-2.0
"""POSIX activation pointer semantics without systemd privileges."""

import json
import os
import subprocess
import zipfile
from pathlib import Path

import pytest
from test_release_windows import release_bundle

from scripts.release import storage
from scripts.release.lifecycle import Deployment
from scripts.release.linux import SystemdAdapter


def stage_core_fixture(engine, document):
    metadata = json.loads(document.read_text())
    entry = next(item for item in metadata["components"] if item["name"] == "core" and item["platform"] == "linux")
    package = document.parent / entry["filename"]
    shared = {key: value for key, value in metadata.items() if key != "components"}
    with zipfile.ZipFile(package, "w") as output:
        output.writestr("package-identity.json", json.dumps({**shared, **entry}))
        for name in entry["required_files"]:
            if name != "package-identity.json":
                output.writestr(name, "fixture payload")
    entry["sha256"] = storage.digest(package)
    document.write_text(json.dumps(metadata))
    return engine.stage(document, package, "linux")


@pytest.mark.skipif(os.name == "nt", reason="POSIX symlink activation")
def test_linux_atomic_pointer_and_exact_rollback(tmp_path):
    root = tmp_path / "deployment"
    config = tmp_path / "runtime.json"
    config.write_text("external configuration")
    engine = Deployment(root, "core")
    a, b = release_bundle(tmp_path / "A", 1), release_bundle(tmp_path / "B", 2)
    records = []
    for document, _, _ in (a, b):
        records.append(stage_core_fixture(engine, document))
    state = {"service": "inactive"}

    def run(arguments, **_):
        operation = arguments[1]
        if operation == "show":
            value = (
                f"{root}/core/current/bin/nomad-runtime --config {config}"
                if "--property=ExecStart" in arguments
                else state["service"]
            )
            return subprocess.CompletedProcess(arguments, 0, value + "\n", "")
        if operation in {"start", "stop"}:
            state["service"] = "active" if operation == "start" else "inactive"
        return subprocess.CompletedProcess(arguments, 0, "", "")

    adapter = SystemdAdapter(root, config, 1, runner=run)
    adapter.health = lambda candidate: None
    engine.activate(a[2], adapter)
    original = adapter.pointer.readlink()
    assert original == Path(records[0]["install_path"])
    engine.activate(b[2], adapter)
    assert adapter.pointer.readlink() == Path(records[1]["install_path"])
    engine.rollback(adapter)
    assert adapter.pointer.readlink() == original
    assert engine.status()["status"] == "rolled_back"
    assert config.read_text() == "external configuration"


@pytest.mark.skipif(os.name == "nt", reason="POSIX ownership and modes")
def test_protected_root_rejects_writable_descendant(tmp_path):
    from scripts.release.deploy import protect_root

    root = tmp_path / "deployment"
    root.mkdir(mode=0o755)
    child = root / "core"
    child.mkdir(mode=0o755)
    record = child / "deployment.json"
    record.write_text("{}")
    record.chmod(0o666)
    with pytest.raises(ValueError, match="operator-owned"):
        protect_root(root)


@pytest.mark.parametrize("name", ["../outside", "/absolute", "C:/drive", "..\\escape", "CON", "trailing."])
def test_extract_rejects_unsafe_archive_paths(tmp_path, name):
    archive = tmp_path / "unsafe.zip"
    with zipfile.ZipFile(archive, "w") as output:
        output.writestr(name, "payload")
    destination = tmp_path / "extract"
    destination.mkdir()
    with pytest.raises(ValueError):
        storage.extract(archive, destination)
    assert not list(destination.iterdir())


def test_extract_rejects_case_collisions_and_links(tmp_path):
    archive = tmp_path / "unsafe.zip"
    with zipfile.ZipFile(archive, "w") as output:
        output.writestr("File", "one")
        output.writestr("file", "two")
    with pytest.raises(ValueError, match="duplicate"):
        storage.extract(archive, tmp_path)
    link = zipfile.ZipInfo("link")
    link.external_attr = 0o120777 << 16
    with zipfile.ZipFile(archive, "w") as output:
        output.writestr(link, "outside")
    with pytest.raises(ValueError, match="links"):
        storage.extract(archive, tmp_path)
