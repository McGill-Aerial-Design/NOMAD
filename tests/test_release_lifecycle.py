# SPDX-License-Identifier: Apache-2.0
"""Filesystem deployment failures retain known-good payloads and recovery intent."""

import json
import zipfile

import pytest
from test_release_windows import fake_install, release_bundle, stage_plugin

from scripts.release import lifecycle, storage


@pytest.fixture
def installed(tmp_path):
    adapter = fake_install(tmp_path / "Mission Planner")
    deployment = lifecycle.Deployment(tmp_path / "deployment", "plugin")
    a, b = release_bundle(tmp_path / "A", 1), release_bundle(tmp_path / "B", 2)
    for release in (a, b):
        stage_plugin(deployment, release)
    deployment.activate(a[2], adapter)
    return deployment, adapter, a, b


def test_commit_write_failure_automatically_restores_a(installed, monkeypatch):
    deployment, adapter, a, b = installed
    expected = adapter.target.read_bytes()
    write = storage.write_json
    failed = False

    def fail_commit(path, value):
        nonlocal failed
        if value.get("active", {}).get("release_version") == b[2] and value["pending"] is None and not failed:
            failed = True
            raise OSError("injected activation record write failure")
        write(path, value)

    monkeypatch.setattr(storage, "write_json", fail_commit)
    with pytest.raises(OSError, match="record write"):
        deployment.activate(b[2], adapter)
    state = deployment.status()
    assert state["active"]["release_version"] == a[2]
    assert state["pending"] is None
    assert state["status"] == "rolled_back"
    assert adapter.target.read_bytes() == expected


def test_write_failure_during_restore_leaves_deterministic_recovery(installed, monkeypatch):
    deployment, adapter, a, b = installed
    expected = adapter.target.read_bytes()
    write = storage.write_json

    def fail_after_switch(path, value):
        if value.get("status") == "rollback_pending" or value.get("active", {}).get("release_version") == b[2]:
            raise OSError("injected unavailable record storage")
        write(path, value)

    monkeypatch.setattr(storage, "write_json", fail_after_switch)
    with pytest.raises(OSError, match="storage"):
        deployment.activate(b[2], adapter)
    assert deployment.status()["pending"]["restore"]["release_version"] == a[2]
    assert adapter.target.read_bytes() != expected
    monkeypatch.setattr(storage, "write_json", write)
    recovered = deployment.recover(adapter)
    assert recovered["active"]["release_version"] == a[2]
    assert recovered["pending"] is None
    assert adapter.target.read_bytes() == expected


def test_refused_stop_never_switches_active_payload(installed):
    deployment, adapter, a, b = installed
    expected = adapter.target.read_bytes()
    normal_stop = adapter.stop

    def refuse_stop():
        raise RuntimeError("injected process refuses stop")

    adapter.stop = refuse_stop
    with pytest.raises(RuntimeError, match="refuses stop"):
        deployment.activate(b[2], adapter)
    assert adapter.target.read_bytes() == expected
    assert deployment.status()["status"] == "failed"
    assert deployment.status()["pending"] is not None
    adapter.stop = normal_stop
    assert deployment.recover(adapter)["active"]["release_version"] == a[2]


def test_actual_deployment_drift_is_rejected_before_journaling(installed):
    deployment, adapter, a, b = installed
    before = deployment.record.read_bytes()
    adapter.target.write_bytes(b"unmanaged replacement")
    with pytest.raises(ValueError, match="differs from the recorded"):
        deployment.activate(b[2], adapter)
    assert deployment.record.read_bytes() == before
    assert adapter.target.read_bytes() == b"unmanaged replacement"
    assert deployment.status()["active"]["release_version"] == a[2]


def test_missing_previous_never_reports_rollback_success(installed):
    deployment, adapter, a, b = installed
    deployment.activate(b[2], adapter)
    expected = adapter.target.read_bytes()
    lifecycle.remove_release(deployment.release_path(a[2]))
    with pytest.raises(FileNotFoundError):
        deployment.rollback(adapter)
    assert adapter.target.read_bytes() == expected
    assert deployment.status()["active"]["release_version"] == b[2]


def test_cleanup_protects_active_previous_and_operator_state(installed):
    deployment, adapter, a, b = installed
    deployment.activate(b[2], adapter)
    for version in (a[2], b[2]):
        with pytest.raises(ValueError, match="cannot delete"):
            deployment.cleanup(version)
    assert (adapter.directory / "settings.json").is_file()
    assert deployment.get_release(a[2])["release_version"] == a[2]


def test_stage_failure_keeps_active_a_and_removes_temporary_directory(installed, tmp_path, monkeypatch):
    deployment, adapter, a, _ = installed
    c = release_bundle(tmp_path / "C", 3)
    expected = adapter.target.read_bytes()

    def fail_extract(*_):
        raise OSError("injected staging failure")

    monkeypatch.setattr(storage, "extract", fail_extract)
    with pytest.raises(OSError, match="staging"):
        stage_plugin(deployment, c)
    assert adapter.target.read_bytes() == expected
    assert deployment.status()["active"]["release_version"] == a[2]
    assert not list(deployment.releases.glob(".stage-*"))


def test_manifest_incomplete_or_wrong_platform_is_rejected(installed, tmp_path):
    deployment, adapter, _, _ = installed
    expected = adapter.target.read_bytes()
    c = release_bundle(tmp_path / "C", 3)
    with pytest.raises(ValueError, match="absent or unsupported"):
        deployment.stage(c[0], c[1], "linux", "any")
    document = json.loads(c[0].read_text())
    document["components"].pop()
    c[0].write_text(json.dumps(document))
    with pytest.raises(ValueError, match="four required"):
        stage_plugin(deployment, c)
    assert adapter.target.read_bytes() == expected


def test_same_release_with_different_bytes_never_overwrites_version(installed, tmp_path):
    deployment, adapter, a, _ = installed
    duplicate = release_bundle(tmp_path / "duplicate-A", 1)
    with zipfile.ZipFile(duplicate[1]) as archive:
        contents = {name: archive.read(name) for name in archive.namelist()}
    contents["NOMADPlugin.dll"] += b"changed-release-bytes"
    with zipfile.ZipFile(duplicate[1], "w") as archive:
        for name, payload in contents.items():
            archive.writestr(name, payload)
    document = json.loads(duplicate[0].read_text())
    entry = next(item for item in document["components"] if item["name"] == "plugin")
    entry["sha256"] = storage.digest(duplicate[1])
    duplicate[0].write_text(json.dumps(document))
    expected = adapter.target.read_bytes()
    with pytest.raises(ValueError, match="already exists with different bytes"):
        stage_plugin(deployment, duplicate)
    assert adapter.target.read_bytes() == expected
    assert deployment.get_release(a[2])["artifact_digest"] != entry["sha256"]


def test_corrupt_previous_archive_prevents_unverified_rollback(installed):
    deployment, adapter, a, b = installed
    deployment.activate(b[2], adapter)
    previous = deployment.release_path(a[2]) / "package"
    previous.chmod(0o600)
    with previous.open("ab") as stream:
        stream.write(b"corruption")
    expected = adapter.target.read_bytes()
    with pytest.raises(ValueError, match="package digest mismatch"):
        deployment.rollback(adapter)
    assert adapter.target.read_bytes() == expected
    assert deployment.status()["active"]["release_version"] == b[2]


@pytest.mark.parametrize("key", ["NOMAD_CLIENT_CREDENTIALS_FILE", "NOMAD_AUDIT_DIRECTORY"])
def test_external_state_cannot_use_dot_paths_into_release_root(tmp_path, key):
    from scripts.release.deploy import external_config

    root = tmp_path / "deployment"
    root.mkdir()
    (tmp_path / "operator").mkdir()
    config = tmp_path / "operator" / "runtime.json"
    state = tmp_path / "operator" / ".." / "deployment" / "state"
    config.write_text(json.dumps({key: str(state)}))
    with pytest.raises(ValueError, match="outside deployment root"):
        external_config(root, config)
