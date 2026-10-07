# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Profile loads report final state and restore env after MP commit failure."""

from __future__ import annotations

import json
import stat
import sys
from pathlib import Path

import pytest

from scripts import profile, profile_application, profile_mission_planner

NAME = "groundstation_minimal"


@pytest.fixture
def targets(tmp_path: Path, monkeypatch):
    profiles = tmp_path / "profiles"
    profiles.mkdir()
    (profiles / f"{NAME}.env").write_text(
        f"NOMAD_PROFILE={NAME}\nNOMAD_MAVLINK_ENDPOINT=127.0.0.1:14550\nNOMAD_API_KEY=\n",
        encoding="utf-8",
    )
    active = tmp_path / "nomad.env"
    active.write_text(
        "NOMAD_PROFILE=old\nNOMAD_API_KEY=private-test-value\n"
        'NOMAD_CLIENT_CREDENTIALS_FILE="/srv/nomad/auth/client credentials.json"\n'
        "NOMAD_AUDIT_DIRECTORY=/srv/nomad/audit\nNOMAD_RUNTIME_IPC_PORT=14621\n"
        "GCS_IP=100.64.0.10\nGCS_EXTRA_IPS=100.64.0.11\nMAVLINK_UART_DEV=/dev/test-uart\n",
        encoding="utf-8",
    )
    mp = tmp_path / "mp.json"
    mp.write_text('{"ActiveProfile":"old","CoreClientCredential":"private-client-value"}', encoding="utf-8")
    monkeypatch.setattr(profile, "PROFILES_DIR", profiles)
    monkeypatch.setattr(profile, "ENV_FILE", active)
    monkeypatch.setenv("NOMAD_MP_CONFIG", str(mp))
    monkeypatch.setattr(sys, "argv", ["profile.py", "load", NAME])
    return active, mp


def run_load(capsys) -> tuple[int, list[str]]:
    status = 0
    try:
        profile.main()
    except SystemExit as exc:
        status = exc.code
    output = capsys.readouterr().out
    assert "private-test-value" not in output
    assert "private-client-value" not in output
    lines = [line for line in output.splitlines() if line.startswith(("[APPLIED]", "[SKIPPED]", "[FAILED]", "[OK]"))]
    return status, lines


def assert_no_temporary_files(root: Path) -> None:
    assert not list(root.rglob("*.tmp"))


def test_full_application_and_secret_preservation(targets, capsys) -> None:
    active, mp = targets
    status, lines = run_load(capsys)
    assert status == 0
    assert lines == [
        "[APPLIED] env: profile settings loaded (deployment-local settings preserved)",
        "[APPLIED] mission_planner: profile settings synced",
        f"[OK] Profile load completed: {NAME} (see target results above)",
    ]
    assert profile.read_env_file(active)["NOMAD_PROFILE"] == NAME
    assert profile.read_env_file(active)["NOMAD_API_KEY"] == "private-test-value"
    synced = json.loads(mp.read_text())
    assert synced["ActiveProfile"] == NAME
    assert synced["CoreClientCredential"] == "private-client-value"
    assert "NOMAD_API_KEY" not in synced
    assert len(list(active.parent.glob("nomad.env.bak.*"))) == 1
    assert_no_temporary_files(active.parent)


def test_unknown_mp_path_is_explicit_optional_skip(targets, monkeypatch, capsys) -> None:
    active, mp = targets
    original_mp = mp.read_bytes()
    monkeypatch.delenv("NOMAD_MP_CONFIG")
    monkeypatch.delenv("LOCALAPPDATA", raising=False)
    status, lines = run_load(capsys)
    assert status == 0
    assert lines == [
        "[APPLIED] env: profile settings loaded (deployment-local settings preserved)",
        "[SKIPPED] mission_planner: config path unavailable; set NOMAD_MP_CONFIG to sync",
        f"[OK] Profile load completed: {NAME} (see target results above)",
    ]
    assert profile.read_env_file(active)["NOMAD_PROFILE"] == NAME
    assert mp.read_bytes() == original_mp
    assert_no_temporary_files(active.parent)


@pytest.mark.parametrize(
    ("content", "reason"),
    [
        ("not json", "Mission Planner config is unreadable or malformed"),
        ("[]", "Mission Planner config is not a JSON object"),
        ('{"RouterMode":"Embedded"}', "RouterMode is unsupported; run only the standalone ground router"),
    ],
)
def test_mp_validation_prevents_env_change(targets, capsys, content: str, reason: str) -> None:
    active, mp = targets
    mp.write_text(content)
    original_env = active.read_bytes()
    status, lines = run_load(capsys)
    assert status == 1
    assert lines == [
        "[SKIPPED] env: Mission Planner preflight failed; unchanged",
        f"[FAILED] mission_planner: {reason}",
        f"[FAILED] Profile load: {NAME}",
    ]
    assert active.read_bytes() == original_env
    assert mp.read_text() == content
    assert not list(active.parent.glob("nomad.env.bak.*"))
    assert_no_temporary_files(active.parent)


@pytest.mark.parametrize("failed_target", ["env", "mission_planner"])
def test_staging_write_failure_leaves_both_targets_unchanged(targets, monkeypatch, capsys, failed_target) -> None:
    active, mp = targets
    originals = active.read_bytes(), mp.read_bytes()
    real_fsync = profile_application.os.fsync
    calls = 0

    def fail_write(descriptor):
        nonlocal calls
        calls += 1
        if calls == (2 if failed_target == "env" else 3):
            raise OSError("injected write failure")
        real_fsync(descriptor)

    monkeypatch.setattr(profile_application.os, "fsync", fail_write)
    status, lines = run_load(capsys)
    other = "mission_planner" if failed_target == "env" else "env"
    assert status == 1
    assert lines == [
        f"[FAILED] {failed_target}: preflight or backup failed; unchanged",
        f"[SKIPPED] {other}: application aborted; unchanged",
        f"[FAILED] Profile load: {NAME}",
    ]
    assert (active.read_bytes(), mp.read_bytes()) == originals
    assert not list(active.parent.glob("nomad.env.bak.*"))
    assert_no_temporary_files(active.parent)


@pytest.mark.parametrize("existing_env", [True, False])
def test_mp_commit_failure_restores_env(targets, monkeypatch, capsys, existing_env) -> None:
    active, mp = targets
    if not existing_env:
        active.unlink()
    original_env = active.read_bytes() if existing_env else None
    original_mp = mp.read_bytes()
    real_replace = Path.replace

    def fail_mp_commit(source, destination):
        if destination == mp:
            assert profile.read_env_file(active)["NOMAD_PROFILE"] == NAME
            raise PermissionError("injected MP replacement failure")
        return real_replace(source, destination)

    monkeypatch.setattr(Path, "replace", fail_mp_commit)
    status, lines = run_load(capsys)
    assert status == 1
    assert lines == [
        "[SKIPPED] env: rolled back after Mission Planner commit failed",
        "[FAILED] mission_planner: atomic replacement failed; unchanged",
        f"[FAILED] Profile load: {NAME}",
    ]
    assert (active.read_bytes() if active.exists() else None) == original_env
    assert mp.read_bytes() == original_mp
    if existing_env:
        assert next(active.parent.glob("nomad.env.bak.*")).read_bytes() == original_env
    assert_no_temporary_files(active.parent)


def test_env_commit_failure_does_not_commit_mp(targets, monkeypatch, capsys) -> None:
    active, mp = targets
    originals = active.read_bytes(), mp.read_bytes()
    real_replace = Path.replace

    def fail_env_commit(source, destination):
        if destination == active:
            raise PermissionError("injected env replacement failure")
        return real_replace(source, destination)

    monkeypatch.setattr(Path, "replace", fail_env_commit)
    status, lines = run_load(capsys)
    assert status == 1
    assert lines == [
        "[FAILED] env: atomic replacement failed; unchanged",
        "[SKIPPED] mission_planner: env commit failed; unchanged",
        f"[FAILED] Profile load: {NAME}",
    ]
    assert (active.read_bytes(), mp.read_bytes()) == originals
    assert_no_temporary_files(active.parent)


def test_rollback_failure_reports_changed_env(targets, monkeypatch, capsys) -> None:
    active, mp = targets
    original_env, original_mp = active.read_bytes(), mp.read_bytes()
    real_replace = Path.replace
    committed = False

    def fail_commit_and_rollback(source, destination):
        nonlocal committed
        if destination == mp or committed:
            raise PermissionError("injected replacement failure")
        committed = True
        return real_replace(source, destination)

    monkeypatch.setattr(Path, "replace", fail_commit_and_rollback)
    status, lines = run_load(capsys)
    assert status == 1
    assert lines[0] == (
        "[FAILED] env: changed; rollback failed; restore the env backup or remove newly created env before use"
    )
    assert lines[1:] == [
        "[FAILED] mission_planner: atomic replacement failed; unchanged",
        f"[FAILED] Profile load: {NAME}",
    ]
    assert profile.read_env_file(active)["NOMAD_PROFILE"] == NAME
    assert mp.read_bytes() == original_mp
    assert next(active.parent.glob("nomad.env.bak.*")).read_bytes() == original_env
    assert_no_temporary_files(active.parent)


def test_unreadable_mp_is_failure(targets, monkeypatch, capsys) -> None:
    active, mp = targets
    original_env = active.read_bytes()
    real_read = Path.read_text

    def fail_mp_read(path, *args, **kwargs):
        if path == mp:
            raise PermissionError("injected read failure")
        return real_read(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", fail_mp_read)
    status, lines = run_load(capsys)
    assert status == 1
    assert lines[1] == "[FAILED] mission_planner: Mission Planner config is unreadable or malformed"
    assert active.read_bytes() == original_env
    assert_no_temporary_files(active.parent)


def test_direct_mp_sync_propagates_write_failure_and_cleans_temp(targets, monkeypatch) -> None:
    active, mp = targets
    original = mp.read_bytes()
    monkeypatch.setattr(Path, "replace", lambda *_: (_ for _ in ()).throw(PermissionError("injected")))
    with pytest.raises(PermissionError):
        profile_mission_planner.sync_config(NAME, {"NOMAD_PROFILE": NAME})
    assert mp.read_bytes() == original
    assert_no_temporary_files(active.parent)


def test_credentials_are_preserved_verbatim(targets, capsys) -> None:
    active, _ = targets
    credentials = 'NOMAD_API_KEY="private-test-value\\with\\slashes"\nNOMAD_CLIENT_CREDENTIAL="private-client-value"\n'
    active.write_text(credentials, encoding="utf-8")
    status, _ = run_load(capsys)
    assert status == 0
    content = active.read_text(encoding="utf-8")
    for line in credentials.splitlines():
        assert line in content.splitlines()


def test_backup_failure_leaves_targets_unchanged(targets, monkeypatch, capsys) -> None:
    active, mp = targets
    originals = active.read_bytes(), mp.read_bytes()

    def fail_backup(source, destination):
        destination.write_bytes(b"incomplete")
        raise OSError("injected backup failure")

    monkeypatch.setattr(profile_application.shutil, "copy2", fail_backup)
    status, lines = run_load(capsys)
    assert status == 1
    assert lines == [
        "[FAILED] env: preflight or backup failed; unchanged",
        "[SKIPPED] mission_planner: application aborted; unchanged",
        f"[FAILED] Profile load: {NAME}",
    ]
    assert (active.read_bytes(), mp.read_bytes()) == originals
    assert not list(active.parent.glob("nomad.env.bak.*"))
    assert_no_temporary_files(active.parent)


def test_same_target_path_is_rejected_before_commit(targets, monkeypatch, capsys) -> None:
    active, _ = targets
    original = active.read_bytes()
    monkeypatch.setattr(profile, "prepare_config", lambda *_: (active, b"{}"))
    status, lines = run_load(capsys)
    assert status == 1
    assert lines[0] == "[FAILED] mission_planner: preflight or backup failed; unchanged"
    assert active.read_bytes() == original
    assert_no_temporary_files(active.parent)


@pytest.mark.skipif(sys.platform != "win32", reason="Windows read-only bit prevents atomic replacement")
def test_read_only_mp_failure_cleans_read_only_staged_file(targets, capsys) -> None:
    active, mp = targets
    originals = active.read_bytes(), mp.read_bytes()
    mp.chmod(stat.S_IREAD)
    try:
        status, lines = run_load(capsys)
        assert status == 1
        assert lines[0] == "[SKIPPED] env: rolled back after Mission Planner commit failed"
        assert (active.read_bytes(), mp.read_bytes()) == originals
        assert_no_temporary_files(active.parent)
    finally:
        mp.chmod(stat.S_IREAD | stat.S_IWRITE)
