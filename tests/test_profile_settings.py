# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Profile ownership preserves deployment wiring without exporting it."""

from pathlib import Path

from scripts import profile
from scripts.profile_settings import DEPLOYMENT_KEYS, PROFILE_KEYS


def test_reviewed_example_keys_have_one_owner() -> None:
    example = profile.read_env_file(profile.REPO_ROOT / "config/nomad.env.example")
    assert PROFILE_KEYS.isdisjoint(DEPLOYMENT_KEYS)
    assert set(example) <= PROFILE_KEYS | DEPLOYMENT_KEYS
    for name in profile.PROFILES:
        assert set(profile.read_env_file(profile.PROFILES_DIR / f"{name}.env")) <= PROFILE_KEYS


def test_load_retains_all_reviewed_locals_and_replaces_owned_values(tmp_path: Path, monkeypatch) -> None:
    active = tmp_path / "nomad.env"
    # Every supported local assignment, including quotes/backslashes, must survive exactly.
    locals_text = "\n".join(f'{key}="local value\\{key}"' for key in sorted(DEPLOYMENT_KEYS))
    active.write_text(
        locals_text + "\nNOMAD_PROFILE=old\nNOMAD_SIM_MODE=true\n"
        "NOMAD_ENABLE_SERVOS=true\nNOMAD_AUTOSTART_EDGE_CORE=true\nUNKNOWN_LEGACY=value\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(profile, "ENV_FILE", active)
    monkeypatch.setattr(profile, "prepare_config", lambda *_: None)
    profile.cmd_load("groundstation_minimal")
    result = active.read_text(encoding="utf-8")
    assert set(locals_text.splitlines()) <= set(result.splitlines())
    env = profile.read_env_text(result)
    assert env["NOMAD_PROFILE"] == "groundstation_minimal"
    assert env["NOMAD_SIM_MODE"] == "false"
    assert env["NOMAD_AUTOSTART_MAVLINK_ROUTER"] == "false"
    assert set(env) <= PROFILE_KEYS | DEPLOYMENT_KEYS


def test_missing_active_env_uses_reviewed_local_defaults(tmp_path: Path, monkeypatch) -> None:
    active = tmp_path / "nomad.env"
    monkeypatch.setattr(profile, "ENV_FILE", active)
    monkeypatch.setattr(profile, "prepare_config", lambda *_: None)
    profile.cmd_load("groundstation_gpu")
    env = profile.read_env_file(active)
    defaults = profile.read_env_file(profile.REPO_ROOT / "config/nomad.env.example")
    assert {key: value for key, value in defaults.items() if key in DEPLOYMENT_KEYS}.items() <= env.items()
    assert env["NOMAD_MAVLINK_ENDPOINT"] == "udpin:127.0.0.1:14601"
    assert env["NOMAD_CLIENT_CREDENTIALS_FILE"] == ""
    assert env["NOMAD_AUDIT_DIRECTORY"] == ""
    assert env["NOMAD_API_KEY"] == ""
    assert not list(tmp_path.glob("nomad.env.bak.*"))


def test_save_load_roundtrip_does_not_export_local_state(tmp_path: Path, monkeypatch) -> None:
    name = "groundstation_minimal"
    profiles = tmp_path / "profiles"
    profiles.mkdir()
    template = profiles / f"{name}.env"
    template.write_text(
        (profile.PROFILES_DIR / f"{name}.env").read_text(encoding="utf-8")
        + "NOMAD_CLIENT_CREDENTIALS_FILE=\nNOMAD_AUDIT_DIRECTORY=\nNOMAD_API_KEY=\n",
        encoding="utf-8",
    )
    active = tmp_path / "nomad.env"
    local = (
        'NOMAD_CLIENT_CREDENTIALS_FILE="/srv/nomad/auth/client credentials.json"\n'
        "NOMAD_AUDIT_DIRECTORY=/srv/nomad/audit\nNOMAD_CLIENT_CREDENTIAL=private-test-client\n"
        "NOMAD_API_KEY=private-test-enable\nNOMAD_RUNTIME_IPC_PORT=14621\n"
        "NOMAD_LOG_DIR=/srv/nomad/log\nNOMAD_DATA_DIR=/srv/nomad/data\nNOMAD_RUN_DIR=/run/test-nomad\n"
        'GCS_IP=100.64.0.10\nGCS_EXTRA_IPS="100.64.0.11 100.64.0.12"\nGCS_PORT_LTE=14561\n'
        "GCS_PORT_LOCAL=14551\nMAVLINK_UART_DEV=/dev/test-uart\nMAVLINK_UART_BAUD=115200\n"
        "MEDIAMTX_CONFIG=/srv/nomad/media.yml\nISAAC_CONTAINER_NAME=test-container\n"
    )
    active.write_text(template.read_text(encoding="utf-8") + local + "NOMAD_SIM_MODE=true\n", encoding="utf-8")
    monkeypatch.setattr(profile, "ENV_FILE", active)
    monkeypatch.setattr(profile, "PROFILES_DIR", profiles)
    monkeypatch.setattr(profile, "prepare_config", lambda *_: None)
    monkeypatch.setattr("builtins.input", lambda _: "y")
    profile.cmd_save(name)
    assert set(profile.read_env_file(template)) <= PROFILE_KEYS
    assert "private-test" not in template.read_text(encoding="utf-8")
    profile.cmd_load(name)
    assert set(local.splitlines()) <= set(active.read_text(encoding="utf-8").splitlines())
    assert profile.read_env_file(active)["NOMAD_SIM_MODE"] == "true"


def test_diff_reports_owned_changes_without_deployment_state(tmp_path: Path, monkeypatch, capsys) -> None:
    profiles = tmp_path / "profiles"
    profiles.mkdir()
    name = "groundstation_minimal"
    (profiles / f"{name}.env").write_text(f"NOMAD_PROFILE={name}\nNOMAD_SIM_MODE=false\n", encoding="utf-8")
    active = tmp_path / "nomad.env"
    local = {
        "NOMAD_API_KEY": "private-test-enable",
        "NOMAD_CLIENT_CREDENTIAL": "private-test-client",
        "NOMAD_CLIENT_CREDENTIALS_FILE": "/srv/private/credentials.json",
        "NOMAD_AUDIT_DIRECTORY": "/srv/private/audit",
        "GCS_IP": "100.64.0.10",
    }
    active.write_text(
        f"NOMAD_PROFILE={name}\nNOMAD_SIM_MODE=true\n" + "\n".join(f"{key}={value}" for key, value in local.items()),
        encoding="utf-8",
    )
    original = active.read_bytes()
    monkeypatch.setattr(profile, "ENV_FILE", active)
    monkeypatch.setattr(profile, "PROFILES_DIR", profiles)
    profile.cmd_diff(name)
    output = capsys.readouterr().out
    assert "-NOMAD_SIM_MODE=true" in output
    assert "+NOMAD_SIM_MODE=false" in output
    for key, value in local.items():
        assert key not in output
        assert value not in output
    assert active.read_bytes() == original
