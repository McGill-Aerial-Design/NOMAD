# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Structural and falsification checks for the runtime authority SITL probe."""

from __future__ import annotations

import json
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "dev"))

import authority_sitl_observer as sitl_observer  # noqa: E402
import authority_sitl_relay as sitl_relay  # noqa: E402
import core_sitl_authority as authority  # noqa: E402


def isolated_container() -> dict[str, object]:
    return {
        "Name": "/nomad-authority-sitl",
        "Config": {
            "Image": sitl_observer.IMAGE,
            "Labels": {"com.docker.compose.project": "nomad-authority", "com.docker.compose.service": "sitl"},
            "Env": [
                "SITL_UDP_OUTPUT_ADDRESS=udp:host.docker.internal:14690 --out udp:host.docker.internal:14691",
                "SITL_FENCE_DEFAULTS=/nomad/sitl-fence.parm",
                "SITL_STREAM_DEFAULTS=/nomad/sitl-streams.parm",
                "VEHICLE=ArduCopter",
                "MODEL=quad",
                "SPEEDUP=1",
            ],
            "Entrypoint": ["/bin/sh", "/nomad/sitl-entrypoint.sh"],
        },
        "State": {"Running": True},
        "Mounts": [
            {"Destination": path, "RW": False}
            for path in (
                "/nomad/sitl-fence.parm",
                "/nomad/sitl-streams.parm",
                "/nomad/sitl-entrypoint.sh",
            )
        ],
    }


def test_authority_probe_is_bounded_isolated_and_ordered_within_copter_job() -> None:
    yaml = pytest.importorskip("yaml")
    workflow = yaml.safe_load((ROOT / ".github" / "workflows" / "sitl.yml").read_text(encoding="utf-8"))
    steps = workflow["jobs"]["sitl"]["steps"]
    probe_index = next(
        i
        for i, step in enumerate(steps)
        if step.get("name") == "Qualify runtime authority and independent source on isolated Copter"
    )
    copter_index = next(i for i, step in enumerate(steps) if step.get("run") == "pixi run --skip-deps core-sitl-status")
    step = steps[probe_index]
    assert probe_index < copter_index
    assert steps[probe_index - 1]["name"] == "Build runtime and qualification tools once"
    assert step["timeout-minutes"] == 6
    script = step["run"]
    assert "set -euo pipefail" in script
    assert "-p nomad-authority" in script and "--rm --name nomad-authority-sitl" in script
    assert 'SITL_UDP_OUTPUT_ADDRESS="udp:host.docker.internal:14690' in script
    assert "--out udp:host.docker.internal:14691" in script
    assert "pixi run --frozen python scripts/dev/core_sitl_authority.py" in script
    assert 'authority_container=""' in script
    assert 'if [[ -n "$authority_container" ]]; then\n' in script
    assert "authority_container=$(docker compose" in script
    assert 'docker stop "$authority_container"' in script
    source = (ROOT / "scripts" / "dev" / "core_sitl_authority.py").read_text(encoding="utf-8")
    assert "verify_isolated_simulator()" in source
    assert ".param_set" not in source and "param_set_send" not in source


@pytest.mark.skipif(sys.platform == "win32", reason="CI cleanup shell requires native Bash")
def test_authority_job_creation_failure_cannot_stop_a_preexisting_container() -> None:
    yaml = pytest.importorskip("yaml")
    bash = shutil.which("bash")
    assert bash is not None, "CI cleanup falsification requires Bash"
    workflow = yaml.safe_load((ROOT / ".github" / "workflows" / "sitl.yml").read_text(encoding="utf-8"))
    steps = [step for job in workflow["jobs"].values() for step in job.get("steps", [])]
    script = next(
        step["run"]
        for step in steps
        if step.get("name") == "Qualify runtime authority and independent source on isolated Copter"
    )
    fake_docker = """
docker() {
  if [[ "$1" == "compose" ]]; then
    return 7
  fi
  printf 'unexpected_cleanup\\n'
  return 0
}
"""
    result = subprocess.run([bash, "-c", fake_docker + script], capture_output=True, text=True, timeout=5)
    assert result.returncode == 7
    assert "unexpected_cleanup" not in result.stdout


def test_container_guard_rejects_wrong_image_revision_and_rw_profile(monkeypatch: pytest.MonkeyPatch) -> None:
    details = isolated_container()
    sitl_observer.validate_container(details)
    wrong_image = json.loads(json.dumps(details))
    wrong_image["Config"]["Image"] = "other-image"
    with pytest.raises(sitl_observer.ProbeError, match="wrong_firmware_image"):
        sitl_observer.validate_container(wrong_image)
    wrong_topology = json.loads(json.dumps(details))
    wrong_topology["Name"] = "/unrelated-sitl"
    with pytest.raises(sitl_observer.ProbeError, match="wrong_sitl_topology"):
        sitl_observer.validate_container(wrong_topology)
    writable = json.loads(json.dumps(details))
    writable["Mounts"][0]["RW"] = True
    with pytest.raises(sitl_observer.ProbeError, match="simulator_profile_not_read_only"):
        sitl_observer.validate_container(writable)

    responses = [json.dumps([details]), "wrong-revision"]
    monkeypatch.setattr(sitl_observer, "run_guard_command", lambda _args: responses.pop(0))
    with pytest.raises(sitl_observer.ProbeError, match="wrong_firmware_revision"):
        sitl_observer.verify_isolated_simulator()


class FakeMessage:
    def __init__(self, kind: str, system: int = 1, component: int = 1, **fields: object) -> None:
        self.kind, self.system, self.component = kind, system, component
        self.__dict__.update(fields)

    def get_type(self) -> str:
        return self.kind

    def get_srcSystem(self) -> int:
        return self.system

    def get_srcComponent(self) -> int:
        return self.component


def test_observer_filters_foreign_sources_and_uses_only_port_zero_servo_output() -> None:
    observer = sitl_observer.AuthorityObserver(SimpleNamespace(close=lambda: None))
    observer.record(FakeMessage("SERVO_OUTPUT_RAW", system=250, port=0, servo5_raw=1500), 1.0)
    observer.record(FakeMessage("SERVO_OUTPUT_RAW", port=1, servo5_raw=1500), 2.0)
    observer.record(FakeMessage("COMMAND_ACK", command=authority.SET_SERVO), 3.0)
    assert list(observer.servo5) == []
    observer.record(FakeMessage("SERVO_OUTPUT_RAW", port=0, servo5_raw=1525), 4.0)
    assert list(observer.servo5) == [(4.0, 1525)]


def test_observer_rejects_an_armed_ownship_heartbeat() -> None:
    observer = sitl_observer.AuthorityObserver(SimpleNamespace(close=lambda: None))
    armed = sitl_observer.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    heartbeat = FakeMessage(
        "HEARTBEAT",
        type=sitl_observer.mavutil.mavlink.MAV_TYPE_QUADROTOR,
        autopilot=sitl_observer.mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode=armed,
        custom_mode=0,
    )
    with pytest.raises(sitl_observer.ProbeError, match="aircraft_armed"):
        observer.record(heartbeat, time.monotonic())


def test_observer_rejects_stale_heartbeat_and_historical_mode_does_not_pass() -> None:
    observer = sitl_observer.AuthorityObserver(SimpleNamespace(close=lambda: None))
    observer.heartbeats.extend([(time.monotonic() - 5, 15, False)])
    with pytest.raises(sitl_observer.ProbeError, match="stale_observer_heartbeat"):
        observer.require_disarmed()

    boundary = time.monotonic()
    observer.heartbeats.extend([(boundary - 0.2, 15, False), (boundary - 0.1, 15, False)])
    observer.pump = lambda _timeout=0.2: observer.heartbeats.append((time.monotonic(), 0, False))
    with pytest.raises(sitl_observer.ProbeError, match="external_mode_change_not_observed"):
        observer.wait_mode(15, boundary, timeout=0.03)


def test_retry_ack_is_source_and_command_filtered_and_fence_counts_wire_frames() -> None:
    good = FakeMessage("COMMAND_ACK", command=authority.SET_SERVO)
    wrong_command = FakeMessage("COMMAND_ACK", command=authority.mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    wrong_source = FakeMessage("COMMAND_ACK", system=250, command=authority.SET_SERVO)
    assert sitl_relay.is_servo_ack([good])
    assert not sitl_relay.is_servo_ack([wrong_command, wrong_source])
    authority.require_fence_count(4, 4)
    with pytest.raises(authority.ProbeError, match="revoked_command_retried_after_fence"):
        authority.require_fence_count(4, 5)


def test_summary_keeps_actual_parameter_and_runtime_source_readbacks() -> None:
    parameters = {"SERVO5_FUNCTION": 0.0, "MAV_OPTIONS": 0.0}
    source = {"system": 245, "component": 190}
    result = authority.summary("passed", "2026-01-01T00:00:00+00:00", {}, [], [], {"startup": parameters}, source)
    assert result["parameter_readbacks"] == {"startup": parameters}
    assert result["source_ids"]["runtime_command"] == source
    assert "runtime_incarnation" not in json.dumps(result)


def test_old_output_and_fresh_ack_cannot_pass_output_observation() -> None:
    observer = sitl_observer.AuthorityObserver(SimpleNamespace(close=lambda: None))
    boundary = time.monotonic()
    observer.servo5.extend([(boundary - 1.0, 1500)] * 3)

    def receive_ack(_timeout=0.2):
        observer.heartbeats.append((time.monotonic(), 0, False))
        observer.record(FakeMessage("COMMAND_ACK", command=authority.SET_SERVO), time.monotonic())

    observer.pump = receive_ack
    with pytest.raises(sitl_observer.ProbeError, match="servo_output_observation_failed"):
        observer.wait_servo5(1500, boundary, timeout=0.03)


def test_relay_counts_actual_udp_command_and_drops_fc_ack(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(sitl_relay, "RELAY_PORT", 0)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.bind(("127.0.0.1", 0))
        client.settimeout(1.0)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as fc:
            fc.bind(("127.0.0.1", 0))
            fc.settimeout(1.0)
            relay = sitl_relay.AuthorityRelay(client.getsockname()[1])
            address = ("127.0.0.1", relay.socket.getsockname()[1])
            encoder = authority.mavutil.mavlink.MAVLink(None, srcSystem=245, srcComponent=190)
            fc_encoder = authority.mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
            try:
                fc.sendto(b"learn isolated upstream", address)
                assert client.recv(4096) == b"learn isolated upstream"
                command = encoder.command_long_encode(1, 1, authority.SET_SERVO, 0, 5, 1500, 0, 0, 0, 0, 0)
                client.sendto(command.pack(encoder), address)
                assert fc.recv(4096) == command.pack(encoder)
                frames = authority.wait_frames(relay, 1, timeout=1.0)
                assert frames[0][1:] == (1500, 245, 190)
                relay.drop_acks = True
                ack = fc_encoder.command_ack_encode(authority.SET_SERVO, 0)
                fc.sendto(ack.pack(fc_encoder), address)
                deadline = time.monotonic() + 1.0
                while relay.dropped_acks == 0 and time.monotonic() < deadline:
                    time.sleep(0.01)
                assert relay.dropped_acks == 1, "real FC ACK was not filtered from runtime path"
            finally:
                relay.close()
