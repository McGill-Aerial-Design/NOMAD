# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Falsification checks for the installed Copter LAND simulator qualification."""

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

import authority_sitl_relay as relay_module
import copter_land_sitl_observer as observer_module
import core_sitl_copter_land as scenario


class Message(SimpleNamespace):
    def get_type(self):
        return self.kind

    def get_srcSystem(self):
        return getattr(self, "system", 1)

    def get_srcComponent(self):
        return getattr(self, "component", 1)


def heartbeat(**fields):
    values = {
        "kind": "HEARTBEAT",
        "type": scenario.mavutil.mavlink.MAV_TYPE_QUADROTOR,
        "autopilot": scenario.mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        "base_mode": scenario.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
        "custom_mode": observer_module.GUIDED,
        **fields,
    }
    return Message(**values)


def airborne_observer():
    observer = observer_module.CopterLandObserver(SimpleNamespace(close=lambda: None))
    observer.record(heartbeat(), 9.9)
    observer.record(Message(kind="GLOBAL_POSITION_INT", relative_alt=5000), 9.9)
    observer.record(Message(kind="EXTENDED_SYS_STATE", landed_state=2), 9.9)
    return observer


def ground_observer():
    observer = observer_module.CopterLandObserver(SimpleNamespace(close=lambda: None))
    observer.record(heartbeat(base_mode=0, custom_mode=observer_module.LAND), 9.9)
    observer.record(Message(kind="EXTENDED_SYS_STATE", landed_state=1), 9.9)
    observer.positions.extend([(9.1 + index * 0.2, 0.05) for index in range(5)])
    return observer


def prearm_status(**fields):
    return Message(
        kind="SYS_STATUS",
        onboard_control_sensors_present=observer_module.PREARM_CHECK,
        onboard_control_sensors_enabled=observer_module.PREARM_CHECK,
        onboard_control_sensors_health=observer_module.PREARM_CHECK,
        **fields,
    )


def test_airborne_observer_filters_source_and_rejects_wrong_aircraft(monkeypatch):
    monkeypatch.setattr(observer_module.time, "monotonic", lambda: 10.0)
    observer = airborne_observer()
    assert observer.is_airborne(), "fresh armed GUIDED, 5 m and IN_AIR should establish known setup"
    observer.record(heartbeat(system=250, base_mode=0, custom_mode=9), 10.0)
    observer.record(Message(kind="GLOBAL_POSITION_INT", component=190, relative_alt=0), 10.0)
    assert observer.is_airborne(), "external source telemetry must not replace ownship state"
    with pytest.raises(scenario.ProbeError, match="wrong_aircraft_identity"):
        observer.record(heartbeat(type=scenario.mavutil.mavlink.MAV_TYPE_FIXED_WING), 10.0)


@pytest.mark.parametrize("stale_field", ["heartbeats", "positions", "landed_states"])
def test_airborne_setup_requires_each_fresh_authoritative_sample(monkeypatch, stale_field):
    monkeypatch.setattr(observer_module.time, "monotonic", lambda: 10.0)
    observer = airborne_observer()
    samples = getattr(observer, stale_field)
    sample = samples[-1]
    samples[-1] = (8.0, *sample[1:])
    assert not observer.is_airborne(), f"stale {stale_field} must reject airborne readiness"


def test_ack_alone_and_pre_boundary_state_do_not_establish_airborne_setup(monkeypatch):
    monkeypatch.setattr(observer_module.time, "monotonic", lambda: 10.0)
    observer = airborne_observer()
    observer.record(Message(kind="COMMAND_ACK", command=22, result=0), 10.0)
    assert not observer.is_airborne(9.95), "accepted takeoff ACK cannot substitute for newer observed state"
    observer.positions[-1] = (9.99, 3.9)
    assert not observer.is_airborne(), "armed flight below 4 m is not the known LAND starting envelope"


def test_external_setup_requires_fresh_3d_gps_before_one_shot_arm(monkeypatch):
    monkeypatch.setattr(observer_module.time, "monotonic", lambda: 10.0)
    observer = ground_observer()
    observer.record(prearm_status(), 9.9)
    assert not observer.is_ready_for_setup(), "startup with no GPS observation must not arm"
    observer.record(Message(kind="GPS_RAW_INT", fix_type=3), 8.0)
    assert not observer.is_ready_for_setup(), "historical 3D fix must not admit setup"
    observer.record(Message(kind="GPS_RAW_INT", fix_type=2), 9.9)
    assert not observer.is_ready_for_setup(), "2D fix must not admit setup"
    observer.record(Message(kind="GPS_RAW_INT", fix_type=3), 9.9)
    assert observer.is_ready_for_setup(), "fresh GPS and healthy pre-arm checks admit disarmed low-altitude setup"


@pytest.mark.parametrize("field", ["present", "enabled", "health"])
def test_external_setup_requires_enabled_healthy_prearm_checks(monkeypatch, field):
    monkeypatch.setattr(observer_module.time, "monotonic", lambda: 10.0)
    observer = ground_observer()
    observer.record(Message(kind="GPS_RAW_INT", fix_type=3), 9.9)
    assert not observer.is_ready_for_setup(), "GPS alone must not admit arming without controller pre-arm evidence"
    message = prearm_status()
    setattr(message, "onboard_control_sensors_" + field, 0)
    observer.record(message, 9.9)
    assert not observer.is_ready_for_setup(), f"pre-arm {field} missing must inhibit setup"
    observer.record(prearm_status(), 9.95)
    assert observer.is_ready_for_setup(), "fresh enabled healthy checks should admit the unchanged one-shot setup"


@pytest.mark.parametrize("observed_at, ready", [(8.5, True), (8.499, False), (10.0, True), (10.01, False)])
def test_external_setup_checks_prearm_age_boundaries(monkeypatch, observed_at, ready):
    monkeypatch.setattr(observer_module.time, "monotonic", lambda: 10.0)
    observer = ground_observer()
    observer.record(Message(kind="GPS_RAW_INT", fix_type=3), 9.9)
    observer.record(prearm_status(), observed_at)
    assert observer.is_ready_for_setup() is ready, "pre-arm freshness must include its boundary and reject future data"


@pytest.mark.parametrize("source", [{"system": 250}, {"component": 190}])
def test_external_setup_ignores_other_sources_prearm_checks(monkeypatch, source):
    monkeypatch.setattr(observer_module.time, "monotonic", lambda: 10.0)
    observer = ground_observer()
    observer.record(Message(kind="GPS_RAW_INT", fix_type=3), 9.9)
    observer.record(prearm_status(**source), 9.9)
    assert not observer.is_ready_for_setup(), "another source cannot supply the aircraft's pre-arm readiness"


def test_ground_confirmation_is_separate_fresh_and_requires_disarm(monkeypatch):
    monkeypatch.setattr(observer_module.time, "monotonic", lambda: 10.0)
    observer = ground_observer()
    assert observer.has_stable_ground_state(9.0)
    assert not observer.has_stable_ground_state(9.95), "ground data before the CLI result cannot complete later check"
    observer.heartbeats[-1] = (9.9, observer_module.LAND, True)
    assert not observer.has_stable_ground_state(9.0), "LAND and low altitude cannot substitute for disarm"
    observer.heartbeats[-1] = (9.9, observer_module.LAND, False)
    observer.landed_states[-1] = (9.9, 2)
    assert not observer.has_stable_ground_state(9.0), "IN_AIR cannot qualify the simulator ground result"


def test_ground_confirmation_accepts_10hz_only_after_stable_dwell(monkeypatch):
    monkeypatch.setattr(observer_module.time, "monotonic", lambda: 10.0)
    observer = ground_observer()
    observer.positions.clear()
    observer.positions.extend([(9.5 + index * 0.1, 0.05) for index in range(5)])
    assert not observer.has_stable_ground_state(9.0), "five 10 Hz samples span only 0.4 s"
    observer.positions.appendleft((9.3, 0.05))
    assert observer.has_stable_ground_state(9.0), "six fresh samples spanning 0.6 s should pass"
    assert len(observer.ground_samples(9.0)) == 6


@pytest.mark.parametrize("break_altitude", [0.6, 0.3])
def test_ground_confirmation_resets_after_high_or_unstable_altitude(monkeypatch, break_altitude):
    monkeypatch.setattr(observer_module.time, "monotonic", lambda: 10.0)
    observer = ground_observer()
    observer.positions.clear()
    observer.positions.extend([(9.0 + index * 0.1, 0.0) for index in range(5)])
    observer.positions.append((9.5, break_altitude))
    observer.positions.extend([(9.6 + index * 0.1, 0.0) for index in range(4)])
    assert not observer.has_stable_ground_state(8.9), "prior stable samples cannot bridge an unstable observation"


def land_command(observed_at=10.0, **fields):
    return {
        "observed_at": observed_at,
        "system": 245,
        "component": 190,
        "target_system": 1,
        "target_component": 1,
        "parameters": [0.0] * 7,
        **fields,
    }


def wire_evidence():
    return SimpleNamespace(
        land_commands=[land_command()], land_acknowledgements=[(10.02, 0)], heartbeats=[(10.8, observer_module.LAND)]
    )


def test_wire_evidence_preserves_sdk_attempts_and_reports_separate_timing():
    relay = wire_evidence()
    relay.land_commands.append(land_command(10.01))
    result = scenario.validate_land_wire(relay, 0.9, 10.9)
    assert result["wire_attempts"] == 2, "allowed transport retries must be recorded, not treated as CLI replay"
    assert result["wire_engagement_ms"] == pytest.approx(800)
    assert result["land_heartbeat_after_ack_ms"] == pytest.approx(780)
    assert result["cli_wall_ms"] == pytest.approx(900)
    assert "observed_at" not in json.dumps(result), "retain relative timing rather than temporary timestamps"


@pytest.mark.parametrize(
    ("field", "value", "error"),
    [
        ("target_system", 2, "runtime_land_target_invalid"),
        ("target_component", 190, "runtime_land_target_invalid"),
        ("parameters", [1.0] * 7, "runtime_land_parameters_invalid"),
    ],
)
def test_wire_rejects_commands_outside_qualified_land_contract(field, value, error):
    relay = wire_evidence()
    relay.land_commands[0][field] = value
    with pytest.raises(scenario.ProbeError, match=error):
        scenario.validate_land_wire(relay, 0.9, 10.9)


def test_wire_rejects_changed_runtime_source_or_missing_accepted_ack():
    relay = wire_evidence()
    relay.land_commands.append(land_command(10.01, system=250))
    with pytest.raises(scenario.ProbeError, match="runtime_land_source_changed"):
        scenario.validate_land_wire(relay, 0.9, 10.9)
    relay = wire_evidence()
    relay.land_acknowledgements = [(10.02, 2)]
    with pytest.raises(scenario.ProbeError, match="runtime_land_accepted_ack_missing"):
        scenario.validate_land_wire(relay, 0.9, 10.9)


@pytest.mark.parametrize("heartbeats", [[], [(10.01, observer_module.LAND)], [(10.8, observer_module.GUIDED)]])
def test_wire_ack_does_not_substitute_for_fresh_post_ack_land_heartbeat(heartbeats):
    relay = wire_evidence()
    relay.heartbeats = heartbeats
    with pytest.raises(scenario.ProbeError, match="runtime_land_post_ack_heartbeat_missing"):
        scenario.validate_land_wire(relay, 0.9, 10.9)


def test_wire_engagement_and_cli_overhead_have_independent_budgets():
    relay = wire_evidence()
    relay.heartbeats = [(13.0, observer_module.LAND)]
    with pytest.raises(scenario.ProbeError, match="land_engagement_budget_exceeded"):
        scenario.validate_land_wire(relay, 3.2, 13.2)
    relay = wire_evidence()
    with pytest.raises(scenario.ProbeError, match="land_engagement_budget_exceeded"):
        scenario.validate_land_wire(relay, 3.501, 13.6)


def test_relay_observes_actual_runtime_land_wire_and_only_ownship_ack_heartbeat(monkeypatch):
    monkeypatch.setattr(relay_module, "RELAY_PORT", 0)
    with (
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as runtime,
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as fc,
    ):
        runtime.bind(("127.0.0.1", 0))
        runtime.settimeout(1.0)
        fc.bind(("127.0.0.1", 0))
        fc.settimeout(1.0)
        relay = relay_module.AuthorityRelay(runtime.getsockname()[1])
        address = ("127.0.0.1", relay.socket.getsockname()[1])
        runtime_encoder = scenario.mavutil.mavlink.MAVLink(None, srcSystem=245, srcComponent=190)
        fc_encoder = scenario.mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        try:
            fc.sendto(b"isolated upstream", address)
            assert runtime.recv(4096) == b"isolated upstream"
            command = runtime_encoder.command_long_encode(1, 1, relay_module.NAV_LAND, 0, *([0] * 7))
            runtime.sendto(command.pack(runtime_encoder), address)
            assert fc.recv(4096) == command.pack(runtime_encoder)
            assert relay.land_commands[0]["parameters"] == [0.0] * 7
            assert relay.land_commands[0]["system"] == 245
            ack = fc_encoder.command_ack_encode(relay_module.NAV_LAND, 0)
            fc.sendto(ack.pack(fc_encoder), address)
            assert runtime.recv(4096) == ack.pack(fc_encoder)
            assert len(relay.land_acknowledgements) == 1
            relay.record_observations([Message(kind="COMMAND_ACK", system=250, command=21, result=0)])
            relay.record_observations([heartbeat(system=250, custom_mode=9)])
            assert len(relay.land_acknowledgements) == 1 and relay.heartbeats == []
            relay.record_observations([heartbeat(custom_mode=9)])
            assert relay.heartbeats[-1][1] == 9
        finally:
            relay.close()


def test_invalid_simulator_guard_stops_before_setup_or_runtime_start(monkeypatch):
    calls = []
    runtime = SimpleNamespace(stop=lambda: calls.append("stop"))
    monkeypatch.setattr(scenario, "RuntimeProcess", lambda: runtime)
    monkeypatch.setattr(scenario, "close_resources", lambda _resources, failure: failure)
    monkeypatch.setattr(scenario, "code_identity", lambda: {"code_sha": "test", "dirty_source": True})

    def reject_guard():
        raise scenario.ProbeError("wrong_firmware_revision")

    monkeypatch.setattr(scenario, "verify_isolated_simulator", reject_guard)
    monkeypatch.setattr(scenario, "find_cli", lambda: calls.append("find_cli"))
    result = scenario.run_probe()
    assert result["result"] == "failed" and result["failure"] == "wrong_firmware_revision"
    assert calls == [], "failed exact simulator guard must precede all setup and runtime activity"


def test_external_setup_refuses_any_existing_runtime():
    with pytest.raises(scenario.ProbeError, match="runtime_present_during_external_setup"):
        scenario.prepare_airborne_simulator(SimpleNamespace(), SimpleNamespace(process=object()))


def test_observer_failure_while_cli_waiting_closes_process_without_retry():
    calls = []
    process = SimpleNamespace(
        poll=lambda: None, kill=lambda: calls.append("kill"), communicate=lambda: calls.append("communicate")
    )

    def fail_observation(_timeout):
        raise scenario.ProbeError("wrong_aircraft_identity")

    with pytest.raises(scenario.ProbeError, match="wrong_aircraft_identity"):
        scenario.wait_cli(process, SimpleNamespace(pump=fail_observation), time.monotonic() + 1.0)
    assert calls == ["kill", "communicate"], "close the local CLI, never issue a compensating flight operation"


def test_cli_land_invokes_installed_binary_once_and_pumps_observer(monkeypatch):
    invocations, observations = [], []

    class CLIProcess:
        returncode = 0

        def __init__(self):
            self.pending = True

        def __enter__(self):
            return self

        def __exit__(self, *_arguments):
            return False

        def poll(self):
            if self.pending:
                self.pending = False
                return None
            return 0

        def communicate(self):
            return scenario.SUCCESS_MESSAGE + "\n", ""

    def start(arguments, **options):
        assert options["env"]["NOMAD_CLIENT_ID"] == "nomad-cli"
        assert options["env"]["NOMAD_RUNTIME_IPC_PORT"] == "12345"
        invocations.append(arguments)
        return CLIProcess()

    monkeypatch.setattr(scenario.subprocess, "Popen", start)
    observer = SimpleNamespace(pump=lambda timeout: observations.append(timeout))
    output, elapsed, _completed = scenario.run_cli(Path("nomad"), 12345, "land", observer)
    assert invocations == [["nomad", "land"]], "CLI operation must not replay or use direct qualification tooling"
    assert observations == [0.02, 0.0], "observe continuously while the process waits and drain after its result"
    assert output == scenario.SUCCESS_MESSAGE and 0 <= elapsed < scenario.CLI_BUDGET_SECONDS


def test_cleanup_failure_preserves_original_failure_and_closes_all_resources(monkeypatch):
    calls = []

    class Runtime:
        def stop(self):
            calls.append("stop_runtime")
            raise scenario.ProbeError("runtime_shutdown_failed")

    monkeypatch.setattr(scenario, "RuntimeProcess", Runtime)
    observer = SimpleNamespace(close=lambda: calls.append("close_observer"))
    relay = SimpleNamespace(close=lambda: calls.append("close_relay"))
    failure = scenario.close_resources([Runtime(), observer, relay], "land_engagement_budget_exceeded")
    assert failure == "land_engagement_budget_exceeded"
    assert calls == ["stop_runtime", "close_observer", "close_relay"], "cleanup must not issue another flight command"


def land_ci_script():
    yaml = pytest.importorskip("yaml")
    workflow = yaml.safe_load((ROOT / ".github" / "workflows" / "sitl.yml").read_text(encoding="utf-8"))
    steps = workflow["jobs"]["sitl"]["steps"]
    step = next(
        step
        for step in steps
        if step.get("name") == "Qualify installed CLI Copter LAND on three clean isolated simulators"
    )
    return step


@pytest.mark.skipif(sys.platform == "win32", reason="CI cleanup shell requires native Bash")
def test_failed_land_container_creation_does_not_stop_an_unowned_container():
    bash = shutil.which("bash")
    assert bash is not None
    fake_docker = """
docker() {
  if [[ "$1" == "compose" ]]; then
    return 7
  fi
  printf 'unexpected_cleanup\\n'
  return 0
}
"""
    result = subprocess.run(
        [bash, "-c", fake_docker + land_ci_script()["run"]], capture_output=True, text=True, timeout=5
    )
    assert result.returncode == 7 and "unexpected_cleanup" not in result.stdout
