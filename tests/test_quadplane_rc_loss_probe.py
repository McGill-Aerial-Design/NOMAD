# SPDX-License-Identifier: Apache-2.0
"""Falsify the disarmed QuadPlane RC fault-injection probe without SITL."""

from __future__ import annotations

import sys
import time
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts" / "dev"
sys.path.insert(0, str(SCRIPTS))

import core_sitl_quadplane_rc_loss_probe as probe  # noqa: E402


class FakeMessage:
    def __init__(self, kind: str, **fields) -> None:
        self.kind = kind
        self.__dict__.update(fields)

    def get_type(self) -> str:
        return self.kind

    def get_srcSystem(self) -> int:
        return 1

    def get_srcComponent(self) -> int:
        return 1


class FakeConnection:
    def __init__(self, messages: list[FakeMessage] | None = None) -> None:
        self.messages = list(messages or [])
        self.commands: list[tuple] = []
        self.read_values: dict[str, float] = {}
        self.mav = FakeMav(self)

    def recv_match(self, blocking: bool, timeout: float | None = None):
        del blocking, timeout
        if self.messages:
            return self.messages.pop(0)
        time.sleep(0.001)
        return None

    def close(self) -> None:
        pass


class FakeMav:
    def __init__(self, connection: FakeConnection) -> None:
        self.connection = connection

    def param_request_read_send(self, _system: int, _component: int, name: bytes, _index: int) -> None:
        self.connection.commands.append(("read", name.decode("ascii")))
        value = self.connection.read_values.get(name.decode("ascii"), 1.0)
        self.connection.messages.append(parameter_message(name.decode("ascii"), value))

    def param_set_send(self, _system: int, _component: int, name: bytes, value: float, _kind: int) -> None:
        self.connection.commands.append(("set", name.decode("ascii"), value))
        self.connection.messages.append(parameter_message(name.decode("ascii"), value))


def parameter_message(name: str, value: float) -> FakeMessage:
    return FakeMessage("PARAM_VALUE", param_id=name.encode("ascii"), param_value=value)


def make_container() -> dict:
    return {
        "Name": f"/{probe.CONTAINER}",
        "Config": {
            "Image": probe.IMAGE,
            "Labels": {
                "com.docker.compose.project": probe.COMPOSE_PROJECT,
                "com.docker.compose.service": "quadplane_sitl",
            },
            "Env": [
                f"SITL_UDP_OUTPUT_ADDRESS=udp:host.docker.internal:{probe.CLIENT_PORT}",
                f"SITL_UDP_OBSERVER_ADDRESS=udp:host.docker.internal:{probe.OBSERVER_PORT}",
                "SITL_PROFILE_DEFAULTS=/nomad/quadplane-tilttri.parm",
                "SPEEDUP=1",
            ],
            "Entrypoint": ["/bin/sh", "/nomad/sitl-quadplane-entrypoint.sh"],
        },
        "State": {"Running": True},
        "Mounts": [{"Destination": destination, "RW": False} for destination in probe.REQUIRED_MOUNTS],
    }


def make_heartbeat(armed: bool = False) -> FakeMessage:
    mode = probe.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
    return FakeMessage(
        "HEARTBEAT",
        type=probe.mavutil.mavlink.MAV_TYPE_FIXED_WING,
        autopilot=probe.mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode=mode,
    )


def make_sys_status(healthy: bool) -> FakeMessage:
    health = probe.RC_RECEIVER if healthy else 0
    return FakeMessage(
        "SYS_STATUS",
        onboard_control_sensors_present=probe.RC_RECEIVER,
        onboard_control_sensors_health=health,
    )


def make_observer() -> probe.ReceiverObserver:
    return probe.ReceiverObserver(FakeConnection())


def test_container_guard_accepts_only_the_dedicated_pinned_topology() -> None:
    probe.validate_container(make_container())


@pytest.mark.parametrize(
    "mutate",
    [
        lambda details: details["Config"]["Labels"].update({"com.docker.compose.project": "nomad-quadplane"}),
        lambda details: details["Config"].update({"Image": "unrelated:latest"}),
        lambda details: details["Config"]["Env"].__setitem__(
            0, "SITL_UDP_OUTPUT_ADDRESS=udp:host.docker.internal:14580"
        ),
        lambda details: details["Mounts"][0].update({"RW": True}),
        lambda details: details.update({"Name": "/nomad-quadplane-quadplane_sitl-1"}),
    ],
)
def test_container_guard_rejects_reused_or_mutable_topology(mutate) -> None:
    details = make_container()
    mutate(details)

    with pytest.raises(probe.ProbeError):
        probe.validate_container(details)


def test_receiver_health_requires_present_flag() -> None:
    message = make_sys_status(True)
    message.onboard_control_sensors_present = 0

    with pytest.raises(probe.ProbeError, match="not observable"):
        probe.get_receiver_health(message)


def test_observer_rejects_armed_heartbeat_and_emits_no_commands() -> None:
    observer = make_observer()
    with pytest.raises(probe.ProbeError, match="remain disarmed"):
        observer.record(make_heartbeat(armed=True), time.monotonic())

    observer.record(make_heartbeat(), time.monotonic())
    observer.record(make_sys_status(True), time.monotonic())
    assert observer.connection.commands == []


def test_observer_run_ignores_unrelated_system_and_component() -> None:
    class OtherMessage(FakeMessage):
        def __init__(self, kind: str, source_system: int, source_component: int, **fields) -> None:
            super().__init__(kind, **fields)
            self.source_system = source_system
            self.source_component = source_component

        def get_srcSystem(self) -> int:
            return self.source_system

        def get_srcComponent(self) -> int:
            return self.source_component

    wrong_system_fields = dict(make_heartbeat().__dict__)
    wrong_system_fields.pop("kind")
    wrong_component_fields = dict(make_sys_status(False).__dict__)
    wrong_component_fields.pop("kind")
    connection = FakeConnection(
        [
            OtherMessage("HEARTBEAT", 2, 1, **wrong_system_fields),
            OtherMessage("SYS_STATUS", 1, 2, **wrong_component_fields),
            make_heartbeat(),
            make_sys_status(True),
        ]
    )
    observer = probe.ReceiverObserver(connection)
    observer.start()
    deadline = time.monotonic() + 1.0
    while not observer.heartbeat_at and time.monotonic() < deadline:
        time.sleep(0.001)
    observer.stop_requested.set()
    observer.thread.join(timeout=1)

    assert observer.heartbeat_at > 0
    assert len(observer.samples) == 1
    assert observer.samples[0][1] is True
    assert connection.commands == []
    connection.close()


def test_fault_wait_ignores_pre_boundary_unhealthy_samples() -> None:
    observer = make_observer()
    boundary = time.monotonic() - 0.2
    observer.heartbeat_at = boundary + 0.05
    observer.samples.append((boundary - 0.01, False))

    with pytest.raises(probe.ObservationTimeout):
        observer.wait_for_health(False, boundary, timeout=0.01)


def test_fault_detector_rejects_three_fresh_healthy_samples() -> None:
    observer = make_observer()
    now = time.monotonic()
    boundary = now - 0.5
    observer.heartbeat_at = now - 0.05
    observer.samples.extend([(now - 0.3, True), (now - 0.2, True), (now - 0.1, True)])

    with pytest.raises(probe.ObservationTimeout):
        observer.wait_for_health(False, boundary, timeout=0.01)


def test_fault_detector_rejects_three_stale_unhealthy_samples() -> None:
    observer = make_observer()
    boundary = time.monotonic() - 0.2
    observer.heartbeat_at = time.monotonic()
    observer.samples.extend([(boundary - 0.3, False), (boundary - 0.2, False), (boundary - 0.1, False)])

    with pytest.raises(probe.ObservationTimeout):
        observer.wait_for_health(False, boundary, timeout=0.01)


def test_healthy_negative_control_requires_fresh_healthy_samples() -> None:
    observer = make_observer()
    boundary = time.monotonic() - 0.2
    observer.heartbeat_at = boundary + 0.05
    observer.samples.extend((boundary - 0.1, True) for _ in range(3))

    with pytest.raises(probe.ObservationTimeout):
        observer.wait_for_health(True, boundary, timeout=0.01, fail_on_change=True)


def test_healthy_negative_control_fails_on_any_receiver_loss() -> None:
    observer = make_observer()
    boundary = time.monotonic() - 0.2
    observer.heartbeat_at = time.monotonic()
    observer.samples.append((time.monotonic(), False))

    with pytest.raises(probe.ProbeError, match="no-injection control"):
        observer.wait_for_health(True, boundary, timeout=0.01, fail_on_change=True)


def test_fault_observation_requires_three_fresh_post_boundary_samples() -> None:
    observer = make_observer()
    now = time.monotonic()
    boundary = now - 0.5
    observer.heartbeat_at = now - 0.05
    observer.samples.extend([(boundary - 0.1, True), (now - 0.3, False), (now - 0.2, False), (now - 0.1, False)])

    samples = observer.wait_for_health(False, boundary, timeout=0.01)

    assert len(samples) == 3
    assert all(stamp > boundary and value is False for stamp, value in samples)


def test_parameter_read_discards_a_queued_stale_value_before_request() -> None:
    connection = FakeConnection([parameter_message("SIM_RC_FAIL", 0.0)])
    parameters = probe.SimulatorParameters(connection)

    assert parameters.read("SIM_RC_FAIL") == 1.0


def test_parameter_and_receiver_timeouts_name_distinct_failed_conditions(monkeypatch) -> None:
    parameter_clock = iter([0.0, 6.0])
    monkeypatch.setattr(probe.time, "monotonic", lambda: next(parameter_clock))
    parameters = probe.SimulatorParameters(FakeConnection())

    with pytest.raises(probe.ObservationTimeout) as parameter_error:
        parameters.wait_for_parameter("FS_LONG_TIMEOUT", boundary=0.0)

    assert parameter_error.value.category == "fresh_parameter_readback_missing"

    receiver_clock = iter([10.0, 11.0])
    monkeypatch.setattr(probe.time, "monotonic", lambda: next(receiver_clock))
    observer = make_observer()

    with pytest.raises(probe.ObservationTimeout) as receiver_error:
        observer.wait_for_health(False, boundary=9.0, timeout=0.1, timeout_category="simulated_rc_loss_not_observed")

    assert receiver_error.value.category == "simulated_rc_loss_not_observed"
    monkeypatch.setattr(
        probe,
        "get_code_identity",
        lambda: {"code_sha": "test-sha", "dirty_source": True},
    )

    parameter_failure = probe.failure_evidence(parameter_error.value, [])
    receiver_failure = probe.failure_evidence(receiver_error.value, [])

    assert parameter_failure["failure"] == "fresh_parameter_readback_missing"
    assert receiver_failure["failure"] == "simulated_rc_loss_not_observed"
    assert parameter_failure["failure"] != receiver_failure["failure"]


def test_rc_parameter_set_is_limited_to_no_pulses_and_read_back_twice() -> None:
    connection = FakeConnection()
    parameters = probe.SimulatorParameters(connection)
    parameters.set_rc_failure(1)

    assert connection.commands == [
        ("set", "SIM_RC_FAIL", 1),
        ("read", "SIM_RC_FAIL"),
    ]
    with pytest.raises(probe.ProbeError, match="only simulator no-pulses"):
        parameters.set_rc_failure(2)


def test_rc_parameter_restore_rejects_mismatched_live_readback() -> None:
    connection = FakeConnection()
    connection.read_values["SIM_RC_FAIL"] = 1.0
    parameters = probe.SimulatorParameters(connection)

    with pytest.raises(probe.ProbeError, match="did not match the request"):
        parameters.set_rc_failure(0)


def test_restore_attempts_parameter_and_observer_and_keeps_both_failures() -> None:
    parameter_error = probe.ProbeError("reset failed")
    recovery_error = probe.ProbeError("receiver did not recover")

    class Parameters:
        def set_rc_failure(self, value: int) -> None:
            assert value == 0
            raise parameter_error

    class Observer:
        def wait_for_health(self, healthy: bool, boundary: float, **_options):
            assert healthy is True
            assert boundary > 0
            raise recovery_error

    _samples, errors = probe.restore_rc_input(Parameters(), Observer(), time.monotonic())
    combined = probe.ProbeCleanupError(probe.ProbeError("fault observation failed"), errors)

    assert errors == [parameter_error, recovery_error]
    assert combined.primary_error.args == ("fault observation failed",)
    assert combined.cleanup_errors == [parameter_error, recovery_error]


def test_probe_preserves_fault_failure_when_restore_and_recovery_fail() -> None:
    primary_error = probe.ProbeError("parameter injection failed")
    restore_error = probe.ProbeError("parameter reset failed")
    recovery_error = probe.ProbeError("receiver recovery failed")

    class Parameters:
        def read(self, name: str) -> float:
            return 2.0 if name == "Q_ENABLE" else 0.0

        def set_rc_failure(self, value: int) -> None:
            raise primary_error if value == 1 else restore_error

    class Observer:
        started_at = time.monotonic()

        def __init__(self) -> None:
            self.calls = 0

        def wait_for_health(self, healthy: bool, boundary: float, **_options):
            self.calls += 1
            if self.calls == 3:
                raise recovery_error
            return [(boundary + 0.1, healthy)] * 3

    with pytest.raises(probe.ProbeCleanupError) as raised:
        probe.run_receiver_probe(Parameters(), Observer())

    assert raised.value.primary_error is primary_error
    assert raised.value.cleanup_errors == [restore_error, recovery_error]


def test_failure_output_preserves_categories_without_raw_error_text() -> None:
    primary = probe.ProbeError("secret transcript or parameter output")
    shutdown = probe.ProbeError("socket close details")

    evidence = probe.failure_evidence(primary, [shutdown])

    assert evidence["failure"] == "probe_guard_or_observation_failure"
    assert evidence["shutdown_failures"] == ["probe_guard_or_observation_failure"]
    assert "secret" not in repr(evidence)
    assert "socket" not in repr(evidence)


def test_wrong_firmware_stops_before_control_socket_or_parameter_write(monkeypatch, capsys) -> None:
    import json

    guard_calls: list[list[str]] = []

    def run_guard_command(arguments: list[str]) -> str:
        guard_calls.append(arguments)
        if arguments[1] == "inspect":
            return json.dumps([make_container()])
        if "rev-parse" in arguments:
            return "wrong-firmware-revision"
        raise AssertionError("wrong revision must stop all later guard and parameter work")

    control_sockets: list[str] = []
    monkeypatch.setattr(probe, "run_guard_command", run_guard_command)
    monkeypatch.setattr(
        probe.mavutil,
        "mavlink_connection",
        lambda endpoint, **_kwargs: control_sockets.append(endpoint),
    )
    monkeypatch.setattr(
        probe,
        "get_code_identity",
        lambda: {"code_sha": "test-source-sha", "dirty_source": True},
    )

    assert probe.main() == 1

    assert control_sockets == []
    assert len(guard_calls) == 2
    assert "wrong_firmware_pin" in capsys.readouterr().err


def test_probe_scope_does_not_claim_a_total_c2_cut() -> None:
    evidence = probe.format_probe_evidence(
        1.0,
        {"Q_ENABLE": 2.0},
        [(1.1, True)] * 3,
        [(2.1, True)] * 3,
        3.0,
        [(3.1, False)] * 3,
        4.0,
        [(4.1, True)] * 3,
    )

    assert "not_total_c2_cut" in evidence["scope"]
    assert evidence["parameter_readbacks"] == {"Q_ENABLE": 2.0}
    assert evidence["unhealthy_fault_samples"] == [{"t_s": 2.1, "receiver_healthy": False}] * 3
