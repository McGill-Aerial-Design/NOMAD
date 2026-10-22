# SPDX-License-Identifier: Apache-2.0
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest


def load_function(name, namespace):
    source = Path(__file__).resolve().parents[1] / "mission_planner/tests/native_sitl/native_sitl.py"
    function = next(
        node
        for node in ast.parse(source.read_text(encoding="utf-8")).body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), namespace)
    return namespace[name]


def test_action_categories_do_not_echo_native_error_text():
    category = load_function("action_category", {})
    assert category("Safe command ACK accepted. Physical state is unverified.") == "safe_accepted"
    assert (
        category("Mission Planner did not confirm Safe acceptance; physical state is unverified.")
        == "safe_not_confirmed"
    )
    assert (
        category(
            "Safe outcome is uncertain; inspect aircraft before retrying. "
            "Mission Planner is busy with a native request."
        )
        == "safe_host_busy"
    )
    assert (
        category("Safe outcome is uncertain; inspect aircraft before retrying. diagnostic-redaction-sentinel")
        == "safe_uncertain_other"
    )
    assert category("diagnostic-redaction-sentinel") == "other_status"


def idle_fixture(states, callback_error=""):
    clock = [0]
    samples = iter(states)
    current = [states[0]]

    def sample():
        current[0] = next(samples, current[0])
        return current[0]

    def sleep(milliseconds):
        clock[0] += milliseconds

    namespace = {
        "now": lambda: clock[0] / 1000.0,
        "Thread": SimpleNamespace(Sleep=sleep),
        "action_state": sample,
        "callback_error": callback_error,
    }
    wait = load_function("wait_for_native_idle", namespace)
    return wait, namespace, clock


def test_native_idle_waits_for_both_owners_before_returning_snapshot():
    idle = {"generation": 6, "target_valid": True, "target_current": True, "give_comport": False, "payload_busy": False}
    wait, _, clock = idle_fixture([dict(idle, give_comport=True), dict(idle, payload_busy=True), idle])
    assert wait() is idle
    assert clock[0] == 40


def test_native_idle_ready_does_not_add_an_artificial_delay():
    idle = {"generation": 6, "target_valid": True, "target_current": True, "give_comport": False, "payload_busy": False}
    wait, _, clock = idle_fixture([idle])
    assert wait() is idle
    assert clock[0] == 0


@pytest.mark.parametrize("owner", ["give_comport", "payload_busy"])
def test_native_idle_deadline_prevents_payload_submission(owner):
    busy = {"generation": 6, "target_valid": True, "target_current": True, "give_comport": False, "payload_busy": False}
    busy[owner] = True
    _, namespace, clock = idle_fixture([busy])
    namespace["owner"] = SimpleNamespace(GetType=lambda: pytest.fail("payload submitted before native idle"))
    action = load_function("action", namespace)
    with pytest.raises(Exception, match="^native_idle_deadline$"):
        action("motion", "safe")
    assert clock[0] == 5000


@pytest.mark.parametrize("fault", [{"target_valid": False}, {"target_current": False}, {"generation": 7}])
def test_native_idle_rejects_context_fault_while_waiting(fault):
    busy = {"generation": 6, "target_valid": True, "target_current": True, "give_comport": True, "payload_busy": False}
    wait, _, clock = idle_fixture([busy, dict(busy, give_comport=False, **fault)])
    with pytest.raises(Exception, match="^native_idle_context_changed$"):
        wait()
    assert clock[0] == 20


def test_native_idle_rejects_packet_decode_fault_before_ready():
    idle = {"generation": 6, "target_valid": True, "target_current": True, "give_comport": False, "payload_busy": False}
    wait, _, clock = idle_fixture([idle], callback_error="decode-error-sentinel")
    with pytest.raises(Exception, match="^packet_decode_failed$"):
        wait()
    assert clock[0] == 0
