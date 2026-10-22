# SPDX-License-Identifier: Apache-2.0
import ast
from pathlib import Path


def test_action_categories_do_not_echo_native_error_text():
    source = Path(__file__).resolve().parents[1] / "mission_planner/tests/native_sitl/native_sitl.py"
    function = next(
        node
        for node in ast.parse(source.read_text(encoding="utf-8")).body
        if isinstance(node, ast.FunctionDef) and node.name == "action_category"
    )
    namespace = {}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), namespace)
    category = namespace["action_category"]
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
