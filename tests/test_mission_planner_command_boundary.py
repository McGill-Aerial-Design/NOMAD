# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Keep all production Mission Planner vehicle mutations behind runtime IPC."""

from __future__ import annotations

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PRODUCTION_ROOT = ROOT / "mission_planner" / "src"

_STRING_OR_COMMENT = re.compile(
    r'(?P<string>"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\')|(?P<comment>//[^\r\n]*|/\*.*?\*/)',
    re.DOTALL,
)

_FORBIDDEN_WRITES = (
    ("MAVLink command helper", re.compile(r"\bdoCommand(?:Async|Int|Long)?\s*\(", re.IGNORECASE)),
    ("MAVLink packet writer", re.compile(r"\bsendPacket(?:Async)?\s*\(", re.IGNORECASE)),
    (
        "parameter writer",
        re.compile(
            r"\b(?:setParam(?:Async|Value)?|setAndSave|writeParameter|sendParameter)\s*\(",
            re.IGNORECASE,
        ),
    ),
    (
        "mission writer",
        re.compile(
            r"\b(?:setWPs?|setWPCurrent|setMission|uploadMission|writeMission|sendMission)\s*\(",
            re.IGNORECASE,
        ),
    ),
    (
        "flight mode writer",
        re.compile(r"\bsetMode\s*\("),
    ),
    (
        "guided navigation or position-target writer",
        re.compile(
            r"\b(?:setGuidedMode|setGuidedPos(?:ition|AndYaw)?|"
            r"setPositionTarget(?:Local|Global(?:Int)?|LocalNed)?|setFlightMode)\s*\(",
            re.IGNORECASE,
        ),
    ),
    ("MAV_CMD construction", re.compile(r"\bMAV_CMD\s*\.\s*[A-Z0-9_]+\b", re.IGNORECASE)),
    (
        "MAVLink command packet construction",
        re.compile(r"\bmavlink_(?:command_(?:long|int)|set_mode)_t\b", re.IGNORECASE),
    ),
    (
        "MAVLink vehicle-write packet construction",
        re.compile(
            r"\bmavlink_(?:set_position_target_[a-z0-9_]+|param_set|mission_(?:item(?:_int)?|count|clear_all|set_current|write_partial_list))_t\b",
            re.IGNORECASE,
        ),
    ),
    ("MAVLink passthrough API", re.compile(r"\bMavlinkPassthrough\b", re.IGNORECASE)),
    (
        "MAVLink command or message writer",
        re.compile(
            r"\b(?:sendMavlink(?:Packet|Message|Command)?|sendCommand(?:Long|Int)?)\s*\(",
            re.IGNORECASE,
        ),
    ),
)


def _remove_comments(source: str) -> str:
    """Ignore comments while retaining string contents and executable code."""
    return _STRING_OR_COMMENT.sub(
        lambda match: match.group("string") or " " * len(match.group(0)),
        source,
    )


def _find_direct_writes(source: str) -> list[str]:
    code = _remove_comments(source)
    return [name for name, pattern in _FORBIDDEN_WRITES if pattern.search(code)]


def _production_sources() -> list[Path]:
    return [path for path in sorted(PRODUCTION_ROOT.rglob("*.cs")) if not {"bin", "obj"}.intersection(path.parts)]


def test_all_production_mission_planner_sources_use_runtime_for_vehicle_writes() -> None:
    findings = []
    for source in _production_sources():
        for violation in _find_direct_writes(source.read_text(encoding="utf-8")):
            findings.append(f"{source.relative_to(ROOT).as_posix()}: {violation}")

    assert not findings, "Direct vehicle writes in Mission Planner production code:\n" + "\n".join(findings)


@pytest.mark.parametrize(
    ("source", "expected"),
    [
        ("MainV2.comPort.doCommandAsync(command);", "MAVLink command helper"),
        ("MainV2.comPort.MAV.sendPacket(packet);", "MAVLink packet writer"),
        ("MainV2.comPort.MAV.setMode(mode);", "flight mode writer"),
        ("MainV2.comPort.setParam(name, value);", "parameter writer"),
        ("MainV2.comPort.setWPs(waypoints);", "mission writer"),
        ("MainV2.comPort.setPositionTarget(target);", "guided navigation or position-target writer"),
        ("MainV2.comPort.SetGuidedMode();", "guided navigation or position-target writer"),
        (
            "MainV2.comPort.SetPositionTargetGlobalInt(target);",
            "guided navigation or position-target writer",
        ),
        ("var command = MAVLink.MAV_CMD.DO_SET_MODE;", "MAV_CMD construction"),
        ("var command = new MAVLink.mavlink_command_long_t();", "MAVLink command packet construction"),
        ("MavlinkPassthrough.SendMessage(message);", "MAVLink passthrough API"),
        ("new MAVLink.mavlink_mission_item_int_t();", "MAVLink vehicle-write packet construction"),
        ("new MAVLink.mavlink_set_mode_t();", "MAVLink command packet construction"),
    ],
)
def test_guard_rejects_direct_vehicle_write_patterns(source: str, expected: str) -> None:
    assert expected in _find_direct_writes(source)


def test_read_only_telemetry_and_runtime_requests_remain_allowed() -> None:
    source = """
    var heartbeat = MainV2.comPort.MAV.cs.heartbeat;
    var capacity = MainV2.comPort.MAV.param[\"BATT_CAPACITY\"];
    client.RequestGimbalTarget(pitch, roll, yaw);
    """

    assert _find_direct_writes(source) == []


def test_comment_examples_do_not_trigger_the_guard() -> None:
    source = "// Never call MainV2.comPort.doCommandAsync(command);\nvar status = client.GetStatus();"

    assert _find_direct_writes(source) == []
