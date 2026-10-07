# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Read-only telemetry and stale-source integration tests for ``nomad_ros``."""

from __future__ import annotations

import time

import pytest


def _require_connection(session) -> None:
    if not session["state"].node_connected:
        pytest.fail("node never connected to the MAVLink telemetry responder")


def _wait_for_count(values: list, minimum: int, timeout: float = 10.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and len(values) < minimum:
        time.sleep(0.1)
    assert len(values) >= minimum, f"expected at least {minimum} telemetry messages, got {len(values)}"


def test_node_publishes_validated_fix_and_battery(ros_session) -> None:
    """A sustained healthy peer produces source-stamped GPS and battery data."""
    _require_connection(ros_session)
    state = ros_session["state"]
    _wait_for_count(state.fix_messages, 2)
    _wait_for_count(state.battery_messages, 2)

    fix = state.fix_messages[-1]
    assert fix.status.status == 0, "expected STATUS_FIX from a valid 3D GPS sample"
    assert fix.latitude == pytest.approx(42.3898)
    assert fix.longitude == pytest.approx(-71.1476)
    assert fix.altitude == pytest.approx(14.1)
    assert fix.header.stamp.sec > 0, "fix must carry its source sample time"

    battery = state.battery_messages[-1]
    assert battery.present
    assert battery.voltage == pytest.approx(12.6)
    assert battery.percentage == pytest.approx(0.5)
    assert battery.header.stamp.sec > 0, "battery must carry its source sample time"


@pytest.mark.parametrize("source_field", ["send_position", "send_gps"])
def test_stale_position_or_gps_is_not_republished_as_a_healthy_fix(ros_session, source_field: str) -> None:
    """Fresh heartbeats cannot make an old position or GPS sample appear current."""
    _require_connection(ros_session)
    state = ros_session["state"]
    _wait_for_count(state.fix_messages, 2)
    setattr(state, source_field, False)
    try:
        time.sleep(1.75)
        before = len(state.fix_messages)
        battery_before = len(state.battery_messages)
        time.sleep(0.4)
        assert state.node_connected, "fresh heartbeat should keep link connectivity true"
        assert len(state.fix_messages) == before, (
            f"stale GPS was republished: {len(state.fix_messages) - before} extra fixes"
        )
        assert len(state.battery_messages) > battery_before, "independent fresh battery updates should continue"
    finally:
        setattr(state, source_field, True)


def test_stale_battery_is_not_republished_as_current(ros_session) -> None:
    """Fresh navigation data does not make an old battery sample current."""
    _require_connection(ros_session)
    state = ros_session["state"]
    _wait_for_count(state.battery_messages, 2)
    state.send_battery = False
    try:
        time.sleep(1.75)
        before = len(state.battery_messages)
        fix_before = len(state.fix_messages)
        time.sleep(0.4)
        assert len(state.battery_messages) == before, (
            f"stale battery was republished: {len(state.battery_messages) - before} extra messages"
        )
        assert len(state.fix_messages) > fix_before, "fresh navigation observations should continue"
    finally:
        state.send_battery = True


@pytest.mark.parametrize("timestamp_field", ["repeat_position_timestamp", "repeat_gps_timestamp"])
def test_repeated_navigation_source_timestamp_is_not_refreshed(ros_session, timestamp_field: str) -> None:
    """New packet sequence numbers do not make an old navigation sample fresh."""
    _require_connection(ros_session)
    state = ros_session["state"]
    _wait_for_count(state.fix_messages, 2)
    setattr(state, timestamp_field, True)
    try:
        time.sleep(1.75)
        before = len(state.fix_messages)
        time.sleep(0.4)
        assert len(state.fix_messages) == before, f"repeated {timestamp_field} was republished as a new fix"
    finally:
        setattr(state, timestamp_field, False)


def test_invalid_coordinates_and_fix_quality_are_not_published(ros_session) -> None:
    """Fresh packets with invalid coordinates or a non-3D fix stay unavailable."""
    _require_connection(ros_session)
    state = ros_session["state"]
    _wait_for_count(state.fix_messages, 2)
    state.invalid_position = True
    try:
        time.sleep(0.5)
        before = len(state.fix_messages)
        state.gps_fix_type = 2
        time.sleep(0.4)
        assert len(state.fix_messages) == before, "invalid coordinates or 2D fix were published as healthy"
    finally:
        state.invalid_position = False
        state.gps_fix_type = 3


@pytest.mark.parametrize(
    ("field", "invalid_value"),
    [("battery_voltage_mv", 65535), ("battery_remaining_percent", -1)],
)
def test_invalid_battery_fields_are_not_published(ros_session, field: str, invalid_value: int) -> None:
    """MAVLink's unknown battery sentinels do not become healthy sensor data."""
    _require_connection(ros_session)
    state = ros_session["state"]
    _wait_for_count(state.battery_messages, 2)
    setattr(state, field, invalid_value)
    try:
        time.sleep(0.4)
        before = len(state.battery_messages)
        time.sleep(0.3)
        assert len(state.battery_messages) == before, f"invalid {field} was published"
    finally:
        setattr(state, field, 12600 if field == "battery_voltage_mv" else 50)
