# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Unit tests for the C++ SITL safety harness helpers."""

from __future__ import annotations

import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest
from pymavlink import mavutil

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts" / "dev"
sys.path.insert(0, str(SCRIPTS))

import core_sitl_command_flow as command_flow  # noqa: E402
import core_sitl_containment as containment  # noqa: E402
import core_sitl_gcs_heartbeat as gcs_heartbeat  # noqa: E402
import core_sitl_zero_delivery as zero_delivery  # noqa: E402


def test_run_cli_rejection_requires_nonzero_result_and_expected_message(monkeypatch) -> None:
    completed = subprocess.CompletedProcess([], 1, "", "error: outside the geofence")
    monkeypatch.setattr(command_flow.subprocess, "run", lambda *args, **kwargs: completed)

    output = command_flow.run_cli_rejection(Path("nomad"), "14570", "outside the geofence", "goto", "1", "2", "3")

    assert "outside the geofence" in output


def test_run_cli_rejection_rejects_a_successful_command(monkeypatch) -> None:
    completed = subprocess.CompletedProcess([], 0, "goto verified", "")
    monkeypatch.setattr(command_flow.subprocess, "run", lambda *args, **kwargs: completed)

    with pytest.raises(command_flow.ScenarioError, match="expected command to fail"):
        command_flow.run_cli_rejection(Path("nomad"), "14570", "outside the geofence", "goto", "1", "2", "3")


@pytest.mark.parametrize("port, expected", [("1", (1, 2, 3)), ("65533", (65533, 65534, 65535))])
def test_zero_delivery_relay_ports_allow_only_safe_boundaries(port: str, expected: tuple[int, int, int]) -> None:
    assert zero_delivery.get_relay_ports(port) == expected


@pytest.mark.parametrize("port", ["0", "65534", "65535", "not-a-port"])
def test_zero_delivery_relay_ports_fail_closed(port: str) -> None:
    with pytest.raises(ValueError):
        zero_delivery.get_relay_ports(port)


def make_velocity_packet(vx: float, vy: float, vz: float, yaw_rate: float) -> bytes:
    encoder = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    message = encoder.set_position_target_local_ned_encode(
        0,
        1,
        1,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0,
        0.0,
        0.0,
        0.0,
        vx,
        vy,
        vz,
        0.0,
        0.0,
        0.0,
        0.0,
        yaw_rate,
    )
    return message.pack(encoder)


def test_zero_delivery_observer_decodes_nonzero_and_zero_datagrams() -> None:
    observer = zero_delivery.SetpointObserver(0)
    try:
        observer._record_datagram(make_velocity_packet(0.3, 0.0, 0.0, 0.0))
        observer._record_datagram(make_velocity_packet(0.0, 0.0, 0.0, 0.0))
        points = observer.setpoints_in(1.0)
    finally:
        observer.close()

    assert len(points) == 2
    assert zero_delivery.is_nonzero(points[0])
    assert zero_delivery.is_zero(points[1])


def make_gcs_heartbeat_frame() -> bytes:
    frame = bytearray(21)
    frame[0] = 0xFD
    frame[5] = gcs_heartbeat.GCS_SYSTEM_ID
    frame[6] = 190
    frame[7] = 0
    frame[14] = 6
    frame[15] = 8
    return bytes(frame)


def test_gcs_heartbeat_cadence_uses_intervals_not_short_command_duration() -> None:
    frame = make_gcs_heartbeat_frame()
    announcements = [(10.0, frame), (11.0, frame), (12.0, frame), (13.0, frame)]

    gcs_heartbeat.verify_announcements(announcements, require_cadence=True)

    assert gcs_heartbeat.measured_rate(announcements) == pytest.approx(1.0)


def test_gcs_heartbeat_cadence_rejects_fast_or_single_samples() -> None:
    frame = make_gcs_heartbeat_frame()
    with pytest.raises(gcs_heartbeat.ScenarioError, match="fewer than 4"):
        gcs_heartbeat.verify_announcements([(10.0, frame), (11.0, frame), (12.0, frame)], require_cadence=True)
    with pytest.raises(gcs_heartbeat.ScenarioError, match="faster than the 1 Hz tolerance"):
        gcs_heartbeat.verify_announcements(
            [(10.0, frame), (10.5, frame), (11.5, frame), (12.5, frame)], require_cadence=True
        )
    with pytest.raises(gcs_heartbeat.ScenarioError, match="slower than the 1 Hz tolerance"):
        gcs_heartbeat.verify_announcements(
            [(10.0, frame), (11.0, frame), (12.0, frame), (13.4, frame)], require_cadence=True
        )


def test_gcs_heartbeat_cadence_accepts_tolerance_boundaries() -> None:
    frame = make_gcs_heartbeat_frame()
    announcements = [(10.0, frame), (10.9, frame), (12.2, frame), (13.5, frame)]

    gcs_heartbeat.verify_announcements(announcements, require_cadence=True)


def test_gcs_heartbeat_cadence_rejects_non_finite_intervals() -> None:
    frame = make_gcs_heartbeat_frame()

    with pytest.raises(gcs_heartbeat.ScenarioError, match="non-finite"):
        gcs_heartbeat.verify_announcements(
            [(10.0, frame), (11.0, frame), (float("nan"), frame), (13.0, frame)], require_cadence=True
        )


def test_gcs_heartbeat_relay_learns_mavsdk_udpout_source() -> None:
    relay_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    relay_socket.bind(("127.0.0.1", 0))
    relay_port = relay_socket.getsockname()[1]
    relay_socket.close()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.bind(("127.0.0.1", 0))
    sender.settimeout(1.0)
    upstream = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    relay = gcs_heartbeat.GatedRelay(relay_port, gate_opens=True)
    try:
        sender.sendto(make_gcs_heartbeat_frame(), ("127.0.0.1", relay_port))
        deadline = time.monotonic() + 1.0
        while not relay.announcements and time.monotonic() < deadline:
            time.sleep(0.01)
        assert relay.announcements

        payload = b"vehicle telemetry"
        upstream.sendto(payload, ("127.0.0.1", relay_port))
        received, _ = sender.recvfrom(1024)
        assert received == payload
    finally:
        relay.close()
        upstream.close()
        sender.close()


def test_containment_converts_latitude_and_longitude_to_local_metres() -> None:
    north, east = containment.get_local_displacement((45.0, -73.0), (45.0001, -72.9999))

    assert north == pytest.approx(11.054)
    assert east == pytest.approx(7.871, rel=0.01)


def test_containment_fence_corners_have_equal_local_extent() -> None:
    home = (45.0, -73.0)
    corners = [
        tuple(float(part) for part in vertex.split(",")) for vertex in containment.fence_env_around(home).split(";")
    ]

    displacements = [containment.get_local_displacement(home, corner) for corner in corners]

    assert len(displacements) == 4
    for north, east in displacements:
        assert abs(north) == pytest.approx(containment.FENCE_HALF_EXTENT_M, rel=0.001)
        assert abs(east) == pytest.approx(containment.FENCE_HALF_EXTENT_M, rel=0.001)


def test_containment_cleanup_attempts_every_safe_action_after_failures(monkeypatch) -> None:
    attempted: list[str] = []

    def fail_command(binary, port, action, **kwargs):
        attempted.append(action)
        raise containment.ScenarioError(f"{action} failed")

    def fail_status(binary, port, expected, timeout):
        attempted.append("status")
        raise containment.ScenarioError("status failed")

    monkeypatch.setattr(containment, "run_cli", fail_command)
    monkeypatch.setattr(containment, "wait_for_status", fail_status)

    errors = containment.cleanup_containment(Path("nomad"), "14570")

    assert attempted == ["rtl", "land", "status", "disarm"]
    assert len(errors) == 4
