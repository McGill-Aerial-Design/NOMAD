# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Link-level parity cases for the MAVSDK transport.

These are the legacy UDP transport's link proofs moved onto the MAVSDK
transport: the pre-latch GCS-heartbeat announcement that opens a heartbeat-gated
relay, MAVProxy-style coalesced datagrams, a heartbeat that dies while the
delivery path stays open, the body-frame velocity setpoint with its
zero-on-disconnect, the core-level zero-delivery scenarios of SR-LNK-03, and
the core fence upload and verification of SR-FEN-01.

Every case judges the peer's decoding of what arrived rather than the transport's
own report. `mavsdk_connection_fixture.py` is the entry point that runs these
with the command-parity cases; the pytest driver in
`tests/test_mavsdk_connection.py` runs them one by one.
"""

from __future__ import annotations

import math
import subprocess
from pathlib import Path

from mavsdk_fixture_harness import (
    describe,
    find_free_udp_port,
    require,
    run_cli_case,
    run_fence_probe,
    run_probe,
    run_staleness_probe,
    run_velocity_probe,
    run_zero_delivery_probe,
    with_peer,
)
from mavsdk_peer import (
    ACCEPTED,
    BODY_NED_FRAME,
    COMMAND_ARM_DISARM,
    GCS_AUTOPILOT_TYPE,
    GCS_COMPONENT_ID,
    GCS_VEHICLE_TYPE,
    ReceivedMessage,
    SetpointRecord,
    VehiclePeer,
)

# NOMAD's velocity command, pinned independently of the transport: ignore
# position, acceleration and absolute yaw; send velocity and yaw rate.
# BODY_NED uses body FRD velocity; the force bit is clear because acceleration is ignored.
VELOCITY_TYPE_MASK = 0x05C7

# Must match kStreamedVelocity in tests/mavsdk_zero_delivery_test.cpp. The core
# converts vx/vy/vz/yaw_rate to the body-frame NED convention, so a setpoint
# with only vx set is the one both sides can name without sign arithmetic.
STREAMED_SETPOINT = (0.5, 0.0, 0.0, 0.0)

# Must match the boundary in run_fence_probe (tests/mavsdk_connection_test.cpp).
# The peer decodes lat/lon back from the int32 wire form, so the tolerance
# covers the 1e-7 degree encoding.
FENCE_BOUNDARY = ((45.5, -73.6), (45.51, -73.6), (45.505, -73.59))
FENCE_COORDINATE_TOLERANCE = 2e-5

# The watchdog reason each zero-delivery scenario must report.
EXPECTED_REASONS = {
    "command-timeout": "reason=command_timeout",
    "caller-stop": "reason=none",
    "vehicle-destructor": "reason=none",
    "link-loss": "reason=heartbeat_stale",
    "vio-stale": "reason=vio_stale",
}
ZERO_DELIVERY_SCENARIOS = tuple(EXPECTED_REASONS)


def require_gcs_announcement(announced: list[ReceivedMessage]) -> None:
    """Require the recorded heartbeats to be NOMAD announcing as a GCS."""
    require(
        len(announced) >= 2,
        "NOMAD announces at about 1 Hz while unlatched",
        f"observed {len(announced)} heartbeat(s)",
    )
    first = announced[0]
    require(
        first.vehicle_type == GCS_VEHICLE_TYPE
        and first.autopilot == GCS_AUTOPILOT_TYPE
        and first.component_id == GCS_COMPONENT_ID,
        "the announcement identifies NOMAD as a GCS",
        f"type={first.vehicle_type} autopilot={first.autopilot} component={first.component_id}",
    )


def case_gcs_heartbeat_announces_to_a_silent_peer(probe: Path) -> None:
    """NOMAD must announce a GCS heartbeat before any vehicle latches.

    The peer never sends, so discovery has to fail: that failure is the
    precondition, not the assertion. The case pins that NOMAD announced itself
    as a GCS during the dead window, which is what opens a heartbeat-gated leg
    on mavlink-router, Docker's UDP relay and MAVProxy. Without the
    announcement those links stay shut and every later command is lost.

    NOMAD points straight at the peer with udpout:, which is the shape a relay
    setup uses, so the announcement has a reachable address before any peer
    traffic exists. A udpin: link has no pre-latch destination at all, since
    MAVSDK only learns one from traffic it receives, so a relay setup must name
    the relay. The pinned MAVSDK revision announces without a connected system;
    this case pins the observable behavior, so a version that only announces
    after discovery fails here instead of in the field.
    """
    port = find_free_udp_port()
    announced: list[ReceivedMessage] = []

    def action(peer: VehiclePeer) -> subprocess.CompletedProcess:
        result = run_probe(probe, port, 1, COMMAND_ARM_DISARM, "long", scheme="udpout")
        announced.extend(peer.messages("HEARTBEAT"))
        return result

    result = with_peer(port, 1, ACCEPTED, action, stream=False, bind=True)
    require(
        result.returncode != 0 and "connect=fail" in result.stdout,
        "a silent peer is never discovered",
        describe(result, []),
    )
    require_gcs_announcement(announced)


def case_coalesced_telemetry_is_verified(cli: Path) -> None:
    """A MAVProxy-style burst must not hide the heartbeat or the ack.

    The peer joins every telemetry frame and the command acknowledgement into
    one datagram, so a transport that reads only the first frame of a datagram
    loses the link and times out instead of verifying the arm.
    """
    result, observed = run_cli_case(cli, "arm", coalesce=True)
    require(
        result.returncode == 0 and "arm verified" in result.stdout,
        "the command is verified across a coalesced datagram",
        describe(result, observed),
    )


def case_link_loss_is_observed_as_stale(probe: Path) -> None:
    """A vehicle that stops heartbeating must stop looking connected (SR-LNK-01).

    The peer streams, then falls silent while its socket stays open. The probe
    requires the transition from a live link to a not-fresh, not-connected one,
    which is the link observation every link-loss response depends on.
    """
    peer_lifetime = 2
    port = find_free_udp_port()

    def action(peer: VehiclePeer) -> subprocess.CompletedProcess:
        return run_staleness_probe(probe, port, 1, peer_lifetime)

    result = with_peer(port, 1, ACCEPTED, action, telemetry_seconds=peer_lifetime)
    require(
        result.returncode == 0 and "live_before=1" in result.stdout and "fresh=false" in result.stdout,
        "a silent peer becomes stale instead of staying fresh",
        f"rc={result.returncode} stdout={result.stdout!r} stderr={result.stderr!r}",
    )


def matches_streamed_rates(record: SetpointRecord, streamed: tuple[float, float, float, float]) -> bool:
    """Compare decoded float32 rates against the requested values."""
    observed = (record.vx, record.vy, record.vz, record.yaw_rate)
    return all(abs(got - want) < 1e-6 for got, want in zip(observed, streamed))


def require_velocity_wire_format(received: list[SetpointRecord], streamed: tuple[float, float, float, float]) -> None:
    """Require a streamed setpoint followed by the zero setpoint on disconnect."""
    nonzero = [item for item in received if not item.is_zero()]
    require(len(nonzero) >= 1, "a non-zero setpoint reaches the wire", f"observed {received}")
    require(
        all(math.isfinite(value) for item in received for value in (item.vx, item.vy, item.vz, item.yaw_rate)),
        "no non-finite setpoint reaches the wire",
        f"observed {received}",
    )
    first = nonzero[0]
    require(
        matches_streamed_rates(first, streamed)
        and first.type_mask == VELOCITY_TYPE_MASK
        and first.coordinate_frame == BODY_NED_FRAME
        and first.target_system == 1,
        "the setpoint carries the body-frame velocity mask and the requested rates",
        f"observed {first}",
    )
    require(
        received[-1].is_zero(),
        "the newest setpoint on the wire is zero",
        f"last observed {received[-1]}",
    )


def case_velocity_setpoint_reaches_the_wire(probe: Path) -> None:
    """Velocity parity, and the zero setpoint the transport sends on disconnect.

    The peer decodes the frames it actually received, so this asserts the wire
    rather than the transport's own report. Both halves matter: a setpoint that
    never leaves cannot steer, and a stream that is not zeroed when NOMAD lets go
    keeps steering a vehicle nothing is watching (SR-LNK-03).
    """
    port = find_free_udp_port()
    streamed = (0.5, 0.0, -0.2, 0.3)
    received: list[SetpointRecord] = []

    def action(peer: VehiclePeer) -> subprocess.CompletedProcess:
        result = run_velocity_probe(probe, port, 1, streamed)
        received.extend(peer.setpoints())
        return result

    result = with_peer(port, 1, ACCEPTED, action)
    require(
        result.returncode == 0
        and "active_before=1" in result.stdout
        and "active_after=0" in result.stdout
        and "refused_invalid=1" in result.stdout,
        "the transport holds velocity active until it disconnects and refuses a non-finite rate",
        f"rc={result.returncode} stdout={result.stdout!r} stderr={result.stderr!r}",
    )
    require_velocity_wire_format(received, streamed)


def require_fence_polygon(held: list[tuple[float, float]], boundary) -> None:
    """Require the polygon the peer decoded to be the uploaded boundary, in order."""
    require(
        len(held) == len(boundary),
        "the fence reached the vehicle with every vertex",
        f"held {held}",
    )
    for index, (expected, actual) in enumerate(zip(boundary, held)):
        require(
            abs(actual[0] - expected[0]) < FENCE_COORDINATE_TOLERANCE
            and abs(actual[1] - expected[1]) < FENCE_COORDINATE_TOLERANCE,
            f"fence vertex {index} matches the uploaded coordinate",
            f"expected {expected} held {actual}",
        )


def case_fence_upload_and_readback(probe: Path) -> None:
    """Fence parity on the MAVSDK transport (SR-FEN-01).

    The peer stores the vertex items it is sent, so the polygon it holds is the
    wire evidence that the upload happened; the readback then proves the
    transport returns the autopilot's own copy, and FENCE_ENABLE=1 proves the
    verification path reads the enable parameter. The same run proves the
    refusal paths: a boundary below the three-vertex minimum, a non-finite
    vertex, and an expected boundary the autopilot does not hold.
    """
    port = find_free_udp_port()
    held: list[tuple[float, float]] = []

    def action(peer: VehiclePeer) -> subprocess.CompletedProcess:
        result = run_fence_probe(probe, port, 1, enabled=True)
        held.extend(peer.fence_polygon())
        return result

    result = with_peer(port, 1, ACCEPTED, action, params={"FENCE_ENABLE": 1.0})
    require(
        result.returncode == 0
        and "upload_ok=1" in result.stdout
        and "verify_ok=1" in result.stdout
        and "bad_count_refused=1" in result.stdout
        and "bad_point_refused=1" in result.stdout
        and "mismatch_refused=1" in result.stdout,
        "an enabled fence uploads, verifies, and refuses invalid or mismatched boundaries",
        f"rc={result.returncode} stdout={result.stdout!r} stderr={result.stderr!r}",
    )
    require_fence_polygon(held, FENCE_BOUNDARY)


def case_disabled_fence_does_not_verify(probe: Path) -> None:
    """A plan on the autopilot is not an enforced fence (SR-FEN-01 failure path).

    With FENCE_ENABLE=0 the polygon still reaches the vehicle, but verification
    must fail closed: NOMAD must not report a fence ArduPilot is not enforcing.
    Reporting success here would let autonomous flight proceed behind a fence
    that does not act.
    """
    port = find_free_udp_port()
    held: list[tuple[float, float]] = []

    def action(peer: VehiclePeer) -> subprocess.CompletedProcess:
        result = run_fence_probe(probe, port, 1, enabled=False)
        held.extend(peer.fence_polygon())
        return result

    result = with_peer(port, 1, ACCEPTED, action, params={"FENCE_ENABLE": 0.0})
    require(
        result.returncode == 0 and "upload_ok=1" in result.stdout and "verify_ok=0" in result.stdout,
        "the upload succeeds but verification fails when the fence is disabled",
        f"rc={result.returncode} stdout={result.stdout!r} stderr={result.stderr!r}",
    )
    require_fence_polygon(held, FENCE_BOUNDARY)


def case_zero_delivery_scenarios(zero_delivery: Path) -> None:
    """SR-LNK-03 at the core level: every stop path must zero the wire.

    The peer decodes the setpoints it received, so each scenario is judged on
    the frames that arrived: the streamed setpoint, then the all-zero one. These
    are the core-level loopback proofs for the transport that remains, and they
    include the VIO-feed death the legacy proof never covered.
    """
    for scenario in ZERO_DELIVERY_SCENARIOS:
        require_zero_delivery_scenario(zero_delivery, scenario)


def require_zero_delivery_scenario(zero_delivery: Path, scenario: str) -> None:
    port = find_free_udp_port()
    received: list[SetpointRecord] = []

    def action(peer: VehiclePeer) -> subprocess.CompletedProcess:
        result = run_zero_delivery_probe(zero_delivery, port, 1, scenario)
        received.extend(peer.setpoints())
        return result

    # link-loss needs the peer to fall silent; every other scenario needs a
    # heartbeat for the whole run, which is the peer's default.
    peer_options = {"telemetry_seconds": 2} if scenario == "link-loss" else {}
    result = with_peer(port, 1, ACCEPTED, action, **peer_options)
    require(
        result.returncode == 0 and EXPECTED_REASONS[scenario] in result.stdout,
        f"the core stops the stream and reports the reason ({scenario})",
        describe(result, []),
    )
    require_velocity_wire_format(received, STREAMED_SETPOINT)
