# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Phase B transport parity tests driven by the deterministic MAVSDK peer.

These run the real `nomad` CLI and the connection probe
binary against scripts/dev/mavsdk_connection_fixture.py. They skip when the built
binaries are absent (run `pixi run test-mavsdk-phase-b` first); the fixture's own
run is the recorded evidence.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts" / "dev"
sys.path.insert(0, str(SCRIPTS))

import mavsdk_connection_fixture as fixture  # noqa: E402
from mavsdk_vtol_transition_fixture import (  # noqa: E402
    case_quadplane_transition_to_vtol_is_verified,
    case_transition_directions_use_the_same_targeted_command_transport,
)

CLI = fixture.find_binary("nomad")
PROBE = fixture.find_binary("nomad_mavsdk_connection_tests")
ZERO_DELIVERY = fixture.find_binary("nomad_mavsdk_zero_delivery_tests")

requires_mavsdk_build = pytest.mark.skipif(
    CLI is None or PROBE is None or ZERO_DELIVERY is None,
    reason="MAVSDK Phase B binaries not built; run `pixi run build-core-mavsdk`",
)


@requires_mavsdk_build
def test_status_telemetry_parity() -> None:
    fixture.case_status(CLI)


@requires_mavsdk_build
def test_unlatched_link_announces_a_gcs_heartbeat() -> None:
    """A heartbeat-gated relay opens only after NOMAD announces a GCS heartbeat.

    Ported from the legacy UDP transport's pre-latch announcement test, which is
    the behavior this branch exists to protect.
    """
    fixture.case_gcs_heartbeat_announces_to_a_silent_peer(PROBE)


@requires_mavsdk_build
def test_coalesced_telemetry_and_ack_are_handled() -> None:
    """A MAVProxy-style burst must not starve the heartbeat or the ack.

    Ported from the legacy UDP transport's coalesced-datagram tests.
    """
    fixture.case_coalesced_telemetry_is_verified(CLI)


@requires_mavsdk_build
def test_link_loss_is_observed_as_stale() -> None:
    """A vehicle that stops heartbeating must stop looking connected (SR-LNK-01)."""
    fixture.case_link_loss_is_observed_as_stale(PROBE)


@requires_mavsdk_build
def test_velocity_reaches_the_wire_and_is_zeroed_on_disconnect() -> None:
    """Velocity parity plus the zero-on-disconnect safety behavior (SR-LNK-03).

    Ported from the legacy transport's zero-delivery proof, which needs a
    transport that can actually send a setpoint.
    """
    fixture.case_velocity_setpoint_reaches_the_wire(PROBE)


@requires_mavsdk_build
def test_fence_uploads_reads_back_and_refuses_invalid_boundaries() -> None:
    """SR-FEN-01: the core fence path over the MAVSDK transport.

    Fence parity: a boundary reaches the vehicle as fence mission items, the
    readback matches, and boundaries that are too small, non-finite or different
    from the autopilot's copy are refused.
    """
    fixture.case_fence_upload_and_readback(PROBE)


@requires_mavsdk_build
def test_disabled_fence_never_verifies() -> None:
    """SR-FEN-01 failure path: an uploaded plan is not an enforced fence."""
    fixture.case_disabled_fence_does_not_verify(PROBE)


@requires_mavsdk_build
def test_zero_delivery_reaches_the_wire_on_every_stop_path() -> None:
    """SR-LNK-03: watchdog, caller stop, destruction, link loss and VIO loss.

    The core-level loopback proof on the transport that remains, after the
    legacy codec and its equivalent test were deleted at the Phase E cutover.
    """
    fixture.case_zero_delivery_scenarios(ZERO_DELIVERY)


@requires_mavsdk_build
def test_arm_acknowledgement_paths() -> None:
    fixture.case_arm_accepted(CLI)
    fixture.case_arm_denied(CLI)
    fixture.case_arm_timeout(CLI)


@requires_mavsdk_build
def test_command_timeout_honors_caller_budget() -> None:
    fixture.case_command_timeout_honors_caller_budget(PROBE)


@requires_mavsdk_build
def test_parameter_timeout_honors_caller_budget() -> None:
    fixture.case_param_timeout_honors_caller_budget(PROBE)


@requires_mavsdk_build
def test_command_wire_forms() -> None:
    fixture.case_command_int(PROBE)
    fixture.case_command_long(PROBE)


@requires_mavsdk_build
def test_wrong_autopilot_identity_is_refused() -> None:
    fixture.case_wrong_system(PROBE)


@requires_mavsdk_build
def test_mode_change_is_verified_from_state() -> None:
    fixture.case_mode_is_verified(CLI)


@requires_mavsdk_build
def test_takeoff_is_verified_from_altitude() -> None:
    fixture.case_takeoff_is_verified(CLI)


@requires_mavsdk_build
def test_quadplane_vtol_takeoff_is_verified_from_climb_state() -> None:
    fixture.case_quadplane_vtol_takeoff_is_verified(CLI)


@requires_mavsdk_build
def test_quadplane_transition_is_verified_from_vtol_state() -> None:
    fixture.case_quadplane_transition_is_verified(CLI)


@requires_mavsdk_build
def test_quadplane_transition_to_vtol_command_wire_and_ack() -> None:
    case_quadplane_transition_to_vtol_is_verified(CLI)


@requires_mavsdk_build
def test_quadplane_transition_directions_use_targeted_command_long() -> None:
    case_transition_directions_use_the_same_targeted_command_transport(CLI)


@requires_mavsdk_build
def test_quadplane_fixed_wing_route_wire_protocol_and_completion() -> None:
    fixture.case_quadplane_fixed_wing_route_uses_reposition_command_int(CLI)


@requires_mavsdk_build
def test_quadplane_fixed_wing_recovery_wire_protocol_and_completion() -> None:
    fixture.case_quadplane_fixed_wing_recovery_uses_reposition_command_int(CLI)


@requires_mavsdk_build
def test_quadplane_fixed_wing_recovery_denied_ack() -> None:
    fixture.case_quadplane_fixed_wing_recovery_rejects_denied_ack(CLI)


@requires_mavsdk_build
def test_goto_is_verified_from_position() -> None:
    fixture.case_goto_is_verified(CLI)


@requires_mavsdk_build
def test_land_and_rtl_are_verified_from_mode() -> None:
    fixture.case_rtl_is_verified(CLI)
    fixture.case_land_is_verified(CLI)


@requires_mavsdk_build
def test_output_commands_reach_the_wire() -> None:
    fixture.case_servo_parity(CLI)
    fixture.case_relay_parity(CLI)
    fixture.case_gimbal_config_parity(CLI)
    fixture.case_motor_test_parity(CLI)
