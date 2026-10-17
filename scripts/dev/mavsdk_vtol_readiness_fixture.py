# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Initial VTOL telemetry must arrive before direct transition qualification."""

from __future__ import annotations

import time
from pathlib import Path

from mavsdk_fixture_harness import describe, find_binary, find_free_udp_port, require, run_cli
from mavsdk_peer import ACCEPTED, COMMAND_DO_VTOL_TRANSITION, VehiclePeer
from mavsdk_vtol_transition_fixture import PINNED_QUADPLANE_PARAMETERS
from pymavlink.dialects.v20 import ardupilotmega as mavlink


class InitialVtolPeer(VehiclePeer):
    def __init__(self, *args, delay: float | None, **kwargs):
        super().__init__(*args, **kwargs)
        self._initial_vtol_delay = delay
        self._release_vtol_at = None
        self.vtol_frames_sent = 0
        self.vtol_frames_before_transition = None

    def _handle(self, message):
        if message.get_type() == "HEARTBEAT" and message.type == mavlink.MAV_TYPE_GCS:
            if self._release_vtol_at is None:
                self._release_vtol_at = (
                    float("inf") if self._initial_vtol_delay is None else time.monotonic() + self._initial_vtol_delay
                )
        if message.get_type() in {"COMMAND_LONG", "COMMAND_INT"}:
            if int(message.command) == COMMAND_DO_VTOL_TRANSITION and self.vtol_frames_before_transition is None:
                self.vtol_frames_before_transition = self.vtol_frames_sent
        super()._handle(message)

    def _send(self, message):
        if message.get_type() == "EXTENDED_SYS_STATE":
            if self._release_vtol_at is None or time.monotonic() < self._release_vtol_at:
                return
            self.vtol_frames_sent += 1
        super()._send(message)


def run_initial_state_case(cli: Path, verb: str, delay: float | None):
    port = find_free_udp_port()
    initial = mavlink.MAV_VTOL_STATE_MC if verb == "transition-to-fixed-wing" else mavlink.MAV_VTOL_STATE_FW
    peer = InitialVtolPeer(
        port,
        1,
        ACCEPTED,
        delay=delay,
        params=PINNED_QUADPLANE_PARAMETERS,
        vehicle_type=mavlink.MAV_TYPE_FIXED_WING,
        autopilot_type=mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        initial_mode=10,
        initial_armed=True,
        vtol_state=initial,
        initial_relative_altitude_m=20.0,
    )
    peer.start()
    try:
        arguments = ("45.5017", "-73.5673", "20") if verb == "transition-to-vtol" else ()
        result = run_cli(cli, port, verb, *arguments, timeout=25)
        commands = [row for row in peer.commands() if row[1] == COMMAND_DO_VTOL_TRANSITION]
        targets = [row for row in peer.command_targets if row[0] == COMMAND_DO_VTOL_TRANSITION]
        return result, commands, targets, peer.vtol_frames_before_transition
    finally:
        peer.stop()


def case_transitions_wait_for_initial_vtol_state(cli: Path) -> None:
    for verb, target in [
        ("transition-to-fixed-wing", mavlink.MAV_VTOL_STATE_FW),
        ("transition-to-vtol", mavlink.MAV_VTOL_STATE_MC),
    ]:
        result, commands, targets, state_frames = run_initial_state_case(cli, verb, 0.75)
        require(
            result.returncode == 0 and bool(commands) and state_frames is not None and state_frames > 0,
            verb + " waits for initial VTOL state before dispatch",
            describe(result, commands),
        )
        require(
            all(
                row[0] == "COMMAND_LONG" and row[3] == (float(target), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) for row in commands
            )
            and targets == [(COMMAND_DO_VTOL_TRANSITION, 1, 1)] * len(commands),
            verb + " preserves targeted transition transport after preparation",
            describe(result, commands),
        )

    missing, commands, _targets, _state_frames = run_initial_state_case(cli, "transition-to-fixed-wing", None)
    require(
        missing.returncode != 0 and not commands and "VTOL state is unavailable" in missing.stdout,
        "missing initial VTOL state still rejects without a transition command",
        describe(missing, commands),
    )


def main() -> int:
    cli = find_binary("nomad-qualification")
    if cli is None:
        raise RuntimeError("Build the qualification driver before running VTOL readiness cases")
    case_transitions_wait_for_initial_vtol_state(cli)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
