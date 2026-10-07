# SPDX-License-Identifier: Apache-2.0
"""Transition-only state used by the deterministic MAVSDK peer."""

from __future__ import annotations

from dataclasses import dataclass

from pymavlink.dialects.v20 import ardupilotmega as mavlink


@dataclass
class TransitionModel:
    vtol_state: int
    reaches_fixed_wing: bool
    reaches_multicopter: bool
    reports_intermediate: bool
    finish_at: float | None = None
    finish_state: int | None = None

    @classmethod
    def create(
        cls,
        vtol_state: int | None,
        vehicle_type: int,
        params: dict[str, float],
        reaches_fixed_wing: bool,
        reaches_multicopter: bool,
        reports_intermediate: bool,
    ) -> TransitionModel:
        if vtol_state is None:
            is_quadplane = vehicle_type == mavlink.MAV_TYPE_FIXED_WING and params.get("Q_ENABLE") in (1.0, 2.0)
            vtol_state = mavlink.MAV_VTOL_STATE_MC if is_quadplane else mavlink.MAV_VTOL_STATE_UNDEFINED
        return cls(vtol_state, reaches_fixed_wing, reaches_multicopter, reports_intermediate)

    def request(self, target_state: int, requested_at: float) -> None:
        if target_state == mavlink.MAV_VTOL_STATE_FW:
            reaches_target = self.reaches_fixed_wing
            intermediate_state = mavlink.MAV_VTOL_STATE_TRANSITION_TO_FW
        elif target_state == mavlink.MAV_VTOL_STATE_MC:
            reaches_target = self.reaches_multicopter
            intermediate_state = mavlink.MAV_VTOL_STATE_TRANSITION_TO_MC
        else:
            return

        self.finish_at = None
        self.finish_state = None
        if not self.reports_intermediate:
            if reaches_target:
                self.vtol_state = target_state
            return

        self.vtol_state = intermediate_state
        if reaches_target:
            self.finish_at = requested_at + 0.6
            self.finish_state = target_state

    def advance(self, now: float) -> None:
        if self.finish_at is None or now < self.finish_at:
            return
        self.vtol_state = self.finish_state
        self.finish_at = None
        self.finish_state = None
