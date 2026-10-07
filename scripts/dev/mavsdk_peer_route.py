# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Small deterministic route-motion model for the MAVSDK peer."""

from __future__ import annotations

import math
import time


class RouteProgress:
    """Advance toward an accepted route target over several telemetry updates."""

    def __init__(self) -> None:
        self._target: tuple[float, float, float] | None = None
        self._start_time = 0.0

    def start(self, target: tuple[float, float, float]) -> None:
        self._target = target
        self._start_time = time.monotonic() + 1.0

    def advance(self, latitude_deg: float, longitude_deg: float) -> tuple[float, float, float] | None:
        if self._target is None or time.monotonic() < self._start_time:
            return None
        target_latitude, target_longitude, target_altitude = self._target
        north_m = (target_latitude - latitude_deg) * 111_111.0
        east_m = (target_longitude - longitude_deg) * 111_111.0 * math.cos(math.radians(latitude_deg))
        distance_m = math.hypot(north_m, east_m)
        if distance_m <= 55.0:
            self._target = None
            return target_latitude, target_longitude, target_altitude
        fraction = 50.0 / distance_m
        next_latitude = latitude_deg + (target_latitude - latitude_deg) * fraction
        next_longitude = longitude_deg + (target_longitude - longitude_deg) * fraction
        return next_latitude, next_longitude, target_altitude
