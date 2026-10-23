# SPDX-License-Identifier: Apache-2.0
"""Measure sampled MAVLink descent with an altitude/time cross-check."""

import math


def controller_time_advanced(previous, current):
    # Separate serial streams can deliver older samples within the admitted 3.5-second packet age.
    if current < previous - 3500:
        raise ValueError("Controller boot time regressed beyond the telemetry freshness window")
    return current > previous


def measure_descent(samples, minimum, maximum):
    """Validate rows (boot milliseconds, relative metres, downward metres/second)."""
    try:
        minimum, maximum = float(minimum), float(maximum)
        rows = [tuple(float(value) for value in row) for row in samples]
    except (TypeError, ValueError, OverflowError):
        raise ValueError("Descent samples and limits must be numeric")
    if any(math.isnan(value) or math.isinf(value) for value in (minimum, maximum)):
        raise ValueError("Descent limits must be finite")
    if not 0 < minimum <= maximum:
        raise ValueError("Descent limits must be positive and ordered")
    previous = -1.0
    for row in rows:
        if len(row) != 3 or any(math.isnan(value) or math.isinf(value) for value in row):
            raise ValueError("Each descent sample must contain three finite values")
        if row[0] < 0 or row[0] <= previous:
            raise ValueError("Controller sample times must be nonnegative and strictly increasing")
        previous = row[0]
    result = {}
    for name, low, high in (("high", 12, 18), ("final", 1, 5)):
        band = [row for row in rows if low <= row[1] <= high]
        if len(band) < 3:
            raise ValueError(name + " descent band needs at least three samples")
        duration = (band[-1][0] - band[0][0]) / 1000.0
        if duration < 0.5:
            raise ValueError(name + " descent band needs at least 0.5 seconds")
        if band[0][1] < high - 0.6 or band[-1][1] > low + 0.6:
            raise ValueError(name + " altitude band coverage is incomplete")
        if any(current[0] - previous[0] > 500 for previous, current in zip(band, band[1:])):
            raise ValueError(name + " descent band sample gap exceeds 500 milliseconds")
        slowest = min(row[2] for row in band)
        fastest = max(row[2] for row in band)
        if slowest < minimum or fastest > maximum:
            raise ValueError(name + " signed downward velocity is outside the limits")
        altitude_rate = (band[0][1] - band[-1][1]) / duration
        if not minimum <= altitude_rate <= maximum:
            raise ValueError(name + " altitude/time descent is outside the limits")
        result[name] = {
            "samples": len(band),
            "duration_seconds": duration,
            "vz_min_mps": slowest,
            "vz_max_mps": fastest,
            "altitude_rate_mps": altitude_rate,
        }
    return result
