# SPDX-License-Identifier: Apache-2.0
import json
import runpy
from pathlib import Path

import pytest

helpers = runpy.run_path(str(Path(__file__).resolve().parents[1] / "conference/descent.py"))
measure_descent = helpers["measure_descent"]
controller_time_advanced = helpers["controller_time_advanced"]


def test_route_switch_ignores_old_samples_without_lowering_the_high_water_mark():
    high_water = 100000
    accepted = []
    for stamp in (99900, 100000, 100100):
        if controller_time_advanced(high_water, stamp):
            high_water = stamp
            accepted.append(stamp)
    assert accepted == [100100]
    assert high_water == 100100


def test_controller_restart_cannot_pass_as_a_route_switch():
    with pytest.raises(ValueError, match="boot time regressed"):
        controller_time_advanced(100000, 0)


def test_stale_sample_tolerance_ends_at_the_packet_freshness_boundary():
    assert not controller_time_advanced(100000, 96500)
    with pytest.raises(ValueError, match="freshness window"):
        controller_time_advanced(100000, 96499)


def descent(rate):
    return [(index * 250, 20 - rate * index / 4.0, rate) for index in range(int(84 / rate) + 1)]


@pytest.mark.parametrize("rate,minimum,maximum", [(0.5, 0.2, 1), (2.2, 2, 2.7)])
def test_realistic_signed_descent_and_altitude_cross_check_agree(rate, minimum, maximum):
    result = measure_descent(descent(rate), minimum, maximum)
    assert set(result) == {"high", "final"}
    for band in result.values():
        assert band["samples"] >= 3
        assert band["duration_seconds"] >= 0.5
        assert band["vz_min_mps"] == rate
        assert band["vz_max_mps"] == rate
        assert band["altitude_rate_mps"] == pytest.approx(rate)
    json.dumps(result, allow_nan=False)


def test_height_and_rate_boundaries_are_inclusive():
    rows = [(index * 500, 18 - index * 0.5, 1) for index in range(35)]
    result = measure_descent(rows, 1, 1)
    assert result["high"]["samples"] == 13
    assert result["final"]["samples"] == 9
    assert result["high"]["duration_seconds"] == 6
    assert result["final"]["duration_seconds"] == 4


def test_half_second_partial_band_is_not_complete_coverage():
    rows = [(0, 18, 1), (250, 17.75, 1), (500, 17.5, 1), (13000, 5, 1), (13250, 4.75, 1), (13500, 4.5, 1)]
    with pytest.raises(ValueError, match="coverage"):
        measure_descent(rows, 1, 1)


def test_full_height_coverage_cannot_hide_a_missing_sample_interval():
    rows = [row for row in descent(2.2) if not 15 < row[1] < 16]
    with pytest.raises(ValueError, match="gap exceeds 500"):
        measure_descent(rows, 2, 2.7)


def test_coverage_edge_tolerance_is_inclusive():
    rows = [(index * 200, 17.4 - index * 0.1, 0.5) for index in range(49)]
    rows += [(26000 + index * 200, 4.4 - index * 0.1, 0.5) for index in range(29)]
    result = measure_descent(rows, 0.2, 1)
    assert result["high"]["altitude_rate_mps"] == pytest.approx(0.5)
    assert result["final"]["altitude_rate_mps"] == pytest.approx(0.5)


@pytest.mark.parametrize("band", ["high", "final"])
def test_missing_upper_or_lower_edge_rejected(band):
    rows = descent(2.2)
    if band == "high":
        rows = [row for row in rows if not 17.3 < row[1] <= 18]
    else:
        rows = [row for row in rows if not 1 <= row[1] < 1.7]
    with pytest.raises(ValueError, match=band + " altitude band coverage"):
        measure_descent(rows, 2, 2.7)


@pytest.mark.parametrize("reported", [-0.5, 0.5])
def test_climb_rejected_even_when_reported_velocity_falsely_says_descent(reported):
    rows = [(index * 250, 1 + 0.5 * index / 4.0, reported) for index in range(161)]
    with pytest.raises(ValueError, match="velocity|altitude"):
        measure_descent(rows, 0.2, 1)


def test_altitude_cross_check_rejects_inconsistent_velocity():
    rows = [(stamp, altitude, 0.5) for stamp, altitude, _ in descent(2.2)]
    with pytest.raises(ValueError, match="altitude/time"):
        measure_descent(rows, 0.2, 1)


def test_one_outlying_velocity_cannot_hide_in_an_average():
    rows = descent(2.2)
    index = next(i for i, row in enumerate(rows) if 12 <= row[1] <= 18)
    stamp, altitude, _ = rows[index]
    rows[index] = (stamp, altitude, 2.71)
    with pytest.raises(ValueError, match="velocity"):
        measure_descent(rows, 2, 2.7)


def test_fast_altitude_loss_rejects_falsely_bounded_velocity():
    rows = [(stamp, altitude, 2.2) for stamp, altitude, _ in descent(4)]
    with pytest.raises(ValueError, match="altitude/time"):
        measure_descent(rows, 2, 2.7)


def test_missing_final_band_is_not_completed_descent():
    with pytest.raises(ValueError, match="final"):
        measure_descent([row for row in descent(0.5) if row[1] > 5], 0.2, 1)


@pytest.mark.parametrize("stamp", [0, -1])
def test_duplicate_and_stale_controller_time_rejected(stamp):
    rows = descent(0.5)
    rows[1] = (stamp, rows[1][1], rows[1][2])
    with pytest.raises(ValueError, match="strictly increasing"):
        measure_descent(rows, 0.2, 1)


@pytest.mark.parametrize("column", [0, 1, 2])
@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_samples_rejected_even_outside_measurement_bands(column, value):
    rows = descent(0.5)
    row = list(rows[0])
    row[column] = value
    rows[0] = row
    with pytest.raises(ValueError, match="finite"):
        measure_descent(rows, 0.2, 1)


@pytest.mark.parametrize("limits", [(0, 1), (2, 1), (1, float("nan")), (1, float("inf"))])
def test_invalid_limits_rejected(limits):
    with pytest.raises(ValueError, match="limits"):
        measure_descent(descent(0.5), *limits)


def test_three_samples_without_half_second_dwell_are_insufficient():
    rows = [(0, 18, 2.2), (100, 17.78, 2.2), (200, 17.56, 2.2), (6000, 5, 2.2), (6500, 3.9, 2.2), (7000, 2.8, 2.2)]
    with pytest.raises(ValueError, match="0.5 seconds"):
        measure_descent(rows, 2, 2.7)


def test_two_samples_are_insufficient_even_with_long_dwell():
    rows = [(0, 18, 0.5), (12000, 12, 0.5), (26000, 5, 0.5), (30000, 3, 0.5), (34000, 1, 0.5)]
    with pytest.raises(ValueError, match="three samples"):
        measure_descent(rows, 0.2, 1)
