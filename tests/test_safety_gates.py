# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Unit tests for edge_core.safety.gates (SR-LNK-01, SR-VIO-01)."""

from __future__ import annotations

from edge_core.safety import gates


def test_heartbeat_never_seen_is_not_fresh():
    assert gates.heartbeat_fresh(0.0, now=100.0, timeout_s=3.0) is False
    assert gates.heartbeat_fresh(-1.0, now=100.0, timeout_s=3.0) is False


def test_heartbeat_fresh_within_and_at_boundary():
    assert gates.heartbeat_fresh(98.0, now=100.0, timeout_s=3.0) is True
    # Exactly at the timeout is still fresh (<=).
    assert gates.heartbeat_fresh(97.0, now=100.0, timeout_s=3.0) is True


def test_heartbeat_stale_past_timeout():
    assert gates.heartbeat_fresh(96.0, now=100.0, timeout_s=3.0) is False


def test_vio_never_updated_is_not_fresh():
    assert gates.vio_fresh(0.0, now=100.0, max_age_s=1.0) is False


def test_vio_fresh_within_and_stale_past_window():
    assert gates.vio_fresh(99.5, now=100.0, max_age_s=1.0) is True
    assert gates.vio_fresh(99.0, now=100.0, max_age_s=1.0) is True  # boundary
    assert gates.vio_fresh(98.5, now=100.0, max_age_s=1.0) is False


def test_vio_ready_requires_all_three_conditions():
    base = dict(min_confidence=0.3, last_update=100.0, now=100.0, max_age_s=1.0)
    # Healthy, confident, fresh -> ready.
    assert gates.vio_ready(healthy=True, confidence=0.5, **base) is True
    # Not healthy -> not ready.
    assert gates.vio_ready(healthy=False, confidence=0.5, **base) is False
    # Below confidence threshold -> not ready.
    assert gates.vio_ready(healthy=True, confidence=0.2, **base) is False
    # At threshold -> ready (>=).
    assert gates.vio_ready(healthy=True, confidence=0.3, **base) is True


GCS = 6  # mavutil.mavlink.MAV_TYPE_GCS
QUAD = 2  # mavutil.mavlink.MAV_TYPE_QUADROTOR


def test_heartbeat_from_vehicle_accepts_commanded_autopilot():
    assert gates.heartbeat_from_vehicle(1, QUAD, target_system=1, gcs_type=GCS) is True


def test_heartbeat_from_vehicle_rejects_gcs():
    # A GCS heartbeat on the same link (e.g. MAVProxy sys 255) must not count.
    assert gates.heartbeat_from_vehicle(255, GCS, target_system=1, gcs_type=GCS) is False


def test_heartbeat_from_vehicle_rejects_other_system():
    assert gates.heartbeat_from_vehicle(2, QUAD, target_system=1, gcs_type=GCS) is False


def test_heartbeat_from_vehicle_bootstraps_before_target_latched():
    # target_system == 0 -> not yet latched; accept any non-GCS heartbeat.
    assert gates.heartbeat_from_vehicle(1, QUAD, target_system=0, gcs_type=GCS) is True
    assert gates.heartbeat_from_vehicle(255, GCS, target_system=0, gcs_type=GCS) is False


def test_vio_ready_false_when_stale():
    assert (
        gates.vio_ready(
            healthy=True,
            confidence=1.0,
            min_confidence=0.3,
            last_update=90.0,
            now=100.0,
            max_age_s=1.0,
        )
        is False
    )
