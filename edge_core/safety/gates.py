# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Freshness and arm/mode gates for the safety-critical core.

Tier SC. Pure boolean predicates over explicit state — no ``self``, no clock of
their own (the caller passes ``now``), no I/O. This makes every gate trivially
unit-testable and fault-injectable.

Implements the predicate side of requirements SR-VEL-05, SR-VIO-01/02,
SR-LNK-01 (see ``docs/safety/requirements.md``).
"""

from __future__ import annotations


def heartbeat_fresh(last_heartbeat: float, now: float, timeout_s: float) -> bool:
    """True if an FC heartbeat was seen within ``timeout_s`` of ``now``.

    A non-positive ``last_heartbeat`` means "never seen" and is never fresh
    (SR-LNK-01). ``last_heartbeat`` and ``now`` share a monotonic clock.
    """
    if last_heartbeat <= 0.0:
        return False
    return (now - last_heartbeat) <= timeout_s


def vio_fresh(last_update: float, now: float, max_age_s: float) -> bool:
    """True if a VIO/odom update arrived within ``max_age_s`` of ``now``.

    A non-positive ``last_update`` means "never updated" and is never fresh.
    """
    if last_update <= 0.0:
        return False
    return (now - last_update) <= max_age_s


def heartbeat_from_vehicle(src_system: int, mav_type: int, target_system: int, *, gcs_type: int) -> bool:
    """True if a HEARTBEAT should drive the armed / flight-mode gate (SR-VEL-06).

    A companion MAVLink link commonly carries heartbeats from more than one
    system - the autopilot *and* a GCS (e.g. MAVProxy emits a ``MAV_TYPE_GCS``
    heartbeat with the armed bit clear). Letting those drive the gate makes the
    controller's view of armed/mode flip every other heartbeat, intermittently
    dropping valid setpoints. Only the commanded autopilot should count.

    ``gcs_type`` is ``mavutil.mavlink.MAV_TYPE_GCS`` (injected so this module
    stays free of the pymavlink dependency). ``target_system == 0`` means the
    link has not latched the autopilot yet - accept any non-GCS heartbeat so the
    link can bootstrap.
    """
    if mav_type == gcs_type:
        return False
    if target_system and src_system != target_system:
        return False
    return True


def vio_ready(
    healthy: bool,
    confidence: float,
    min_confidence: float,
    last_update: float,
    now: float,
    max_age_s: float,
) -> bool:
    """True if VIO is healthy, confident enough, and fresh (SR-VIO-01).

    All three conditions must hold to accept a velocity setpoint. ``healthy`` is
    the source's own validity flag; ``confidence`` is gated at ``min_confidence``;
    freshness reuses :func:`vio_fresh`.
    """
    return healthy and confidence >= min_confidence and vio_fresh(last_update, now, max_age_s)
