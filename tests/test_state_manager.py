# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""StateManager batching/rebuild semantics + a couple of SystemState helpers.

The state manager accumulates high-rate telemetry in a plain dict and only
rebuilds the immutable Pydantic snapshot at MODEL_UPDATE_INTERVAL (10 Hz), with
force_state_update bypassing the rate limit. These tests pin that contract.
"""

from __future__ import annotations

import time

from edge_core.services.models import DetectionInfo, SystemState
from edge_core.services.state import StateManager


def _fresh() -> StateManager:
    StateManager.reset_instance()
    return StateManager()


def test_instance_is_singleton_and_resettable():
    a = StateManager.instance()
    assert StateManager.instance() is a
    StateManager.reset_instance()
    assert StateManager.instance() is not a


def test_default_state():
    m = _fresh()
    s = m.get_state()
    assert s.flight_mode == "UNKNOWN"
    assert s.connected is False
    assert s.time_synced is False


def test_update_state_rebuilds_immediately_when_interval_elapsed():
    m = _fresh()  # _last_model_update == 0.0, so the first update rebuilds
    s = m.update_state(time_synced=True, flight_mode="GUIDED")
    assert s.time_synced is True
    assert s.flight_mode == "GUIDED"
    assert m._raw_state == {}  # cleared on rebuild


def test_update_state_batches_within_interval():
    m = _fresh()
    # Pin the last rebuild far in the future so the interval test never fires.
    m._last_model_update = time.monotonic() + 1000.0
    s = m.update_state(flight_mode="LOITER")
    assert s.flight_mode == "UNKNOWN"  # model not rebuilt yet
    assert m._raw_state["flight_mode"] == "LOITER"  # but raw dict holds it


def test_get_state_rebuilds_pending_after_interval():
    m = _fresh()
    m._last_model_update = time.monotonic() + 1000.0
    m.update_state(flight_mode="RTL")  # batched, not yet visible
    m._last_model_update = 0.0  # force the interval to have elapsed
    s = m.get_state()
    assert s.flight_mode == "RTL"
    assert m._raw_state == {}


def test_force_state_update_applies_immediately_and_clears_raw():
    m = _fresh()
    m._raw_state = {"battery_voltage": 12.0}
    s = m.force_state_update(armed=True)
    assert s.armed is True
    assert s.battery_voltage == 12.0
    assert m._raw_state == {}


def test_detection_info_centers_and_from_dict():
    d = DetectionInfo.from_dict(
        {
            "class_id": 1,
            "class_name": "target",
            "confidence": 0.9,
            "bbox": {"x1": 0.0, "y1": 0.0, "x2": 1.0, "y2": 0.5},
            "timestamp": 1.0,
        }
    )
    assert d.center_x == 0.5
    assert d.center_y == 0.25


def test_has_valid_gps():
    s = SystemState.default()
    assert s.has_valid_gps() is False
    s2 = s.model_copy(update={"gps_fix": True, "gps_lat": 45.0, "gps_lon": -73.0})
    assert s2.has_valid_gps() is True
