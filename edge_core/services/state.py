# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Edge Core - State Manager

Thread-safe singleton state manager for system telemetry and status.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import threading
import time
from datetime import datetime, timezone
from typing import Any

from .models import SystemState


class StateManager:
    """
    Thread-safe singleton state manager for NOMAD Edge Core.

    Manages:
    - SystemState: Telemetry and sensor data (immutable snapshots)
    - Raw telemetry dict for high-frequency updates (50-70Hz)
    - Batched Pydantic model updates at 10Hz to reduce GC pressure
    """

    _instance: StateManager | None = None
    _instance_lock = threading.Lock()
    MODEL_UPDATE_INTERVAL = 0.1  # 10Hz max for Pydantic model updates

    def __init__(self) -> None:
        self._state = SystemState.default()
        self._lock = threading.RLock()
        self._raw_state: dict[str, Any] = {}  # Fast mutable dict for batching
        self._last_model_update: float = 0.0

    @classmethod
    def instance(cls) -> StateManager:
        if cls._instance is None:
            with cls._instance_lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        """Reset the singleton instance (for testing)."""
        with cls._instance_lock:
            cls._instance = None

    def get_state(self) -> SystemState:
        with self._lock:
            now = time.monotonic()
            if self._raw_state and now - self._last_model_update >= self.MODEL_UPDATE_INTERVAL:
                self._rebuild_state_locked(now)
            return self._state

    def update_state(self, **fields: Any) -> SystemState:
        """
        Update system state with rate-limited Pydantic model rebuilding.

        High-frequency telemetry (50-70Hz) is accumulated in _raw_state dict
        without Pydantic overhead. The model is rebuilt at 10Hz intervals.
        This reduces memory allocation pressure and GC pauses from model creation.

        Args:
            **fields: State fields to update

        Returns:
            Current SystemState (may be up to 100ms stale for telemetry fields)
        """
        with self._lock:
            # Always update raw state immediately (fast dict operation)
            self._raw_state.update(fields)

            # Only rebuild Pydantic model at configured interval (10Hz)
            now = time.monotonic()
            if now - self._last_model_update >= self.MODEL_UPDATE_INTERVAL:
                self._rebuild_state_locked(now)

            return self._state

    def force_state_update(self, **fields: Any) -> SystemState:
        """
        Force immediate Pydantic model rebuild (for critical updates).

        Use this for important state changes that need immediate visibility
        (e.g., flight mode changes, armed status) rather than the rate-limited
        update_state().

        Args:
            **fields: State fields to update

        Returns:
            Updated SystemState with changes applied immediately
        """
        with self._lock:
            self._raw_state.update(fields)
            data = self._state.model_dump()
            data.update(self._raw_state)
            data["timestamp"] = datetime.now(timezone.utc)
            self._state = SystemState(**data)
            self._raw_state.clear()
            self._last_model_update = time.monotonic()
            return self._state

    def _rebuild_state_locked(self, monotonic_now: float) -> None:
        """Apply pending raw fields to the immutable SystemState snapshot."""
        data = self._state.model_dump()
        data.update(self._raw_state)
        data["timestamp"] = datetime.now(timezone.utc)
        self._state = SystemState(**data)
        self._raw_state.clear()
        self._last_model_update = monotonic_now
