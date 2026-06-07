# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
from __future__ import annotations

from typing import Any

from ..state import StateManager
from .commands import MavlinkCommands
from .connection import MavlinkConnection


class MavlinkService(MavlinkConnection, MavlinkCommands):
    """Combines connection management and message commanding into a single service."""

    def __init__(self, state_manager: StateManager, endpoint: str | None = None) -> None:
        MavlinkConnection.__init__(self, state_manager, endpoint)
        MavlinkCommands.__init__(self)
