# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""AppContext — the service + config registry handed to NOMAD modules.

A module receives an :class:`AppContext` in ``configure(ctx)`` and uses it to
reach core services (state, mavlink, nav, health, video, ...) and read
configuration. Services registered explicitly take precedence; if a name is not
registered, the context falls back to the matching ``app.state`` attribute.
"""

from __future__ import annotations

import os
from typing import Any

from ..env import parse_bool


class AppContext:
    """Holds the FastAPI app, a named service registry, and config access."""

    def __init__(
        self,
        app: Any = None,
        *,
        services: dict[str, Any] | None = None,
        config: dict[str, str] | None = None,
    ) -> None:
        self.app = app
        self._services: dict[str, Any] = dict(services or {})
        # Config defaults to the process environment — config/nomad.env is sourced
        # into the environment by the service scripts and systemd EnvironmentFile.
        self._config: dict[str, str] = dict(config if config is not None else os.environ)

    # -- services -----------------------------------------------------------
    def register_service(self, name: str, service: Any) -> None:
        """Expose a service to modules under ``name``."""
        self._services[name] = service

    def get_service(self, name: str, default: Any = None) -> Any:
        """Return a registered service, falling back to ``app.state.<name>``."""
        if name in self._services:
            return self._services[name]
        state = getattr(self.app, "state", None)
        if state is not None:
            found = getattr(state, name, None)
            if found is not None:
                return found
        return default

    def require_service(self, name: str) -> Any:
        """Like :meth:`get_service` but raise if the service is unavailable."""
        service = self.get_service(name)
        if service is None:
            raise KeyError(f"required service not available: {name!r}")
        return service

    # -- config -------------------------------------------------------------
    def get_config(self, key: str, default: str | None = None) -> str | None:
        return self._config.get(key, default)

    def is_enabled(self, key: str, default: bool = False) -> bool:
        """Interpret a ``NOMAD_*`` flag as a boolean (true/1/yes/on)."""
        return parse_bool(self._config.get(key), default)
