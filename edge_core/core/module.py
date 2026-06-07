# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""The NOMAD module interface.

A module is any object that exposes :class:`ModuleMetadata` as ``metadata`` and
implements the lifecycle hooks below. Modules can be plain classes, instances, or
factory callables registered via the ``nomad.modules`` entry-point group (see
:mod:`edge_core.core.registry`).

Lifecycle (driven by the :class:`~edge_core.core.registry.ModuleRegistry`):

1. ``configure(ctx)``      — read config, grab services from the AppContext.
2. ``register_routes(app)`` — add FastAPI routes/routers to the app.
3. ``start()``             — begin background work (called on app startup).
4. ``stop()``              — tear down (called on app shutdown, reverse order).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol, runtime_checkable

from .context import AppContext


@dataclass(frozen=True)
class ModuleMetadata:
    """Static description of a module."""

    name: str
    version: str = "0.0.0"
    description: str = ""
    # Names of other modules that must load (and start) before this one.
    requires: tuple[str, ...] = ()
    # Optional NOMAD_* env flag that gates the module. When set, the module is
    # enabled according to the flag's value. Mirrors the existing NOMAD_ENABLE_*
    # conventions (e.g. "NOMAD_ENABLE_ISAAC_ROS").
    enable_flag: str | None = None
    # Value used when ``enable_flag`` is set but absent from the environment.
    # True => opt-out (on unless disabled); False => opt-in (off unless enabled).
    enabled_by_default: bool = True


@runtime_checkable
class NomadModule(Protocol):
    """Structural type a module must satisfy."""

    metadata: ModuleMetadata

    def configure(self, ctx: AppContext) -> None: ...
    def register_routes(self, app: Any) -> None: ...
    def start(self) -> None: ...
    def stop(self) -> None: ...


class BaseModule:
    """Convenience base class with no-op hooks — override only what you need."""

    metadata: ModuleMetadata = ModuleMetadata(name="unnamed")

    def configure(self, ctx: AppContext) -> None:  # noqa: D401 - simple default
        """Read config / grab services. Default: no-op."""

    def register_routes(self, app: Any) -> None:
        """Add routes to the FastAPI app. Default: no-op."""

    def start(self) -> None:
        """Start background work. Default: no-op."""

    def stop(self) -> None:
        """Tear down. Default: no-op."""
