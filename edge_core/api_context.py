# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Shared route registration context for the Edge Core API.

Route modules receive one of these from ``api.create_app``.  It keeps shared
helpers in one place so extracted route files do not duplicate state mutation
or subprocess/runtime probing logic.
"""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any


class ApiRouteContext(SimpleNamespace):
    """Mutable namespace used by route-registration modules."""

    logger: Any
