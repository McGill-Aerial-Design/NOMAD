# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Shared route registration context for the Edge Core API.

Route modules receive one of these from ``api.create_app``. It is a plain
attribute bag (logger + auth flags) built in :func:`api.build_route_context`;
the named alias just keeps route signatures self-documenting.
"""

from __future__ import annotations

from types import SimpleNamespace

ApiRouteContext = SimpleNamespace
