# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Small helpers for reading environment-style configuration."""

from __future__ import annotations

import os
from collections.abc import Mapping

TRUE_VALUES = {"1", "true", "yes", "on"}


def parse_bool(value: str | None, default: bool = False) -> bool:
    if value is None:
        return default
    return value.strip().lower() in TRUE_VALUES


def env_bool(name: str, default: bool = False, source: Mapping[str, str] | None = None) -> bool:
    values = source if source is not None else os.environ
    return parse_bool(values.get(name), default)


def env_secret(name: str, source: Mapping[str, str] | None = None) -> str | None:
    values = source if source is not None else os.environ
    return (values.get(name) or "").strip() or None
