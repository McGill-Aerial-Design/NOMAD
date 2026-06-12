# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for shared environment parsing helpers."""

from __future__ import annotations

from edge_core.env import env_bool, env_secret, parse_bool


def test_parse_bool_accepts_common_true_values():
    for value in ("1", "true", "TRUE", " yes ", "on"):
        assert parse_bool(value) is True


def test_parse_bool_uses_default_only_when_missing():
    assert parse_bool(None, default=True) is True
    assert parse_bool("false", default=True) is False
    assert parse_bool("", default=True) is False


def test_env_bool_reads_from_mapping():
    assert env_bool("ENABLED", source={"ENABLED": "yes"}) is True
    assert env_bool("MISSING", default=True, source={}) is True


def test_env_secret_strips_blank_values():
    assert env_secret("TOKEN", source={"TOKEN": "  abc  "}) == "abc"
    assert env_secret("TOKEN", source={"TOKEN": "   "}) is None
    assert env_secret("MISSING", source={}) is None
