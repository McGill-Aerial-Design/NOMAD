# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for Edge Core CLI parsing."""

from __future__ import annotations

import os

from edge_core.cli import apply_cli_environment, parse_args


def test_parse_args_defaults():
    args = parse_args([])

    assert args.host == "0.0.0.0"
    assert args.port == 8000
    assert args.log_level == "info"
    assert args.sim is False


def test_parse_args_overrides():
    args = parse_args(["--host", "127.0.0.1", "--port", "9000", "--log-level", "debug", "--sim"])

    assert args.host == "127.0.0.1"
    assert args.port == 9000
    assert args.log_level == "debug"
    assert args.sim is True


def test_apply_cli_environment_sets_sim(monkeypatch):
    monkeypatch.delenv("NOMAD_SIM_MODE", raising=False)
    args = parse_args(["--sim"])

    apply_cli_environment(args)

    assert args.sim is True
    assert os.environ["NOMAD_SIM_MODE"] == "true"
