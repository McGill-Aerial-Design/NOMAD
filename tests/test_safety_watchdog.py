# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Unit tests for edge_core.safety.watchdog (SR-LNK-02, SR-VIO-02)."""

from __future__ import annotations

from edge_core.safety.watchdog import watchdog_decision


def _decide(**overrides):
    kwargs = dict(
        active=True,
        last_command_time=100.0,
        now=100.0,
        command_timeout_s=0.5,
        vio_is_fresh=True,
    )
    kwargs.update(overrides)
    return watchdog_decision(**kwargs)


def test_inactive_never_stops():
    result = _decide(active=False, last_command_time=0.0, vio_is_fresh=False)
    assert result.stop is False
    assert result.reason is None


def test_fresh_command_and_vio_does_not_stop():
    assert _decide(now=100.4).stop is False  # 0.4s < 0.5s timeout, vio fresh


def test_command_timeout_stops_with_reason():
    result = _decide(now=100.6)  # 0.6s > 0.5s timeout
    assert result.stop is True
    assert result.reason == "command timeout"


def test_at_timeout_boundary_does_not_stop():
    # Exactly at the timeout is not "> timeout".
    assert _decide(now=100.5).stop is False


def test_vio_stale_stops_with_reason():
    result = _decide(vio_is_fresh=False)
    assert result.stop is True
    assert result.reason == "VIO stale"


def test_command_timeout_takes_precedence_over_vio_stale():
    # Both conditions hold; command-timeout reason wins (historical behaviour).
    result = _decide(now=100.6, vio_is_fresh=False)
    assert result.stop is True
    assert result.reason == "command timeout"
