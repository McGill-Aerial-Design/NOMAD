# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""MediaMTX contract: ONE always-on stream that any number of viewers can read.

The Jetson is supposed to expose a single RTSP stream (no /primary or
/secondary). MediaMTX serves it from a ``source: publisher`` path, which fans
out to an unbounded set of concurrent readers, and ``sourceOnDemand: no`` keeps
it running so consumers can attach at any time. These tests pin that contract so
a future edit can't silently reintroduce a second path or make the stream
on-demand (which would tear it down between viewers).
"""

from __future__ import annotations

from pathlib import Path

import pytest

_MEDIAMTX = Path(__file__).resolve().parents[1] / "infra" / "mediamtx.yml"


def test_no_primary_or_secondary_paths():
    text = _MEDIAMTX.read_text(encoding="utf-8")
    assert "primary:" not in text
    assert "secondary:" not in text
    assert "stream:" in text


def test_single_stream_publisher_path():
    yaml = pytest.importorskip("yaml")
    cfg = yaml.safe_load(_MEDIAMTX.read_text(encoding="utf-8"))
    paths = cfg["paths"]

    # Exactly one stream path.
    assert set(paths) == {"stream"}

    stream = paths["stream"]
    # publisher source => MediaMTX relays the one publisher to many readers
    # (multiple consumers), and always-on so viewers can join/leave freely.
    assert stream["source"] == "publisher"
    assert stream["sourceOnDemand"] is False
