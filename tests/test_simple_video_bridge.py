# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for the video bridge command-line entry point."""

from __future__ import annotations

import sys
from functools import partial

import pytest

from python.tools import simple_video_bridge as bridge_cli


class NoThread:
    """Run the HTTP serving thread inline so main() stays synchronous."""

    def __init__(self, *args, target=None, **kwargs):
        self._target = target

    def start(self):
        if self._target is not None:
            self._target()


class FakeServer:
    """Record serve/shutdown calls instead of binding a socket."""

    def __init__(self, host, port, bridge, *, seen):
        self.bridge = bridge
        self.seen = seen
        seen["host"] = host

    def serve_forever(self):
        self.seen["served"] = True

    def shutdown(self):
        self.seen["shutdown"] = True


def run_main(monkeypatch, argv: list[str]) -> None:
    monkeypatch.setattr(sys, "argv", ["prog", *argv])
    bridge_cli.main()


def capture_bridge_arguments(monkeypatch) -> dict:
    """Replace VideoBridge with a stub that records its constructor arguments."""
    seen: dict = {}

    class FakeBridge:
        def __init__(self, **kwargs):
            seen.update(kwargs)
            self.running = False

        def start(self):
            return False

    monkeypatch.setattr(bridge_cli, "VideoBridge", FakeBridge)
    return seen


@pytest.mark.parametrize("host", ["127.0.0.1", "127.42.0.9", "::1", "localhost"])
def test_validate_http_host_accepts_loopback(host):
    assert bridge_cli.validate_http_host(host) == host


@pytest.mark.parametrize("host", ["0.0.0.0", "::", "192.0.2.1", "bridge.example"])
def test_validate_http_host_rejects_non_loopback(host):
    with pytest.raises(ValueError, match="loopback"):
        bridge_cli.validate_http_host(host)


def test_main_returns_when_pipeline_fails_to_start(monkeypatch):
    monkeypatch.setattr(bridge_cli.VideoBridge, "start", lambda self: False)

    run_main(monkeypatch, ["--source-topic", "/zed/img"])


def test_main_uses_rtsp_url_argument(monkeypatch):
    seen = capture_bridge_arguments(monkeypatch)

    run_main(monkeypatch, ["--source-topic", "/zed/img", "--rtsp-url", "rtsp://m/stream"])

    assert seen["rtsp_url"] == "rtsp://m/stream"


def test_main_uses_flip_method_argument(monkeypatch):
    seen = capture_bridge_arguments(monkeypatch)

    run_main(monkeypatch, ["--source-topic", "/zed/img", "--flip-method", "rotate-180"])

    assert seen["flip_method"] == "rotate-180"


def test_main_uses_rtsp_url_env(monkeypatch):
    seen = capture_bridge_arguments(monkeypatch)
    monkeypatch.setenv("NOMAD_VIDEO_RTSP_PUBLISH_URL", "rtsp://mediamtx:8554/stream")

    run_main(monkeypatch, ["--source-topic", "/zed/img"])

    assert seen["rtsp_url"] == "rtsp://mediamtx:8554/stream"


def test_main_serves_then_shuts_down(monkeypatch):
    seen: dict = {"stopped": False, "shutdown": False, "served": False}
    monkeypatch.setattr(bridge_cli.VideoBridge, "start", lambda self: True)
    running_sequence = iter([True, False])
    monkeypatch.setattr(bridge_cli.VideoBridge, "running", property(lambda self: next(running_sequence, False)))
    monkeypatch.setattr(bridge_cli.VideoBridge, "stop", lambda self: seen.__setitem__("stopped", True))
    monkeypatch.setattr(bridge_cli, "BridgeHTTPServer", partial(FakeServer, seen=seen))
    monkeypatch.setattr(bridge_cli.threading, "Thread", NoThread)
    monkeypatch.setattr(bridge_cli.time, "sleep", lambda *_: None)

    run_main(monkeypatch, ["--source-topic", "/zed/img", "--http-port", "0"])

    assert seen["served"] is True
    assert seen["stopped"] is True
    assert seen["shutdown"] is True
    assert seen["host"] == "127.0.0.1"


def test_main_handles_keyboard_interrupt(monkeypatch):
    seen: dict = {"stopped": False, "shutdown": False, "served": False}
    monkeypatch.setattr(bridge_cli.VideoBridge, "start", lambda self: True)
    monkeypatch.setattr(bridge_cli.VideoBridge, "running", property(lambda self: True))
    monkeypatch.setattr(bridge_cli.VideoBridge, "stop", lambda self: seen.__setitem__("stopped", True))

    def interrupt(*_args):
        raise KeyboardInterrupt

    monkeypatch.setattr(bridge_cli, "BridgeHTTPServer", partial(FakeServer, seen=seen))
    monkeypatch.setattr(bridge_cli.threading, "Thread", NoThread)
    monkeypatch.setattr(bridge_cli.time, "sleep", interrupt)

    run_main(monkeypatch, ["--source-topic", "/zed/img", "--http-port", "0"])

    assert seen["stopped"] is True
    assert seen["shutdown"] is True
