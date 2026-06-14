# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for edge_core.services.ros.simple_video_bridge.

The bridge shells out to ``gst-launch-1.0`` and binds an HTTP API; both are
stubbed here so the pipeline lifecycle, status accounting, and HTTP routing are
exercised without GStreamer, ROS, or real sockets.
"""

from __future__ import annotations

import io
import json
import sys
import time
import types
from types import SimpleNamespace

import pytest

from edge_core.services.ros import simple_video_bridge as svb
from edge_core.services.ros.simple_video_bridge import (
    BridgeHTTPHandler,
    BridgeHTTPServer,
    VideoBridge,
)


class _FakePopen:
    """Stand-in for ``subprocess.Popen`` that never launches anything."""

    def __init__(self, *args, poll_seq=None, **kwargs):
        self._poll_seq = list(poll_seq) if poll_seq is not None else [None]
        self.terminated = False
        self.killed = False

    def poll(self):
        return self._poll_seq.pop(0) if len(self._poll_seq) > 1 else self._poll_seq[0]

    def wait(self, timeout=None):
        return 0

    def terminate(self):
        self.terminated = True

    def kill(self):
        self.killed = True


class _NoThread:
    """Swallows the monitor thread so ``start()`` stays synchronous."""

    def __init__(self, *args, **kwargs):
        pass

    def start(self):
        pass

    def is_alive(self):
        return False


def _bridge(**overrides):
    kwargs = dict(source_topic="/zed/img", width=640, height=360, fps=15, bitrate=800)
    kwargs.update(overrides)
    return VideoBridge(**kwargs)


# -- VideoBridge: getters / status -----------------------------------------


def test_getters_and_status_defaults():
    b = _bridge()
    assert b.source_topic == "/zed/img"
    assert b.running is False

    status = b.get_status()
    assert status["streaming"] is False
    assert status["source_topic"] == "/zed/img"
    assert status["last_frame_age_s"] == -1  # no frame seen yet
    assert status["width"] == 640

    assert b.get_health() == {
        "healthy": False,
        "pipeline_playing": False,
        "source_topic": "/zed/img",
    }

    b.set_overlay(True)
    assert b.get_overlay_status() == {"enabled": True, "detection_count": 0}

    b.set_center_depth(3.5)
    assert b.get_center_depth() == {"range_m": 3.5}


def test_status_reports_positive_frame_age():
    b = _bridge()
    b._last_frame_time = time.time() - 1
    assert b.get_status()["last_frame_age_s"] > 0


# -- VideoBridge: lifecycle -------------------------------------------------


def test_start_then_stop(monkeypatch):
    monkeypatch.setattr(svb.subprocess, "Popen", _FakePopen)
    monkeypatch.setattr(svb.threading, "Thread", _NoThread)
    b = _bridge()

    assert b.start() is True
    assert b.running is True
    # Idempotent: a second start with the pipeline already up is a no-op success.
    assert b.start() is True

    b.stop()
    assert b.running is False


def test_switch_topic_same_is_noop(monkeypatch):
    monkeypatch.setattr(svb.subprocess, "Popen", _FakePopen)
    monkeypatch.setattr(svb.threading, "Thread", _NoThread)
    b = _bridge()
    assert b.switch_topic("/zed/img") is True  # identical topic -> short-circuit
    assert b.running is False  # never started a pipeline


def test_switch_topic_change_restarts(monkeypatch):
    monkeypatch.setattr(svb.subprocess, "Popen", _FakePopen)
    monkeypatch.setattr(svb.threading, "Thread", _NoThread)
    b = _bridge()
    b.start()
    assert b.switch_topic("/zed/depth") is True
    assert b.source_topic == "/zed/depth"
    assert b.running is True


def test_restart(monkeypatch):
    monkeypatch.setattr(svb.subprocess, "Popen", _FakePopen)
    monkeypatch.setattr(svb.threading, "Thread", _NoThread)
    b = _bridge()
    b.start()
    assert b.restart() is True
    assert b.running is True


# -- VideoBridge: pipeline start failure modes ------------------------------


def test_start_pipeline_generic_error_counts(monkeypatch):
    def boom(*args, **kwargs):
        raise RuntimeError("spawn failed")

    monkeypatch.setattr(svb.subprocess, "Popen", boom)
    b = _bridge()
    assert b.start() is False
    assert b._error_count == 1


def test_start_pipeline_missing_binary_falls_back_to_appsrc(monkeypatch):
    def missing(*args, **kwargs):
        raise FileNotFoundError

    monkeypatch.setattr(svb.subprocess, "Popen", missing)
    b = _bridge()
    monkeypatch.setattr(b, "_start_appsrc_pipeline", lambda: "fallback-called")
    assert b.start() == "fallback-called"


def test_appsrc_pipeline_without_gstreamer_bindings(monkeypatch):
    # Force `import gi` to raise ImportError.
    monkeypatch.setitem(sys.modules, "gi", None)
    b = _bridge()
    assert b._start_appsrc_pipeline() is False
    assert b._error_count == 1


def test_appsrc_pipeline_with_gstreamer_bindings(monkeypatch):
    # Inject a fake `gi` so the appsrc path proceeds to the python pipeline.
    fake_gi = types.ModuleType("gi")
    fake_gi.require_version = lambda *a, **k: None
    fake_repo = types.ModuleType("gi.repository")
    fake_repo.Gst = SimpleNamespace(init=lambda *a: None)
    monkeypatch.setitem(sys.modules, "gi", fake_gi)
    monkeypatch.setitem(sys.modules, "gi.repository", fake_repo)
    b = _bridge()
    monkeypatch.setattr(b, "_start_python_subprocess_pipeline", lambda: "py-pipeline")
    assert b._start_appsrc_pipeline() == "py-pipeline"


def test_python_subprocess_pipeline_success(monkeypatch):
    monkeypatch.setattr(svb.subprocess, "Popen", _FakePopen)
    b = _bridge()
    assert b._start_python_subprocess_pipeline() is True
    assert b.running is True


def test_python_subprocess_pipeline_failure(monkeypatch):
    monkeypatch.setattr(svb.subprocess, "Popen", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom")))
    b = _bridge()
    assert b._start_python_subprocess_pipeline() is False


# -- VideoBridge: stop / monitor -------------------------------------------


def test_stop_pipeline_kills_when_terminate_hangs():
    b = _bridge()
    fp = _FakePopen()

    def bad_wait(timeout=None):
        raise RuntimeError("hung")

    fp.wait = bad_wait
    b._pipeline = fp
    b._running = True

    b._stop_pipeline()
    assert fp.killed is True
    assert b._pipeline is None
    assert b._running is False


def test_stop_pipeline_swallows_kill_failure():
    b = _bridge()
    fp = _FakePopen()
    fp.wait = lambda timeout=None: (_ for _ in ()).throw(RuntimeError("hung"))
    fp.kill = lambda: (_ for _ in ()).throw(RuntimeError("kill also failed"))
    b._pipeline = fp
    b._running = True
    b._stop_pipeline()  # both terminate-wait and kill raise -> all swallowed
    assert b._pipeline is None
    assert b._running is False


def test_monitor_pipeline_returns_without_pipeline():
    b = _bridge()
    b._pipeline = None
    b._monitor_pipeline()  # must not raise


def test_monitor_pipeline_counts_frame_then_detects_exit():
    b = _bridge()
    # First poll: still running (counts a frame); second poll: exited.
    b._pipeline = _FakePopen(poll_seq=[None, 5])
    b._running = True
    b._start_time = time.time() - 1  # ensure elapsed > 0 so fps is computed
    b._monitor_pipeline()
    assert b._frame_count == 1
    assert b._fps_value > 0
    assert b._running is False
    assert b._error_count == 1


# -- HTTP handler routing ---------------------------------------------------


def _handler(bridge):
    """Build a handler without going through the socket-bound base __init__."""
    h = BridgeHTTPHandler.__new__(BridgeHTTPHandler)
    h.server = SimpleNamespace(bridge=bridge)
    h.captured = []
    h._json_response = lambda data, status=200: h.captured.append((status, data))
    return h


@pytest.mark.parametrize(
    "path,key",
    [
        ("/health", "pipeline_playing"),
        ("/status", "streaming"),
        ("/overlay/status", "enabled"),
        ("/depth/center", "range_m"),
    ],
)
def test_do_get_routes_to_bridge(path, key):
    h = _handler(_bridge())
    h.path = path
    h.do_GET()
    status, data = h.captured[0]
    assert status == 200
    assert key in data


def test_do_get_topics(monkeypatch):
    h = _handler(_bridge())
    h._list_topics = lambda: {"topics": ["/zed/img"]}
    h.path = "/topics"
    h.do_GET()
    assert h.captured[0][1] == {"topics": ["/zed/img"]}


def test_do_get_unknown_path_404():
    h = _handler(_bridge())
    h.path = "/nope"
    h.do_GET()
    assert h.captured[0][0] == 404


def test_do_post_switch_success():
    bridge = SimpleNamespace(switch_topic=lambda topic: True)
    h = _handler(bridge)
    h.path = "/switch?topic=/zed/depth"
    h.do_POST()
    status, data = h.captured[0]
    assert status == 200
    assert data["success"] is True
    assert data["topic"] == "/zed/depth"


def test_do_post_switch_missing_topic_400():
    h = _handler(SimpleNamespace(switch_topic=lambda topic: True))
    h.path = "/switch"
    h.do_POST()
    status, data = h.captured[0]
    assert status == 400
    assert data["success"] is False


def test_do_post_restart():
    h = _handler(SimpleNamespace(restart=lambda: True))
    h.path = "/restart"
    h.do_POST()
    assert h.captured[0][1] == {"success": True}


def test_do_post_overlay_enable_and_disable():
    b = _bridge()
    h = _handler(b)
    h.path = "/overlay/enable"
    h.do_POST()
    assert b._overlay_enabled is True

    h.path = "/overlay/disable"
    h.do_POST()
    assert b._overlay_enabled is False


def test_do_post_unknown_path_404():
    h = _handler(_bridge())
    h.path = "/nope"
    h.do_POST()
    assert h.captured[0][0] == 404


# -- HTTP handler helpers ---------------------------------------------------


def test_json_response_writes_status_and_body():
    h = BridgeHTTPHandler.__new__(BridgeHTTPHandler)
    sent = {"headers": []}
    h.send_response = lambda s: sent.__setitem__("status", s)
    h.send_header = lambda k, v: sent["headers"].append((k, v))
    h.end_headers = lambda: sent.__setitem__("ended", True)
    h.wfile = io.BytesIO()

    h._json_response({"ok": True}, status=201)
    assert sent["status"] == 201
    assert sent["ended"] is True
    assert ("Content-Type", "application/json") in sent["headers"]
    assert json.loads(h.wfile.getvalue()) == {"ok": True}


def test_list_topics_filters_image_topics(monkeypatch):
    stdout = "/zed/img sensor_msgs/msg/Image\n/chatter std_msgs/msg/String\n"
    monkeypatch.setattr(
        svb.subprocess,
        "run",
        lambda *a, **k: SimpleNamespace(stdout=stdout),
    )
    h = BridgeHTTPHandler.__new__(BridgeHTTPHandler)
    assert h._list_topics() == {"topics": ["/zed/img"]}


def test_list_topics_swallows_errors(monkeypatch):
    def boom(*a, **k):
        raise OSError("ros2 not found")

    monkeypatch.setattr(svb.subprocess, "run", boom)
    h = BridgeHTTPHandler.__new__(BridgeHTTPHandler)
    assert h._list_topics() == {"topics": []}


def test_log_message_is_quiet():
    h = BridgeHTTPHandler.__new__(BridgeHTTPHandler)
    h.log_message("%s", "GET /health")  # must not raise


# -- HTTP server / main entrypoint ------------------------------------------


def test_http_server_binds_and_exposes_bridge():
    bridge = _bridge()
    srv = BridgeHTTPServer("127.0.0.1", 0, bridge)  # port 0 -> ephemeral
    try:
        assert srv.bridge is bridge
    finally:
        srv.server_close()


def test_main_returns_when_pipeline_fails_to_start(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["prog", "--source-topic", "/zed/img"])
    monkeypatch.setattr(svb.VideoBridge, "start", lambda self: False)
    # No server is started on the failure path, so this returns promptly.
    svb.main()


def test_main_serves_then_shuts_down(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["prog", "--source-topic", "/zed/img", "--http-port", "0"])
    monkeypatch.setattr(svb.VideoBridge, "start", lambda self: True)
    # `running` flips False after the first loop iteration so main() exits.
    running_seq = iter([True, False])
    monkeypatch.setattr(svb.VideoBridge, "running", property(lambda self: next(running_seq, False)))
    seen = {"stopped": False, "shutdown": False, "served": False}
    monkeypatch.setattr(svb.VideoBridge, "stop", lambda self: seen.__setitem__("stopped", True))

    class _FakeServer:
        def __init__(self, host, port, bridge):
            self.bridge = bridge

        def serve_forever(self):
            seen["served"] = True

        def shutdown(self):
            seen["shutdown"] = True

    monkeypatch.setattr(svb, "BridgeHTTPServer", _FakeServer)
    monkeypatch.setattr(svb.threading, "Thread", _NoThread)
    monkeypatch.setattr(svb.time, "sleep", lambda *_: None)

    svb.main()
    assert seen["stopped"] is True
    assert seen["shutdown"] is True


def test_main_handles_keyboard_interrupt(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["prog", "--source-topic", "/zed/img", "--http-port", "0"])
    monkeypatch.setattr(svb.VideoBridge, "start", lambda self: True)
    monkeypatch.setattr(svb.VideoBridge, "running", property(lambda self: True))
    seen = {"stopped": False, "shutdown": False}
    monkeypatch.setattr(svb.VideoBridge, "stop", lambda self: seen.__setitem__("stopped", True))

    class _FakeServer:
        def __init__(self, host, port, bridge):
            self.bridge = bridge

        def serve_forever(self):
            pass

        def shutdown(self):
            seen["shutdown"] = True

    monkeypatch.setattr(svb, "BridgeHTTPServer", _FakeServer)
    monkeypatch.setattr(svb.threading, "Thread", _NoThread)

    def interrupt(*_):
        raise KeyboardInterrupt

    monkeypatch.setattr(svb.time, "sleep", interrupt)
    svb.main()  # Ctrl-C in the run loop is caught; finally still cleans up.
    assert seen["stopped"] is True
    assert seen["shutdown"] is True


# -- Pipeline content: single /stream path + low-latency settings -----------


class _RecordingPopen:
    """FakePopen that keeps the launched command so the pipeline can be asserted."""

    def __init__(self, cmd, *args, **kwargs):
        self.cmd = cmd
        self.terminated = False

    def poll(self):
        return None  # stays 'alive'

    def wait(self, timeout=None):
        return 0

    def terminate(self):
        self.terminated = True

    def kill(self):
        pass


def _record(monkeypatch):
    """Patch Popen with a recorder + neutralise the monitor thread."""
    spawned: list[_RecordingPopen] = []

    def fake(cmd, *args, **kwargs):
        p = _RecordingPopen(cmd)
        spawned.append(p)
        return p

    monkeypatch.setattr(svb.subprocess, "Popen", fake)
    monkeypatch.setattr(svb.threading, "Thread", _NoThread)
    return spawned


def test_default_rtsp_path_is_stream():
    # The Jetson serves exactly one stream — no /primary or /secondary.
    assert _bridge()._rtsp_path == "stream"


def test_pipeline_targets_single_stream_path(monkeypatch):
    spawned = _record(monkeypatch)
    b = _bridge(source_topic="/zed/zed_node/rgb/color/rect/image")
    assert b.start() is True
    cmd = " ".join(spawned[0].cmd)
    assert "rtspclientsink location=rtsp://localhost:8554/stream" in cmd
    assert "/primary" not in cmd and "/secondary" not in cmd
    assert "topic=/zed/zed_node/rgb/color/rect/image" in cmd


def test_pipeline_uses_low_latency_settings(monkeypatch):
    spawned = _record(monkeypatch)
    _bridge().start()
    cmd = " ".join(spawned[0].cmd)
    assert "tune=zerolatency" in cmd
    assert "speed-preset=ultrafast" in cmd
    assert "latency=0" in cmd


def test_pipeline_reflects_resolution_bitrate_and_fps(monkeypatch):
    spawned = _record(monkeypatch)
    _bridge(width=1280, height=720, fps=30, bitrate=2500).start()
    cmd = " ".join(spawned[0].cmd)
    assert "width=1280,height=720,framerate=30/1" in cmd
    assert "bitrate=2500" in cmd
    assert "key-int-max=60" in cmd  # fps * 2


def test_switch_rebuilds_pipeline_on_new_path(monkeypatch):
    spawned = _record(monkeypatch)
    b = _bridge(source_topic="/zed/a")
    b.start()
    assert b.switch_topic("/zed/b") is True
    assert len(spawned) == 2
    assert "topic=/zed/b" in " ".join(spawned[1].cmd)
    assert spawned[0].terminated is True  # old pipeline torn down


def test_rapid_switches_do_not_crash(monkeypatch):
    spawned = _record(monkeypatch)
    b = _bridge(source_topic="/zed/a")
    b.start()
    topics = ["/zed/a", "/zed/b", "/zed/c", "/zed/d"]
    for i in range(60):
        assert b.switch_topic(topics[i % len(topics)]) is True
    assert b.running is True
    assert b.source_topic == topics[59 % len(topics)]
    # Exactly one pipeline is live; every superseded one was terminated.
    assert sum(1 for p in spawned if not p.terminated) == 1
