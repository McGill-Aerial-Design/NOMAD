# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for edge_core.services.ros.simple_video_bridge."""

from __future__ import annotations

import io
import json
import sys
import time
import types
from types import SimpleNamespace

import pytest

from edge_core.services.ros import simple_video_bridge as svb
from edge_core.services.ros.simple_video_bridge import BridgeHTTPHandler, BridgeHTTPServer, VideoBridge


class _NoThread:
    """Keep bridge start/monitor logic synchronous in unit tests."""

    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs

    def start(self):
        return None

    def is_alive(self):
        return False


class _FakeBuffer:
    def __init__(self, size: int):
        self.data = bytearray(size)
        self.duration = 0

    def fill(self, offset: int, payload: bytes) -> None:
        self.data[offset : offset + len(payload)] = payload


class _FakeBufferFactory:
    @staticmethod
    def new_allocate(_allocator, size: int, _params):
        return _FakeBuffer(size)


class _FakeAppSrc:
    def __init__(self, flow_return):
        self.flow_return = flow_return
        self.buffers: list[_FakeBuffer] = []

    def emit(self, event: str, buffer: _FakeBuffer):
        assert event == "push-buffer"
        self.buffers.append(buffer)
        return self.flow_return


class _FakeMessage:
    def __init__(self, msg_type, error: str = "boom", debug: str = "debug"):
        self.type = msg_type
        self._error = error
        self._debug = debug

    def parse_error(self):
        return self._error, self._debug


class _FakeBus:
    def __init__(self, messages=None):
        self.messages = list(messages or [])

    def timed_pop_filtered(self, *_args):
        return self.messages.pop(0) if self.messages else None


class _FakePipeline:
    def __init__(self, appsrc: _FakeAppSrc, bus: _FakeBus):
        self.appsrc = appsrc
        self.bus = bus
        self.states = []

    def get_by_name(self, name: str):
        assert name == "ros_source"
        return self.appsrc

    def set_state(self, state):
        self.states.append(state)

    def get_bus(self):
        return self.bus


class _FakeNode:
    def __init__(self):
        self.subscriptions = []
        self.destroyed = False

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = SimpleNamespace(msg_type=msg_type, topic=topic, callback=callback, qos=qos)
        self.subscriptions.append(sub)
        return sub

    def destroy_subscription(self, subscription):
        self.subscriptions.remove(subscription)

    def destroy_node(self):
        self.destroyed = True


class _FakeExecutor:
    """Minimal SingleThreadedExecutor stand-in for the bridge spin loop."""

    def __init__(self):
        self.nodes = []
        self.shutdown_called = False

    def add_node(self, node):
        self.nodes.append(node)

    def remove_node(self, node):
        if node in self.nodes:
            self.nodes.remove(node)

    def spin(self):
        return None

    def shutdown(self):
        self.shutdown_called = True


def _install_fake_runtime(monkeypatch, *, flow_return="ok", bus_messages=None):
    """Install fake gi/Gst/rclpy modules and return (FakeGst, rclpy_state)."""

    class FakeGst:
        SECOND = 1_000_000_000
        Buffer = _FakeBufferFactory
        FlowReturn = SimpleNamespace(OK="ok")
        State = SimpleNamespace(PLAYING="playing", NULL="null")
        MessageType = SimpleNamespace(ERROR=1, EOS=2)
        launches: list[str] = []
        pipeline: _FakePipeline | None = None

        @classmethod
        def init(cls, *_args):
            return None

        @classmethod
        def parse_launch(cls, command: str):
            cls.launches.append(command)
            cls.pipeline = _FakePipeline(_FakeAppSrc(flow_return), _FakeBus(bus_messages))
            return cls.pipeline

    fake_gi = types.ModuleType("gi")
    fake_gi.require_version = lambda *a, **k: None
    fake_repo = types.ModuleType("gi.repository")
    fake_repo.Gst = FakeGst
    monkeypatch.setitem(sys.modules, "gi", fake_gi)
    monkeypatch.setitem(sys.modules, "gi.repository", fake_repo)

    state = {"ok": False, "node": None, "shutdown": False}
    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.ok = lambda: state["ok"]

    def init(args=None):
        state["ok"] = True

    def create_node(_name):
        state["node"] = _FakeNode()
        return state["node"]

    def shutdown():
        state["ok"] = False
        state["shutdown"] = True

    fake_rclpy.init = init
    fake_rclpy.create_node = create_node
    fake_rclpy.spin = lambda _node: None
    fake_rclpy.shutdown = shutdown

    fake_executors = types.ModuleType("rclpy.executors")
    fake_executors.SingleThreadedExecutor = _FakeExecutor
    monkeypatch.setitem(sys.modules, "rclpy.executors", fake_executors)

    fake_qos = types.ModuleType("rclpy.qos")
    fake_qos.HistoryPolicy = SimpleNamespace(KEEP_LAST="keep_last")
    fake_qos.ReliabilityPolicy = SimpleNamespace(BEST_EFFORT="best_effort")
    fake_qos.QoSProfile = lambda **kwargs: kwargs

    fake_sensor_msgs = types.ModuleType("sensor_msgs")
    fake_sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    fake_sensor_msgs_msg.Image = type("Image", (), {})
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.qos", fake_qos)
    monkeypatch.setitem(sys.modules, "sensor_msgs", fake_sensor_msgs)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", fake_sensor_msgs_msg)
    monkeypatch.setattr(svb.threading, "Thread", _NoThread)
    return FakeGst, state


def _bridge(**overrides):
    kwargs = dict(
        source_topic="/zed/img",
        width=640,
        height=360,
        fps=15,
        bitrate=800,
        rtsp_url="rtsp://localhost:8554/stream",
    )
    kwargs.update(overrides)
    return VideoBridge(**kwargs)


def _image_msg(*, encoding: str, width: int, height: int, step: int, data: bytes):
    return SimpleNamespace(encoding=encoding, width=width, height=height, step=step, data=data)


def _handler(bridge):
    h = BridgeHTTPHandler.__new__(BridgeHTTPHandler)
    h.server = SimpleNamespace(bridge=bridge)
    h.captured = []
    h._json_response = lambda data, status=200: h.captured.append((status, data))
    return h


def test_getters_and_status_defaults():
    b = _bridge()
    assert b.source_topic == "/zed/img"
    assert b.running is False
    assert b.get_status()["last_frame_age_s"] == -1
    assert b.get_status()["width"] == 640
    assert b.get_health() == {"healthy": False, "pipeline_playing": False, "source_topic": "/zed/img"}

    b.set_overlay(True)
    b.set_center_depth(3.5)
    assert b.get_overlay_status() == {"enabled": True, "detection_count": 0}
    assert b.get_center_depth() == {"range_m": 3.5}


def test_status_reports_positive_frame_age():
    b = _bridge()
    b._last_frame_time = time.time() - 1
    assert b.get_status()["last_frame_age_s"] > 0


def test_start_stop_and_pipeline_content(monkeypatch):
    fake_gst, state = _install_fake_runtime(monkeypatch)
    b = _bridge(source_topic="/zed/zed_node/rgb/color/rect/image", flip_method="rotate-180")

    assert b.start() is True
    assert b.start() is True
    assert b.running is True
    assert len(fake_gst.launches) == 1
    command = fake_gst.launches[0]
    assert "appsrc name=ros_source" in command
    assert "videoflip method=rotate-180" in command
    assert "x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast" in command
    assert "rtspclientsink location=rtsp://localhost:8554/stream" in command
    assert state["node"].subscriptions[0].topic == "/zed/zed_node/rgb/color/rect/image"

    pipeline = fake_gst.pipeline
    b.stop()
    assert b.running is False
    assert pipeline.states[-1] == "null"
    assert state["shutdown"] is True


def test_switch_topic_restarts_pipeline(monkeypatch):
    fake_gst, state = _install_fake_runtime(monkeypatch)
    b = _bridge()
    b.start()
    assert b.switch_topic("/zed/depth") is True
    assert b.source_topic == "/zed/depth"
    assert len(fake_gst.launches) == 2
    assert state["node"].subscriptions[0].topic == "/zed/depth"


def test_switch_topic_same_is_noop(monkeypatch):
    fake_gst, _state = _install_fake_runtime(monkeypatch)
    b = _bridge()
    assert b.switch_topic("/zed/img") is True
    assert fake_gst.launches == []


def test_restart(monkeypatch):
    fake_gst, _state = _install_fake_runtime(monkeypatch)
    b = _bridge()
    b.start()
    assert b.restart() is True
    assert len(fake_gst.launches) == 2


def test_start_resets_frame_statistics(monkeypatch):
    _install_fake_runtime(monkeypatch)
    b = _bridge()
    b._frame_count = 12
    b._dropped_count = 3
    b._fps_value = 9.5
    assert b.start() is True
    assert b._frame_count == 0
    assert b._dropped_count == 0
    assert b._fps_value == 0.0


def test_start_pipeline_without_gstreamer_bindings(monkeypatch):
    monkeypatch.setitem(sys.modules, "gi", None)
    b = _bridge()
    assert b.start() is False
    assert b._error_count == 1


def test_start_pipeline_cleans_up_when_ros_subscription_fails(monkeypatch):
    _install_fake_runtime(monkeypatch)
    monkeypatch.setitem(sys.modules, "rclpy", None)
    b = _bridge()
    assert b.start() is False
    assert b.running is False
    assert b._error_count == 1


def test_on_image_pushes_rgb_buffer(monkeypatch):
    fake_gst, _state = _install_fake_runtime(monkeypatch)
    b = _bridge(width=2, height=1)
    b._gst = fake_gst
    b._appsrc = _FakeAppSrc(fake_gst.FlowReturn.OK)
    b._running = True
    b._start_time = time.time() - 1

    msg = _image_msg(encoding="bgr8", width=2, height=1, step=6, data=bytes([1, 2, 3, 4, 5, 6]))
    b._on_image(msg)

    assert b._frame_count == 1
    assert bytes(b._appsrc.buffers[0].data) == bytes([3, 2, 1, 6, 5, 4])
    assert b._fps_value > 0


@pytest.mark.parametrize(
    ("encoding", "data", "expected"),
    [
        ("rgb8", bytes([1, 2, 3]), bytes([1, 2, 3])),
        ("rgba8", bytes([1, 2, 3, 4]), bytes([1, 2, 3])),
        ("bgra8", bytes([1, 2, 3, 4]), bytes([3, 2, 1])),
        ("mono8", bytes([7]), bytes([7, 7, 7])),
    ],
)
def test_image_to_rgb_encodings(encoding, data, expected):
    frame = _bridge(width=1, height=1)._image_to_rgb(
        _image_msg(encoding=encoding, width=1, height=1, step=len(data), data=data)
    )
    assert frame.tobytes() == expected


def test_image_to_rgb_resizes_with_nearest_neighbor():
    msg = _image_msg(
        encoding="rgb8",
        width=2,
        height=2,
        step=6,
        data=bytes([1, 0, 0, 2, 0, 0, 3, 0, 0, 4, 0, 0]),
    )
    frame = _bridge(width=1, height=1)._image_to_rgb(msg)
    assert frame.shape == (1, 1, 3)


def test_image_to_rgb_rejects_unknown_encoding():
    b = _bridge()
    assert b._image_to_rgb(_image_msg(encoding="yuyv", width=1, height=1, step=2, data=b"\0\0")) is None
    assert b._error_count == 1


def test_on_image_counts_dropped_buffers(monkeypatch):
    fake_gst, _state = _install_fake_runtime(monkeypatch, flow_return="not-ok")
    b = _bridge(width=1, height=1)
    b._gst = fake_gst
    b._appsrc = _FakeAppSrc("not-ok")
    b._running = True
    msg = _image_msg(encoding="rgb8", width=1, height=1, step=3, data=bytes([1, 2, 3]))
    b._on_image(msg)
    assert b._dropped_count == 1


def test_monitor_pipeline_handles_error_and_eos(monkeypatch):
    fake_gst, _state = _install_fake_runtime(
        monkeypatch,
        bus_messages=[_FakeMessage(1), _FakeMessage(2)],
    )
    b = _bridge()
    b._gst = fake_gst
    b._pipeline = fake_gst.parse_launch("pipeline")
    b._running = True
    b._monitor_pipeline()
    assert b.running is False
    assert b._error_count == 1

    b._running = True
    b._pipeline = _FakePipeline(_FakeAppSrc("ok"), _FakeBus([_FakeMessage(fake_gst.MessageType.EOS)]))
    b._monitor_pipeline()
    assert b.running is False


def test_monitor_pipeline_returns_without_pipeline():
    _bridge()._monitor_pipeline()


@pytest.mark.parametrize(
    ("path", "key"),
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
    h = _handler(SimpleNamespace(switch_topic=lambda topic: True))
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
    monkeypatch.setattr(svb.subprocess, "run", lambda *a, **k: SimpleNamespace(stdout=stdout))
    h = BridgeHTTPHandler.__new__(BridgeHTTPHandler)
    assert h._list_topics() == {"topics": ["/zed/img"]}


def test_list_topics_swallows_errors(monkeypatch):
    monkeypatch.setattr(svb.subprocess, "run", lambda *a, **k: (_ for _ in ()).throw(OSError("ros2 missing")))
    h = BridgeHTTPHandler.__new__(BridgeHTTPHandler)
    assert h._list_topics() == {"topics": []}


def test_log_message_is_quiet():
    h = BridgeHTTPHandler.__new__(BridgeHTTPHandler)
    h.log_message("%s", "GET /health")


def test_http_server_binds_and_exposes_bridge():
    bridge = _bridge()
    srv = BridgeHTTPServer("127.0.0.1", 0, bridge)
    try:
        assert srv.bridge is bridge
    finally:
        srv.server_close()


def test_main_returns_when_pipeline_fails_to_start(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["prog", "--source-topic", "/zed/img"])
    monkeypatch.setattr(svb.VideoBridge, "start", lambda self: False)
    svb.main()


def test_main_uses_rtsp_url_argument(monkeypatch):
    seen = {}

    class _FakeBridge:
        def __init__(self, **kwargs):
            seen.update(kwargs)
            self.running = False

        def start(self):
            return False

    monkeypatch.setattr(sys, "argv", ["prog", "--source-topic", "/zed/img", "--rtsp-url", "rtsp://m/stream"])
    monkeypatch.setattr(svb, "VideoBridge", _FakeBridge)
    svb.main()
    assert seen["rtsp_url"] == "rtsp://m/stream"


def test_main_uses_flip_method_argument(monkeypatch):
    seen = {}

    class _FakeBridge:
        def __init__(self, **kwargs):
            seen.update(kwargs)
            self.running = False

        def start(self):
            return False

    monkeypatch.setattr(sys, "argv", ["prog", "--source-topic", "/zed/img", "--flip-method", "rotate-180"])
    monkeypatch.setattr(svb, "VideoBridge", _FakeBridge)
    svb.main()
    assert seen["flip_method"] == "rotate-180"


def test_main_uses_rtsp_url_env(monkeypatch):
    seen = {}

    class _FakeBridge:
        def __init__(self, **kwargs):
            seen.update(kwargs)
            self.running = False

        def start(self):
            return False

    monkeypatch.setenv("NOMAD_VIDEO_RTSP_PUBLISH_URL", "rtsp://mediamtx:8554/stream")
    monkeypatch.setattr(sys, "argv", ["prog", "--source-topic", "/zed/img"])
    monkeypatch.setattr(svb, "VideoBridge", _FakeBridge)
    svb.main()
    assert seen["rtsp_url"] == "rtsp://mediamtx:8554/stream"


def test_main_serves_then_shuts_down(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["prog", "--source-topic", "/zed/img", "--http-port", "0"])
    monkeypatch.setattr(svb.VideoBridge, "start", lambda self: True)
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
            return None

        def shutdown(self):
            seen["shutdown"] = True

    monkeypatch.setattr(svb, "BridgeHTTPServer", _FakeServer)
    monkeypatch.setattr(svb.threading, "Thread", _NoThread)
    monkeypatch.setattr(svb.time, "sleep", lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    svb.main()
    assert seen["stopped"] is True
    assert seen["shutdown"] is True
