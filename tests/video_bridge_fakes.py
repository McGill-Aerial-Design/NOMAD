# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Fakes shared by the video bridge tests.

Installs stand-in gi/Gst and rclpy modules so the bridge runs its start, push
and teardown paths synchronously and without ROS or GStreamer installed.
"""

from __future__ import annotations

import sys
import threading
import types
from types import SimpleNamespace

from python.tools.video_bridge import VideoBridge

__all__ = ["install_fake_runtime", "image_msg", "make_bridge"]


class NoThread:
    """Keep bridge start/monitor logic synchronous in unit tests."""

    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs

    def start(self):
        return None

    def is_alive(self):
        return False


class FakeBuffer:
    def __init__(self, size: int):
        self.data = bytearray(size)
        self.duration = 0

    def fill(self, offset: int, payload: bytes) -> None:
        self.data[offset : offset + len(payload)] = payload


class FakeBufferFactory:
    @staticmethod
    def new_allocate(_allocator, size: int, _params):
        return FakeBuffer(size)


class FakeAppSrc:
    def __init__(self, flow_return):
        self.flow_return = flow_return
        self.buffers: list[FakeBuffer] = []

    def emit(self, event: str, buffer: FakeBuffer):
        assert event == "push-buffer"
        self.buffers.append(buffer)
        return self.flow_return


class FakeMessage:
    def __init__(self, msg_type, error: str = "boom", debug: str = "debug"):
        self.type = msg_type
        self._error = error
        self._debug = debug

    def parse_error(self):
        return self._error, self._debug


class FakeBus:
    def __init__(self, messages=None):
        self.messages = list(messages or [])

    def timed_pop_filtered(self, *_args):
        return self.messages.pop(0) if self.messages else None


class FakePipeline:
    def __init__(self, appsrc: FakeAppSrc, bus: FakeBus):
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


class FakeNode:
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


class FakeExecutor:
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


def make_fake_gst_class(flow_return, bus_messages):
    """Build a Gst stand-in whose parse_launch records every pipeline string."""

    class FakeGst:
        SECOND = 1_000_000_000
        Buffer = FakeBufferFactory
        FlowReturn = SimpleNamespace(OK="ok")
        State = SimpleNamespace(PLAYING="playing", NULL="null")
        MessageType = SimpleNamespace(ERROR=1, EOS=2)
        launches: list[str] = []
        pipeline: FakePipeline | None = None

        @classmethod
        def init(cls, *_args):
            return None

        @classmethod
        def parse_launch(cls, command: str):
            cls.launches.append(command)
            cls.pipeline = FakePipeline(FakeAppSrc(flow_return), FakeBus(bus_messages))
            return cls.pipeline

    return FakeGst


def install_fake_gst(monkeypatch, fake_gst) -> None:
    fake_gi = types.ModuleType("gi")
    fake_gi.require_version = lambda *args, **kwargs: None
    fake_repository = types.ModuleType("gi.repository")
    fake_repository.Gst = fake_gst
    monkeypatch.setitem(sys.modules, "gi", fake_gi)
    monkeypatch.setitem(sys.modules, "gi.repository", fake_repository)


def fake_rclpy_attributes(state) -> dict:
    """rclpy functions that record init, node and shutdown calls in ``state``."""

    def create_node(_name):
        state["node"] = FakeNode()
        return state["node"]

    def shutdown():
        state["ok"] = False
        state["shutdown"] = True

    return {
        "ok": lambda: state["ok"],
        "init": lambda args=None: state.__setitem__("ok", True),
        "spin": lambda _node: None,
        "create_node": create_node,
        "shutdown": shutdown,
    }


def install_fake_ros_module(monkeypatch, name: str, **attributes) -> None:
    module = types.ModuleType(name)
    for attribute, value in attributes.items():
        setattr(module, attribute, value)
    monkeypatch.setitem(sys.modules, name, module)


def install_fake_rclpy(monkeypatch, state) -> None:
    install_fake_ros_module(monkeypatch, "rclpy", **fake_rclpy_attributes(state))
    install_fake_ros_module(monkeypatch, "rclpy.executors", SingleThreadedExecutor=FakeExecutor)
    install_fake_ros_module(
        monkeypatch,
        "rclpy.qos",
        HistoryPolicy=SimpleNamespace(KEEP_LAST="keep_last"),
        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT="best_effort"),
        QoSProfile=lambda **kwargs: kwargs,
    )
    install_fake_ros_module(monkeypatch, "sensor_msgs")
    install_fake_ros_module(monkeypatch, "sensor_msgs.msg", Image=type("Image", (), {}))


def install_fake_runtime(monkeypatch, *, flow_return="ok", bus_messages=None):
    """Install fake gi/Gst/rclpy modules and return (FakeGst, rclpy_state)."""
    fake_gst = make_fake_gst_class(flow_return, bus_messages)
    install_fake_gst(monkeypatch, fake_gst)

    state = {"ok": False, "node": None, "shutdown": False}
    install_fake_rclpy(monkeypatch, state)
    monkeypatch.setattr(threading, "Thread", NoThread)
    return fake_gst, state


def make_bridge(**overrides) -> VideoBridge:
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


def image_msg(*, encoding: str, width: int, height: int, step: int, data: bytes):
    return SimpleNamespace(encoding=encoding, width=width, height=height, step=step, data=data)
