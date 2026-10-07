# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for the video bridge GStreamer pipeline, statistics and teardown."""

from __future__ import annotations

import sys
import time

import pytest
from video_bridge_fakes import (
    FakeAppSrc,
    FakeBus,
    FakeMessage,
    FakePipeline,
    image_msg,
    install_fake_runtime,
    make_bridge,
)


def test_getters_and_status_defaults():
    bridge = make_bridge()
    assert bridge.source_topic == "/zed/img"
    assert bridge.running is False
    assert bridge.get_status()["last_frame_age_s"] == -1
    assert bridge.get_status()["width"] == 640
    assert bridge.get_health() == {"healthy": False, "pipeline_playing": False, "source_topic": "/zed/img"}

    bridge.set_overlay(True)
    assert bridge.get_overlay_status() == {"enabled": True, "detection_count": 0}


def test_status_reports_positive_frame_age():
    bridge = make_bridge()
    bridge._last_frame_time = time.time() - 1
    assert bridge.get_status()["last_frame_age_s"] > 0


def test_start_stop_and_pipeline_content(monkeypatch):
    fake_gst, state = install_fake_runtime(monkeypatch)
    bridge = make_bridge(source_topic="/zed/zed_node/rgb/color/rect/image", flip_method="rotate-180")

    assert bridge.start() is True
    assert bridge.start() is True
    assert bridge.running is True
    assert len(fake_gst.launches) == 1
    command = fake_gst.launches[0]
    assert "appsrc name=ros_source" in command
    assert "videoflip method=rotate-180" in command
    assert "x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast" in command
    assert "rtspclientsink location=rtsp://localhost:8554/stream" in command
    assert state["node"].subscriptions[0].topic == "/zed/zed_node/rgb/color/rect/image"

    pipeline = fake_gst.pipeline
    bridge.stop()
    assert bridge.running is False
    assert pipeline.states[-1] == "null"
    assert state["shutdown"] is True


def test_switch_topic_restarts_pipeline(monkeypatch):
    fake_gst, state = install_fake_runtime(monkeypatch)
    bridge = make_bridge()
    bridge.start()
    assert bridge.switch_topic("/zed/depth") is True
    assert bridge.source_topic == "/zed/depth"
    assert len(fake_gst.launches) == 2
    assert state["node"].subscriptions[0].topic == "/zed/depth"


def test_switch_topic_same_is_noop(monkeypatch):
    fake_gst, _state = install_fake_runtime(monkeypatch)
    bridge = make_bridge()
    assert bridge.switch_topic("/zed/img") is True
    assert fake_gst.launches == []


def test_restart(monkeypatch):
    fake_gst, _state = install_fake_runtime(monkeypatch)
    bridge = make_bridge()
    bridge.start()
    assert bridge.restart() is True
    assert len(fake_gst.launches) == 2


def test_start_resets_frame_statistics(monkeypatch):
    install_fake_runtime(monkeypatch)
    bridge = make_bridge()
    bridge._frame_count = 12
    bridge._dropped_count = 3
    bridge._fps_value = 9.5
    assert bridge.start() is True
    assert bridge._frame_count == 0
    assert bridge._dropped_count == 0
    assert bridge._fps_value == 0.0


def test_start_pipeline_without_gstreamer_bindings(monkeypatch):
    monkeypatch.setitem(sys.modules, "gi", None)
    bridge = make_bridge()
    assert bridge.start() is False
    assert bridge._error_count == 1


def test_start_pipeline_cleans_up_when_ros_subscription_fails(monkeypatch):
    install_fake_runtime(monkeypatch)
    monkeypatch.setitem(sys.modules, "rclpy", None)
    bridge = make_bridge()
    assert bridge.start() is False
    assert bridge.running is False
    assert bridge._error_count == 1


def test_on_image_pushes_rgb_buffer(monkeypatch):
    fake_gst, _state = install_fake_runtime(monkeypatch)
    bridge = make_bridge(width=2, height=1)
    bridge._gst = fake_gst
    bridge._appsrc = FakeAppSrc(fake_gst.FlowReturn.OK)
    bridge._running = True
    bridge._start_time = time.time() - 1

    bridge._on_image(image_msg(encoding="bgr8", width=2, height=1, step=6, data=bytes([1, 2, 3, 4, 5, 6])))

    assert bridge._frame_count == 1
    assert bytes(bridge._appsrc.buffers[0].data) == bytes([3, 2, 1, 6, 5, 4])
    assert bridge._fps_value > 0


def test_on_image_counts_dropped_buffers(monkeypatch):
    fake_gst, _state = install_fake_runtime(monkeypatch, flow_return="not-ok")
    bridge = make_bridge(width=1, height=1)
    bridge._gst = fake_gst
    bridge._appsrc = FakeAppSrc("not-ok")
    bridge._running = True
    bridge._on_image(image_msg(encoding="rgb8", width=1, height=1, step=3, data=bytes([1, 2, 3])))
    assert bridge._dropped_count == 1
    assert bridge._frame_count == 0


def test_on_image_counts_conversion_errors(monkeypatch):
    fake_gst, _state = install_fake_runtime(monkeypatch)
    bridge = make_bridge(width=1, height=1)
    bridge._gst = fake_gst
    bridge._appsrc = FakeAppSrc(fake_gst.FlowReturn.OK)
    bridge._running = True

    bridge._on_image(image_msg(encoding="yuyv", width=1, height=1, step=2, data=b"\0\0"))

    assert bridge._error_count == 1
    assert bridge._frame_count == 0


def test_on_image_ignores_frames_when_stopped(monkeypatch):
    fake_gst, _state = install_fake_runtime(monkeypatch)
    bridge = make_bridge(width=1, height=1)
    bridge._gst = fake_gst
    bridge._appsrc = FakeAppSrc(fake_gst.FlowReturn.OK)
    bridge._running = False

    bridge._on_image(image_msg(encoding="rgb8", width=1, height=1, step=3, data=bytes([1, 2, 3])))

    assert bridge._appsrc.buffers == []
    assert bridge._frame_count == 0


def test_monitor_pipeline_handles_error_and_eos(monkeypatch):
    fake_gst, _state = install_fake_runtime(
        monkeypatch,
        bus_messages=[FakeMessage(1), FakeMessage(2)],
    )
    bridge = make_bridge()
    bridge._gst = fake_gst
    bridge._pipeline = fake_gst.parse_launch("pipeline")
    bridge._running = True
    bridge._monitor_pipeline()
    assert bridge.running is False
    assert bridge._error_count == 1

    bridge._running = True
    bridge._pipeline = FakePipeline(FakeAppSrc("ok"), FakeBus([FakeMessage(fake_gst.MessageType.EOS)]))
    bridge._monitor_pipeline()
    assert bridge.running is False


def test_monitor_pipeline_returns_without_pipeline():
    make_bridge()._monitor_pipeline()


@pytest.mark.parametrize("fps", [0, 15])
def test_pipeline_command_uses_requested_frame_rate(monkeypatch, fps):
    fake_gst, _state = install_fake_runtime(monkeypatch)
    bridge = make_bridge(fps=fps, width=320, height=240)
    assert bridge.start() is True
    command = fake_gst.launches[0]
    assert f"video/x-raw,width=320,height=240,framerate={fps}/1" in command
    assert f"key-int-max={fps * 2}" in command
