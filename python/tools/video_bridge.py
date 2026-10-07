# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""GStreamer pipeline that republishes a ROS image topic as an RTSP stream.

  ROS2 Image Topic -> GStreamer x264enc -> RTSP -> MediaMTX -> Viewers

The class also holds the stream statistics and overlay state that the local HTTP
API reports; topic switching and restart are its only state changes.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

from python.tools.video_frame import FrameConversionError, image_to_rgb
from python.tools.video_ros_source import RosImageSource

__all__ = ["VideoBridge"]

logger = logging.getLogger("nomad.video_bridge.pipeline")

_PIPELINE_NAME = "ros_source"
_GST_PARSE_TIMEOUT_NS = 500_000_000


class VideoBridge:
    """Manages the GStreamer pipeline from ROS2 topic to RTSP."""

    def __init__(
        self,
        source_topic: str,
        width: int,
        height: int,
        fps: int,
        bitrate: int,
        rtsp_url: str,
        flip_method: str = "identity",
        rtsp_path: str = "stream",
    ):
        self._source_topic = source_topic
        self._width = width
        self._height = height
        self._fps = fps
        self._bitrate = bitrate
        self._rtsp_url = rtsp_url
        self._flip_method = flip_method
        self._rtsp_path = rtsp_path
        self._pipeline: Any = None
        self._appsrc: Any = None
        self._gst: Any = None
        self._ros_source: RosImageSource | None = None
        self._lock = threading.Lock()
        self._running = False
        self._frame_count = 0
        self._error_count = 0
        self._dropped_count = 0
        self._fps_value = 0.0
        self._start_time = 0.0
        self._last_frame_time = 0.0
        self._overlay_enabled = False
        self._overlay_detection_count = 0

    @property
    def source_topic(self) -> str:
        return self._source_topic

    @property
    def running(self) -> bool:
        return self._running

    def start(self) -> bool:
        with self._lock:
            if self._running:
                return True
            return self._start_pipeline()

    def stop(self) -> None:
        with self._lock:
            self._stop_pipeline()

    def switch_topic(self, topic: str) -> bool:
        with self._lock:
            if topic == self._source_topic:
                return True
            self._stop_pipeline()
            self._source_topic = topic
            return self._start_pipeline()

    def restart(self) -> bool:
        with self._lock:
            self._stop_pipeline()
            return self._start_pipeline()

    def get_status(self) -> dict:
        return {
            "streaming": self._running,
            "source_topic": self._source_topic,
            "fps": self._fps_value,
            "frame_count": self._frame_count,
            "error_count": self._error_count,
            "dropped_count": self._dropped_count,
            "last_frame_age_s": time.time() - self._last_frame_time if self._last_frame_time > 0 else -1,
            "width": self._width,
            "height": self._height,
            "bitrate_kbps": self._bitrate,
        }

    def get_health(self) -> dict:
        return {
            "healthy": self._running,
            "pipeline_playing": self._running,
            "source_topic": self._source_topic,
        }

    def set_overlay(self, enabled: bool) -> None:
        self._overlay_enabled = enabled

    def get_overlay_status(self) -> dict:
        return {
            "enabled": self._overlay_enabled,
            "detection_count": self._overlay_detection_count,
        }

    def _build_gst_pipeline(self) -> str:
        return (
            "appsrc name=ros_source is-live=true block=false format=time do-timestamp=true "
            f"caps=video/x-raw,format=RGB,width={self._width},height={self._height},framerate={self._fps}/1 ! "
            "queue max-size-buffers=2 leaky=downstream ! "
            "videoconvert ! videoscale ! "
            f"video/x-raw,width={self._width},height={self._height},framerate={self._fps}/1 ! "
            f"videoflip method={self._flip_method} ! "
            f"x264enc tune=zerolatency bitrate={self._bitrate} speed-preset=ultrafast "
            f"key-int-max={self._fps * 2} ! "
            "video/x-h264,profile=baseline ! "
            f"rtspclientsink location={self._rtsp_url} protocols=tcp latency=0"
        )

    def _reset_stats(self) -> None:
        self._running = True
        self._frame_count = 0
        self._dropped_count = 0
        self._fps_value = 0.0
        self._start_time = time.time()
        self._last_frame_time = 0.0

    def _start_pipeline(self) -> bool:
        if not self._start_gstreamer():
            return False
        if not self._start_ros_source():
            self._stop_pipeline()
            return False
        threading.Thread(target=self._monitor_pipeline, daemon=True).start()
        return True

    def _start_gstreamer(self) -> bool:
        try:
            import gi

            gi.require_version("Gst", "1.0")
            from gi.repository import Gst

            Gst.init(None)
            self._gst = Gst
            self._pipeline = Gst.parse_launch(self._build_gst_pipeline())
            self._appsrc = self._pipeline.get_by_name(_PIPELINE_NAME)
            self._pipeline.set_state(Gst.State.PLAYING)
            self._reset_stats()
            logger.info("GStreamer appsrc pipeline started: %s -> %s", self._source_topic, self._rtsp_url)
            return True
        except Exception as error:
            logger.error("Failed to start GStreamer pipeline: %s", error)
            self._error_count += 1
            return False

    def _start_ros_source(self) -> bool:
        self._ros_source = RosImageSource(self._source_topic, self._on_image)
        if self._ros_source.start():
            return True
        self._error_count += 1
        return False

    def _stop_pipeline(self) -> None:
        self._stop_ros_source()
        if self._pipeline is not None:
            try:
                self._pipeline.set_state(self._gst.State.NULL)
            except Exception as error:
                logger.debug("Pipeline stop failed: %s", error)
        self._pipeline = None
        self._appsrc = None
        self._running = False
        logger.info("GStreamer pipeline stopped")

    def _stop_ros_source(self) -> None:
        source = self._ros_source
        self._ros_source = None
        if source is not None:
            source.stop()

    def _on_image(self, msg) -> None:
        # Snapshot the pipeline handles: _stop_pipeline() may null them out from
        # another thread between this check and the push-buffer below.
        appsrc = self._appsrc
        gst = self._gst
        if not self._running or appsrc is None or gst is None:
            return
        try:
            frame = self._image_to_rgb(msg)
            if frame is None:
                return
            payload = frame.tobytes()
            buffer = gst.Buffer.new_allocate(None, len(payload), None)
            buffer.fill(0, payload)
            buffer.duration = gst.SECOND // max(self._fps, 1)
            if appsrc.emit("push-buffer", buffer) != gst.FlowReturn.OK:
                self._dropped_count += 1
                return
            self._count_frame()
        except Exception as error:
            self._error_count += 1
            logger.debug("Failed to push image frame: %s", error)

    def _count_frame(self) -> None:
        self._frame_count += 1
        self._last_frame_time = time.time()
        elapsed = self._last_frame_time - self._start_time
        if elapsed > 0:
            self._fps_value = self._frame_count / elapsed

    def _image_to_rgb(self, msg):
        try:
            return image_to_rgb(msg, self._width, self._height)
        except FrameConversionError as error:
            self._error_count += 1
            logger.warning("%s", error)
            return None

    def _monitor_pipeline(self) -> None:
        if self._pipeline is None:
            return
        bus = self._pipeline.get_bus()
        while self._running:
            message = bus.timed_pop_filtered(
                _GST_PARSE_TIMEOUT_NS,
                self._gst.MessageType.ERROR | self._gst.MessageType.EOS,
            )
            if message is None:
                continue
            if self._report_bus_message(message):
                break

    def _report_bus_message(self, message) -> bool:
        """Record a pipeline error or EOS. Returns True when the loop must stop."""
        if message.type == self._gst.MessageType.ERROR:
            error, debug = message.parse_error()
            self._running = False
            self._error_count += 1
            logger.warning("GStreamer pipeline error: %s; %s", error, debug)
            return True
        if message.type == self._gst.MessageType.EOS:
            self._running = False
            logger.warning("GStreamer pipeline reached EOS")
            return True
        return False
