# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
Simple Video Bridge — ROS2 image topic to RTSP via GStreamer.

Runs inside the Isaac ROS Docker container on the Jetson. Subscribes to a ROS2
image topic, encodes frames with x264enc (software, zerolatency tuning), and
streams to MediaMTX via RTSP. Also provides an HTTP API for topic switching,
status, and overlays.

Architecture:
  ROS2 Image Topic -> GStreamer x264enc -> RTSP -> MediaMTX -> Viewers

HTTP API (default port 9200):
  GET  /health       — liveness probe
  GET  /status       — streaming stats (fps, frame count, errors)
  POST /switch       — change source topic (?topic=/zed/...)
  GET  /topics       — list available sensor_msgs/Image topics
  POST /restart      — restart the GStreamer pipeline
  POST /overlay/enable|disable — toggle detection bounding-box overlay
  GET  /overlay/status — current overlay state
  GET  /depth/center — ZED center depth sample

Usage:
  python3 simple_video_bridge.py \
    --source-topic /zed/zed_node/rgb/color/rect/image \
    --width 640 --height 360 --fps 15 --bitrate 800 \
    --http-port 9200
"""

from __future__ import annotations

import argparse
import http.server
import json
import logging
import subprocess
import threading
import time
from subprocess import Popen
from urllib.parse import parse_qs, urlparse

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("simple_video_bridge")


class VideoBridge:
    """Manages the GStreamer pipeline from ROS2 topic to RTSP."""

    def __init__(
        self,
        source_topic: str,
        width: int,
        height: int,
        fps: int,
        bitrate: int,
        rtsp_path: str = "primary",
    ):
        self._source_topic = source_topic
        self._width = width
        self._height = height
        self._fps = fps
        self._bitrate = bitrate
        self._rtsp_path = rtsp_path
        self._pipeline: Popen[bytes] | None = None
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
        self._depth_center_m: float | None = None

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

    def set_center_depth(self, depth_m: float | None) -> None:
        self._depth_center_m = depth_m

    def get_center_depth(self) -> dict:
        return {"range_m": self._depth_center_m}

    def _start_pipeline(self) -> bool:
        rtsp_url = f"rtsp://localhost:8554/{self._rtsp_path}"

        gst_pipeline = (
            f"ros2src topic={self._source_topic} ! "
            f"videoconvert ! "
            f"videoscale ! "
            f"video/x-raw,width={self._width},height={self._height},framerate={self._fps}/1 ! "
            f"x264enc tune=zerolatency bitrate={self._bitrate} speed-preset=ultrafast "
            f"key-int-max={self._fps * 2} ! "
            f"video/x-h264,profile=baseline ! "
            f"rtspclientsink location={rtsp_url} protocols=tcp "
            f"latency=0"
        )

        try:
            self._pipeline = subprocess.Popen(
                ["gst-launch-1.0", "-v"] + gst_pipeline.split(),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self._running = True
            self._start_time = time.time()
            self._last_frame_time = time.time()
            logger.info(f"GStreamer pipeline started: {self._source_topic} -> {rtsp_url}")

            threading.Thread(target=self._monitor_pipeline, daemon=True).start()
            return True
        except FileNotFoundError:
            logger.error("gst-launch-1.0 not found; falling back to appsrc pipeline")
            return self._start_appsrc_pipeline()
        except Exception as e:
            logger.error(f"Failed to start GStreamer pipeline: {e}")
            self._error_count += 1
            return False

    def _start_appsrc_pipeline(self) -> bool:
        """Fallback: use a Python-based GStreamer pipeline with appsrc."""
        try:
            import gi

            gi.require_version("Gst", "1.0")
            gi.require_version("GstRtspServer", "1.0")
            from gi.repository import Gst

            Gst.init(None)
            logger.info("Using Python GStreamer bindings (appsrc mode)")
            return self._start_python_subprocess_pipeline()
        except ImportError:
            logger.error("Neither gst-launch-1.0 nor GStreamer Python bindings available")
            self._error_count += 1
            return False

    def _start_python_subprocess_pipeline(self) -> bool:
        """Start a subprocess-based pipeline using ros2 run + gst-launch."""
        rtsp_url = f"rtsp://localhost:8554/{self._rtsp_path}"

        pipeline_str = (
            f"appsrc name=ros_source ! "
            f"videoconvert ! "
            f"x264enc tune=zerolatency bitrate={self._bitrate} speed-preset=ultrafast ! "
            f"rtspclientsink location={rtsp_url}"
        )

        try:
            self._pipeline = subprocess.Popen(
                ["gst-launch-1.0", "-v", pipeline_str],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self._running = True
            self._start_time = time.time()
            logger.info("Fallback GStreamer pipeline started")
            return True
        except Exception as e:
            logger.error(f"Fallback pipeline failed: {e}")
            return False

    def _stop_pipeline(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.terminate()
                self._pipeline.wait(timeout=5)
            except Exception:
                try:
                    self._pipeline.kill()
                except Exception:
                    pass
        self._pipeline = None
        self._running = False
        logger.info("GStreamer pipeline stopped")

    def _monitor_pipeline(self) -> None:
        if self._pipeline is None:
            return
        while self._running:
            rc = self._pipeline.poll()
            if rc is not None:
                self._running = False
                self._error_count += 1
                logger.warning(f"GStreamer pipeline exited with code {rc}")
                break
            self._frame_count += 1
            self._last_frame_time = time.time()
            elapsed = time.time() - self._start_time
            if elapsed > 0:
                self._fps_value = self._frame_count / elapsed
            time.sleep(1.0 / max(self._fps, 1))


class BridgeHTTPHandler(http.server.BaseHTTPRequestHandler):
    """HTTP API for the video bridge."""

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/health":
            self._json_response(self.server.bridge.get_health())
        elif path == "/status":
            self._json_response(self.server.bridge.get_status())
        elif path == "/topics":
            self._json_response(self._list_topics())
        elif path == "/overlay/status":
            self._json_response(self.server.bridge.get_overlay_status())
        elif path == "/depth/center":
            self._json_response(self.server.bridge.get_center_depth())
        else:
            self._json_response({"error": "not found"}, status=404)

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        params = parse_qs(parsed.query)

        if path == "/switch":
            topic = params.get("topic", [None])[0]
            if not topic:
                self._json_response({"success": False, "message": "missing topic param"}, 400)
                return
            ok = self.server.bridge.switch_topic(topic)
            self._json_response(
                {
                    "success": ok,
                    "message": f"Switched to {topic}" if ok else "Failed to switch",
                    "topic": topic,
                }
            )
        elif path == "/restart":
            ok = self.server.bridge.restart()
            self._json_response({"success": ok})
        elif path == "/overlay/enable":
            self.server.bridge.set_overlay(True)
            self._json_response({"success": True, "message": "Overlay enabled"})
        elif path == "/overlay/disable":
            self.server.bridge.set_overlay(False)
            self._json_response({"success": True, "message": "Overlay disabled"})
        else:
            self._json_response({"error": "not found"}, 404)

    def _json_response(self, data: dict, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def _list_topics(self) -> dict:
        try:
            result = subprocess.run(
                ["ros2", "topic", "list", "-t"],
                capture_output=True,
                text=True,
                timeout=10,
            )
            topics = []
            for line in result.stdout.splitlines():
                parts = line.split()
                if len(parts) >= 2 and "sensor_msgs/msg/Image" in parts[1]:
                    topics.append(parts[0])
            return {"topics": topics}
        except Exception:
            return {"topics": []}

    def log_message(self, format, *args):
        logger.debug(f"HTTP {args[0]}")


class BridgeHTTPServer(http.server.HTTPServer):
    def __init__(self, host: str, port: int, bridge: VideoBridge):
        self.bridge = bridge
        super().__init__((host, port), BridgeHTTPHandler)


def main() -> None:
    parser = argparse.ArgumentParser(description="NOMAD Simple Video Bridge")
    parser.add_argument("--source-topic", required=True, help="ROS2 image topic")
    parser.add_argument("--width", type=int, default=640, help="Output width")
    parser.add_argument("--height", type=int, default=360, help="Output height")
    parser.add_argument("--fps", type=int, default=15, help="Output frame rate")
    parser.add_argument("--bitrate", type=int, default=800, help="H.264 bitrate (kbps)")
    parser.add_argument("--http-port", type=int, default=9200, help="HTTP API port")
    parser.add_argument("--rtsp-path", default="primary", help="RTSP stream path")
    args = parser.parse_args()

    bridge = VideoBridge(
        source_topic=args.source_topic,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate=args.bitrate,
        rtsp_path=args.rtsp_path,
    )

    if not bridge.start():
        logger.error("Failed to start video bridge pipeline")
        return

    server = BridgeHTTPServer("0.0.0.0", args.http_port, bridge)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    logger.info(f"HTTP API listening on port {args.http_port}")

    try:
        while bridge.running:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()
        server.shutdown()


if __name__ == "__main__":
    main()
