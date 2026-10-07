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

Modules:
  video_bridge.py       GStreamer pipeline, stream statistics, overlay state
  video_ros_source.py   rclpy node, subscription and executor lifecycle
  video_frame.py        sensor_msgs/Image to RGB numpy conversion
  video_bridge_server.py HTTP API for control and status
  simple_video_bridge.py CLI entry point that wires the four together

HTTP API (default port 9200):
  GET  /health       — liveness probe
  GET  /status       — streaming stats (fps, frame count, errors)
  POST /switch       — change source image topic
  GET  /topics       — list available sensor_msgs/Image topics
  POST /restart      — restart the GStreamer pipeline
  POST /overlay/enable|disable — toggle detection bounding-box overlay
  GET  /overlay/status — current overlay state

Usage:
  python3 simple_video_bridge.py \
    --source-topic /nomad/camera/image \
    --width 640 --height 360 --fps 15 --bitrate 800 \
    --http-port 9200
"""

from __future__ import annotations

import argparse
import ipaddress
import logging
import os
import threading
import time

from python.tools.video_bridge import VideoBridge
from python.tools.video_bridge_server import BridgeHTTPServer

__all__ = ["VideoBridge", "main", "validate_http_host"]


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("simple_video_bridge")


def validate_http_host(host: str) -> str:
    """Allow only loopback HTTP binds for the unauthenticated local API."""

    candidate = host.strip()
    if candidate.casefold() == "localhost":
        return candidate
    try:
        address = ipaddress.ip_address(candidate)
    except ValueError as exc:
        raise ValueError("HTTP host must be localhost or a loopback IP address") from exc
    if not address.is_loopback:
        raise ValueError("HTTP host must be localhost or a loopback IP address")
    return candidate


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="NOMAD Simple Video Bridge")
    parser.add_argument("--source-topic", required=True, help="ROS2 image topic")
    parser.add_argument("--width", type=int, default=640, help="Output width")
    parser.add_argument("--height", type=int, default=360, help="Output height")
    parser.add_argument("--fps", type=int, default=15, help="Output frame rate")
    parser.add_argument("--bitrate", type=int, default=800, help="H.264 bitrate (kbps)")
    parser.add_argument(
        "--http-host",
        default=os.environ.get("VIDEO_RELAY_HTTP_HOST", "127.0.0.1"),
        help="HTTP API bind host; loopback addresses only",
    )
    parser.add_argument("--http-port", type=int, default=9200, help="HTTP API port")
    parser.add_argument("--rtsp-path", default="stream", help="RTSP stream path")
    parser.add_argument(
        "--flip-method",
        default=os.environ.get("NOMAD_VIDEO_FLIP_METHOD", "identity"),
        help="GStreamer videoflip method, e.g. identity, rotate-180, horizontal-flip.",
    )
    parser.add_argument(
        "--rtsp-url",
        default=os.environ.get("NOMAD_VIDEO_RTSP_PUBLISH_URL"),
        help="Full RTSP publish URL. Defaults to rtsp://localhost:8554/<rtsp-path>.",
    )
    args = parser.parse_args()
    try:
        args.http_host = validate_http_host(args.http_host)
    except ValueError as exc:
        parser.error(str(exc))
    return args


def _build_bridge(args: argparse.Namespace) -> VideoBridge:
    rtsp_url = args.rtsp_url or f"rtsp://localhost:8554/{args.rtsp_path}"
    return VideoBridge(
        source_topic=args.source_topic,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate=args.bitrate,
        rtsp_url=rtsp_url,
        flip_method=args.flip_method,
        rtsp_path=args.rtsp_path,
    )


def _serve_until_stopped(bridge: VideoBridge, server: BridgeHTTPServer) -> None:
    try:
        while bridge.running:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()
        server.shutdown()


def main() -> None:
    args = _parse_args()
    bridge = _build_bridge(args)
    if not bridge.start():
        logger.error("Failed to start video bridge pipeline")
        return

    server = BridgeHTTPServer(args.http_host, args.http_port, bridge)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    logger.info("HTTP API listening on port %s", args.http_port)
    _serve_until_stopped(bridge, server)


if __name__ == "__main__":
    main()
