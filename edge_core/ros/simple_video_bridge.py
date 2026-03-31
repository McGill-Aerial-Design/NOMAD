#!/usr/bin/env python3
"""
Simple Video Bridge for NOMAD - Reliable Implementation

Streams ROS2 camera topics to RTSP via MediaMTX using software H.264 encoding.

Features:
- Software encoding (x264enc with zerolatency tuning, openh264enc fallback)
- Fixed RTSP URL: rtsp://localhost:8554/primary
- Dynamic topic switching via HTTP API
- Auto-discovery of available ROS2 image topics
- MediaMTX for multi-viewer streaming (VLC, phone, Mission Planner)
- Pipeline watchdog with automatic recovery
- Proper frame pacing and keyframe control

Architecture:
    ROS2 Image Topic -> x264enc (software, zerolatency) -> RTSP -> MediaMTX

Target: Jetson Orin Nano / Isaac ROS Docker container
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import numpy as np
import threading
import time
import subprocess
import json
import urllib.request
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

# Initialize GStreamer
Gst.init(None)


def _probe_encoder():
    """Probe which H.264 software encoder is available.

    Tries x264enc first (better quality, proper keyframe control),
    falls back to openh264enc.  Returns a tuple of
    (element_name, extra_pipeline_fragment) or raises if none found.
    """
    # x264enc: best software H.264 on Linux
    for name, fragment in [
        (
            "x264enc",
            "x264enc tune=zerolatency speed-preset=ultrafast "
            "bitrate={bitrate} key-int-max={keyint} bframes=0 "
            "b-adapt=false sliced-threads=true threads={threads} "
            "! video/x-h264,profile=baseline",
        ),
        (
            "openh264enc",
            "openh264enc bitrate={bitrate_bps} complexity=low "
            "! video/x-h264,profile=baseline",
        ),
    ]:
        factory = Gst.ElementFactory.find(name)
        if factory is not None:
            return name, fragment

    raise RuntimeError(
        "No H.264 software encoder found. Install gstreamer1.0-plugins-ugly (x264enc) "
        "or gstreamer1.0-plugins-bad (openh264enc)."
    )


class VideoStreamNode(Node):
    """ROS2 node that streams images to RTSP via GStreamer."""

    # Minimum interval between pipeline restart attempts (seconds)
    _MIN_RESTART_INTERVAL = 5.0

    def __init__(self, source_topic: str, width: int = 1280, height: int = 720,
                 fps: int = 30, bitrate: int = 2000, rtsp_path: str = "primary"):
        super().__init__('simple_video_bridge')

        self.bridge = CvBridge()
        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate = bitrate  # kbps
        self.rtsp_path = rtsp_path
        self.source_topic = source_topic
        self.frame_count = 0
        self.error_count = 0
        self.start_time = time.time()
        self.last_frame_time = 0.0
        self.subscription = None
        self._latest_jpeg = None
        self._last_snapshot_encode_time = 0.0

        # Frame pacing: enforce max fps to prevent encoder overload
        self._min_frame_interval = 1.0 / fps
        self._last_push_time = 0.0
        self._pts_counter = 0  # Monotonic PTS for the encoder

        # Pipeline recovery
        self._pipeline_ok = True
        self._last_restart_time = 0.0
        self._restart_lock = threading.Lock()

        # Detection overlay state
        self._overlay_enabled = False
        self._detections = []
        self._detections_lock = threading.Lock()
        self._edge_core_url = "http://172.17.0.1:8000"
        self._overlay_thread = None
        self._overlay_stop = threading.Event()

        # Probe available encoder
        self._encoder_name, self._encoder_fragment = _probe_encoder()
        self.get_logger().info(f'Using encoder: {self._encoder_name}')

        # Build and start pipeline
        self.pipeline = None
        self.appsrc = None
        self._build_and_start_pipeline()

        # Subscribe to ROS2 image topic
        self._subscribe_to_topic(source_topic)

        # Start GStreamer bus watchdog
        self._bus_thread = threading.Thread(target=self._bus_watch_loop, daemon=True)
        self._bus_thread.start()

        self.get_logger().info('Video bridge ready!')

    def _build_pipeline_string(self) -> str:
        """Build the GStreamer pipeline string."""
        width, height, fps, bitrate = self.width, self.height, self.fps, self.bitrate
        keyint = fps * 2  # IDR every 2 seconds for reliable stream recovery
        threads = min(4, max(1, __import__('os').cpu_count() or 2))

        encoder_str = self._encoder_fragment.format(
            bitrate=bitrate,
            bitrate_bps=bitrate * 1000,
            keyint=keyint,
            threads=threads,
        )

        pipeline_str = (
            f'appsrc name=source is-live=true format=time '
            f'max-buffers=1 block=false '
            f'caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! '
            f'queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream ! '
            f'videoconvert ! '
            f'{encoder_str} ! '
            f'h264parse config-interval=-1 ! '
            f'rtspclientsink location=rtsp://172.17.0.1:8554/{self.rtsp_path} protocols=tcp '
            f'latency=0'
        )
        return pipeline_str

    def _build_and_start_pipeline(self):
        """Build and start the GStreamer pipeline."""
        # Tear down existing pipeline
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
            self.appsrc = None

        pipeline_str = self._build_pipeline_string()

        self.get_logger().info('Starting GStreamer pipeline')
        self.get_logger().info(f'  Topic: {self.source_topic}')
        self.get_logger().info(f'  Resolution: {self.width}x{self.height}@{self.fps}fps')
        self.get_logger().info(f'  Encoder: {self._encoder_name}')
        self.get_logger().info(f'  Bitrate: {self.bitrate}kbps')
        self.get_logger().info(f'  RTSP: rtsp://localhost:8554/{self.rtsp_path}')

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsrc = self.pipeline.get_by_name('source')

            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                self.get_logger().error('Failed to start GStreamer pipeline')
                raise RuntimeError('GStreamer pipeline failed to start')

            self._pipeline_ok = True
            self._pts_counter = 0
            self.get_logger().info('GStreamer pipeline started successfully')

        except Exception as e:
            self.get_logger().error(f'Pipeline error: {e}')
            self._pipeline_ok = False
            raise

    def _restart_pipeline(self):
        """Restart the GStreamer pipeline after an error."""
        with self._restart_lock:
            now = time.time()
            if now - self._last_restart_time < self._MIN_RESTART_INTERVAL:
                return
            self._last_restart_time = now

        self.get_logger().warn('Restarting GStreamer pipeline due to error...')
        try:
            self._build_and_start_pipeline()
            self.get_logger().info('Pipeline restarted successfully')
        except Exception as e:
            self.get_logger().error(f'Pipeline restart failed: {e}')
            self._pipeline_ok = False

    def _bus_watch_loop(self):
        """Monitor GStreamer bus for errors and trigger recovery."""
        while True:
            if self.pipeline is None:
                time.sleep(1)
                continue

            bus = self.pipeline.get_bus()
            if bus is None:
                time.sleep(1)
                continue

            msg = bus.timed_pop_filtered(
                1 * Gst.SECOND,
                Gst.MessageType.ERROR | Gst.MessageType.EOS | Gst.MessageType.STATE_CHANGED
            )

            if msg is None:
                # Check for stale pipeline (no frames pushed for 10s while we should be streaming)
                if (self.last_frame_time > 0
                        and time.time() - self.last_frame_time > 15.0
                        and self._pipeline_ok):
                    self.get_logger().warn('No frames for 15s, restarting pipeline')
                    self._pipeline_ok = False
                    self._restart_pipeline()
                continue

            msg_type = msg.type
            if msg_type == Gst.MessageType.ERROR:
                err, debug = msg.parse_error()
                self.get_logger().error(f'GStreamer error: {err.message} | debug: {debug}')
                self.error_count += 1
                self._pipeline_ok = False
                self._restart_pipeline()
            elif msg_type == Gst.MessageType.EOS:
                self.get_logger().warn('GStreamer pipeline received EOS, restarting')
                self._pipeline_ok = False
                self._restart_pipeline()

    def _subscribe_to_topic(self, topic: str):
        """Subscribe to a ROS2 image topic."""
        if self.subscription:
            self.destroy_subscription(self.subscription)

        # BEST_EFFORT + KEEP_LAST(1): drop frames rather than queue them.
        # Camera publishers (ZED, realsense) use BEST_EFFORT by default.
        # Using RELIABLE here would cause backpressure when the encoder
        # can't keep up, leading to stale frames and timestamp jumps.
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            image_qos
        )
        self.source_topic = topic
        self.get_logger().info(f'Subscribed to: {topic}')

    def switch_topic(self, new_topic: str) -> bool:
        """Switch to a different ROS2 image topic.

        Keeps the existing GStreamer pipeline alive and only switches
        the ROS subscription.
        """
        try:
            self.get_logger().info(f'Switching topic: {self.source_topic} -> {new_topic}')
            self._subscribe_to_topic(new_topic)
            self.get_logger().info(f'Successfully switched to: {new_topic}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to switch topic: {e}')
            return False

    def image_callback(self, msg: Image):
        """Process incoming ROS2 images and push to GStreamer."""
        if not self._pipeline_ok or self.appsrc is None:
            return

        try:
            import cv2

            # Frame pacing: skip frames that arrive faster than target fps
            now = time.time()
            if now - self._last_push_time < self._min_frame_interval * 0.8:
                return

            # Convert ROS image to OpenCV BGR format
            encoding = msg.encoding.lower()
            if encoding in ('bgra8', 'rgba8', '8uc4'):
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                if encoding == 'rgba8':
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2BGR)
                else:
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            elif encoding in ('16uc1', 'mono16', '32fc1'):
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv_image = self._normalize_depth_image(cv_image, encoding)
            elif encoding in ('mono8', '8uc1'):
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize if needed
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                cv_image = cv2.resize(cv_image, (self.width, self.height))

            # Draw detection overlay if enabled
            if self._overlay_enabled:
                self.draw_detections(cv_image)

            # Cache snapshots at a low rate
            if (now - self._last_snapshot_encode_time) >= 0.5:
                ok, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok:
                    self._latest_jpeg = jpeg.tobytes()
                    self._last_snapshot_encode_time = now

            # Create GStreamer buffer with monotonic PTS
            data = cv_image.tobytes()
            buf = Gst.Buffer.new_wrapped(data)

            # Set monotonic timestamps so the encoder gets clean, regular timing
            # instead of relying on wall-clock do-timestamp which jitters.
            frame_duration = Gst.SECOND // self.fps
            buf.pts = self._pts_counter * frame_duration
            buf.duration = frame_duration
            self._pts_counter += 1

            ret = self.appsrc.emit('push-buffer', buf)

            if ret != Gst.FlowReturn.OK:
                self.get_logger().warn(f'Buffer push returned: {ret}')
                if ret == Gst.FlowReturn.FLUSHING or ret == Gst.FlowReturn.ERROR:
                    self._pipeline_ok = False
                return

            # Stats
            self._last_push_time = now
            self.frame_count += 1
            self.last_frame_time = now
            if self.frame_count % 300 == 0:
                elapsed = now - self.start_time
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                self.get_logger().info(
                    f'Streaming: {self.frame_count} frames, {fps:.1f} fps avg'
                )

        except Exception as e:
            if self.frame_count % 100 == 0:
                self.get_logger().error(f'Frame processing error: {e}')

    def _normalize_depth_image(self, image, encoding):
        """Normalize depth/confidence image to visible BGR with colormap."""
        import cv2

        img = image.astype(np.float32) if encoding != '32fc1' else image.copy()

        valid_mask = np.isfinite(img) & (img > 0)

        if not valid_mask.any():
            return np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)

        min_val = np.percentile(img[valid_mask], 2)
        max_val = np.percentile(img[valid_mask], 98)

        if max_val <= min_val:
            max_val = min_val + 1.0

        normalized = np.clip((img - min_val) / (max_val - min_val) * 255.0, 0, 255).astype(np.uint8)
        normalized[~valid_mask] = 0

        colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        return colored

    def cleanup(self):
        """Clean shutdown."""
        self.get_logger().info('Stopping video bridge...')
        self.stop_overlay()
        if self.pipeline is not None:
            # Send EOS so the encoder flushes cleanly
            self.appsrc.emit('end-of-stream')
            self.pipeline.get_bus().timed_pop_filtered(2 * Gst.SECOND, Gst.MessageType.EOS)
            self.pipeline.set_state(Gst.State.NULL)
        self.get_logger().info('Stopped')

    # ---- Detection overlay ----

    _LABEL_COLORS = {
        'red': (0, 0, 255),
        'blue': (255, 140, 0),
        'green': (0, 200, 0),
        'yellow': (0, 255, 255),
        'orange': (0, 165, 255),
        'white': (255, 255, 255),
        'black': (80, 80, 80),
    }
    _DEFAULT_COLOR = (0, 255, 0)

    def _color_for_label(self, label: str):
        label_lower = label.lower()
        for key, color in self._LABEL_COLORS.items():
            if key in label_lower:
                return color
        return self._DEFAULT_COLOR

    def draw_detections(self, frame):
        """Draw detection bounding boxes onto the frame (in-place)."""
        import cv2

        with self._detections_lock:
            detections = list(self._detections)

        if not detections:
            return

        h, w = frame.shape[:2]

        for det in detections:
            if not isinstance(det, dict):
                continue
            try:
                bx = float(det.get('bbox_x', 0) or 0)
                by = float(det.get('bbox_y', 0) or 0)
                bw = float(det.get('bbox_w', 0) or 0)
                bh = float(det.get('bbox_h', 0) or 0)
            except (TypeError, ValueError):
                continue
            if bw <= 0 or bh <= 0:
                continue

            src_w = det.get('_src_w', w)
            src_h = det.get('_src_h', h)
            sx = w / src_w if src_w > 0 else 1.0
            sy = h / src_h if src_h > 0 else 1.0

            x1 = int(bx * sx)
            y1 = int(by * sy)
            x2 = int((bx + bw) * sx)
            y2 = int((by + bh) * sy)

            label = str(det.get('label', 'unknown') or 'unknown')
            try:
                conf = float(det.get('confidence', 0.0) or 0.0)
            except (TypeError, ValueError):
                conf = 0.0
            color = self._color_for_label(label)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            text = f"{label} {conf:.0%}"
            (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1, y1 - th - baseline - 4), (x1 + tw + 4, y1), color, -1)
            cv2.putText(frame, text, (x1 + 2, y1 - baseline - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        count = len(detections)
        badge = f"YOLO: {count} target{'s' if count != 1 else ''}"
        cv2.putText(frame, badge, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

    def _fetch_detections_loop(self):
        """Background thread: poll Edge Core /api/detections at ~5 Hz."""
        while not self._overlay_stop.is_set():
            try:
                url = f"{self._edge_core_url}/api/detections"
                req = urllib.request.Request(url, method='GET')
                req.add_header('Accept', 'application/json')
                with urllib.request.urlopen(req, timeout=2) as resp:
                    data = json.loads(resp.read().decode())
                    current = data.get('current', {})
                    dets = current.get('detections', []) if isinstance(current, dict) else []
                    with self._detections_lock:
                        self._detections = dets
            except Exception:
                with self._detections_lock:
                    self._detections = []
            self._overlay_stop.wait(0.2)

    def start_overlay(self):
        """Enable detection overlay on the video stream."""
        if self._overlay_enabled:
            return
        self._overlay_enabled = True
        self._overlay_stop.clear()
        self._overlay_thread = threading.Thread(
            target=self._fetch_detections_loop, daemon=True, name='overlay-fetch')
        self._overlay_thread.start()
        self.get_logger().info("Detection overlay enabled")

    def stop_overlay(self):
        """Disable detection overlay."""
        if not self._overlay_enabled:
            return
        self._overlay_enabled = False
        self._overlay_stop.set()
        if self._overlay_thread:
            self._overlay_thread.join(timeout=2)
            self._overlay_thread = None
        with self._detections_lock:
            self._detections = []
        self.get_logger().info("Detection overlay disabled")


class ControlServer(BaseHTTPRequestHandler):
    """HTTP server for video bridge control."""

    video_node = None

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        parsed = urlparse(self.path)

        pipeline_playing = False
        if self.video_node and getattr(self.video_node, 'pipeline', None):
            try:
                state = self.video_node.pipeline.get_state(0)[1]
                pipeline_playing = state == Gst.State.PLAYING
            except Exception:
                pipeline_playing = False

        if parsed.path == '/health':
            receiving_frames = (
                self.video_node is not None
                and self.video_node.last_frame_time > 0
                and (time.time() - self.video_node.last_frame_time) < 5.0
            )
            self._send_json(200, {
                'healthy': pipeline_playing,
                'streaming': receiving_frames,
                'pipeline_playing': pipeline_playing,
                'encoder': self.video_node._encoder_name if self.video_node else '',
            })

        elif parsed.path == '/status':
            now = time.time()
            receiving_frames = (
                self.video_node is not None
                and self.video_node.last_frame_time > 0
                and (now - self.video_node.last_frame_time) < 5.0
            )
            last_frame_age = (
                (now - self.video_node.last_frame_time)
                if self.video_node and self.video_node.last_frame_time > 0
                else None
            )
            elapsed = now - self.video_node.start_time if self.video_node else 1
            status = {
                'streaming': receiving_frames,
                'pipeline_playing': pipeline_playing,
                'source_topic': self.video_node.source_topic if self.video_node else '',
                'fps': self.video_node.frame_count / max(elapsed, 1) if self.video_node else 0,
                'frame_count': self.video_node.frame_count if self.video_node else 0,
                'error_count': self.video_node.error_count if self.video_node else 0,
                'last_frame_age_s': last_frame_age,
                'rtsp_url': f'rtsp://localhost:8554/{self.video_node.rtsp_path}' if self.video_node else '',
                'encoder': self.video_node._encoder_name if self.video_node else '',
            }
            self._send_json(200, status)

        elif parsed.path == '/topics':
            topics = self._discover_topics()
            self._send_json(200, {'topics': topics, 'count': len(topics)})

        elif parsed.path == '/snapshot':
            if self.video_node and self.video_node._latest_jpeg:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(self.video_node._latest_jpeg)))
                self.end_headers()
                self.wfile.write(self.video_node._latest_jpeg)
            else:
                self._send_json(503, {'error': 'No frame available'})

        elif parsed.path == '/overlay/status':
            det_count = 0
            if self.video_node:
                with self.video_node._detections_lock:
                    det_count = len(self.video_node._detections)
            self._send_json(200, {
                'enabled': self.video_node._overlay_enabled if self.video_node else False,
                'detection_count': det_count,
            })

        else:
            self._send_json(404, {'error': 'Not found'})

    def do_POST(self):
        parsed = urlparse(self.path)

        if parsed.path == '/switch':
            query = parse_qs(parsed.query)
            new_topic = query.get('topic', [''])[0]

            if not new_topic:
                self._send_json(400, {'success': False, 'message': 'Missing topic parameter'})
                return

            if self.video_node and self.video_node.switch_topic(new_topic):
                self._send_json(200, {
                    'success': True,
                    'message': f'Switched to {new_topic}',
                    'topic': new_topic
                })
            else:
                self._send_json(500, {'success': False, 'message': 'Failed to switch topic'})

        elif parsed.path == '/overlay/enable':
            if self.video_node:
                self.video_node.start_overlay()
                self._send_json(200, {'success': True, 'overlay': True})
            else:
                self._send_json(503, {'success': False, 'message': 'No video node'})

        elif parsed.path == '/overlay/disable':
            if self.video_node:
                self.video_node.stop_overlay()
                self._send_json(200, {'success': True, 'overlay': False})
            else:
                self._send_json(503, {'success': False, 'message': 'No video node'})

        elif parsed.path == '/restart':
            if self.video_node:
                self.video_node._restart_pipeline()
                self._send_json(200, {'success': True, 'message': 'Pipeline restarted'})
            else:
                self._send_json(503, {'success': False, 'message': 'No video node'})

        else:
            self._send_json(404, {'error': 'Not found'})

    def _send_json(self, code: int, data: dict):
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def _discover_topics(self) -> list:
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'list', '-t'],
                capture_output=True,
                text=True,
                timeout=10
            )

            topics = []
            for line in result.stdout.splitlines():
                parts = line.split()
                if len(parts) >= 2:
                    topic, type_ = parts[0], parts[1].strip('[]')
                    if 'Image' in type_ and 'sensor_msgs' in type_:
                        topics.append(topic)

            return sorted(topics)

        except Exception:
            return []


def run_http_server(video_node: VideoStreamNode, port: int = 9200):
    """Run HTTP control server in background thread."""
    ControlServer.video_node = video_node
    server = HTTPServer(('0.0.0.0', port), ControlServer)

    def serve():
        video_node.get_logger().info(f'HTTP control server started on port {port}')
        server.serve_forever()

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    return server


def main(args=None):
    """Main entry point."""
    import argparse
    import signal
    import sys

    parser = argparse.ArgumentParser(description='Simple Video Bridge for NOMAD')
    parser.add_argument('--source-topic', type=str,
                       default='/zed/zed_node/rgb/image_rect_color',
                       help='Initial ROS2 image topic to stream')
    parser.add_argument('--width', type=int, default=1280,
                       help='Output video width')
    parser.add_argument('--height', type=int, default=720,
                       help='Output video height')
    parser.add_argument('--fps', type=int, default=30,
                       help='Target framerate')
    parser.add_argument('--bitrate', type=int, default=2000,
                       help='H264 bitrate in kbps')
    parser.add_argument('--http-port', type=int, default=9200,
                       help='HTTP control server port')
    parser.add_argument('--rtsp-path', type=str, default='primary',
                       help='RTSP path on MediaMTX (e.g. primary, secondary)')

    parsed_args = parser.parse_args()

    def signal_handler(sig, frame):
        print('\nShutting down...')
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rclpy.init(args=args)

    video_node = VideoStreamNode(
        source_topic=parsed_args.source_topic,
        width=parsed_args.width,
        height=parsed_args.height,
        fps=parsed_args.fps,
        bitrate=parsed_args.bitrate,
        rtsp_path=parsed_args.rtsp_path,
    )

    http_server = run_http_server(video_node, parsed_args.http_port)

    try:
        rclpy.spin(video_node)
    except KeyboardInterrupt:
        pass
    finally:
        video_node.cleanup()
        video_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
