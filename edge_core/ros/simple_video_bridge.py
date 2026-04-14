#!/usr/bin/env python3
"""
Simple Video Bridge for NOMAD

Streams ROS2 camera topics to RTSP via MediaMTX using software H.264 encoding.
Optimized for Jetson Orin Nano (no hardware encoder).

Key optimizations:
- Dedicated encoder thread decoupled from ROS callbacks
- 848x480 default (55% fewer pixels than 720p) — plenty for monitoring
- 15fps default — smooth enough, half the CPU of 30fps
- Minimal copies in the hot path

Architecture:
    ROS2 Image Topic -> [encoder thread] -> x264enc (software) -> RTSP -> MediaMTX
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
import os
import json
import urllib.request
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

Gst.init(None)


def _probe_encoder():
    """Probe available H.264 software encoder.

    Returns (element_name, pipeline_fragment_template).
    """
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
        if Gst.ElementFactory.find(name) is not None:
            return name, fragment

    raise RuntimeError(
        "No H.264 encoder found. Need gstreamer1.0-plugins-ugly (x264enc) "
        "or gstreamer1.0-plugins-bad (openh264enc)."
    )


class VideoStreamNode(Node):
    """ROS2 node that streams images to RTSP via GStreamer."""

    _MIN_RESTART_INTERVAL = 5.0

    def __init__(self, source_topic: str, width: int = 848, height: int = 480,
                 fps: int = 15, bitrate: int = 1500, rtsp_path: str = "primary"):
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

        # --- Threaded encoder: ROS callback drops frame into _pending_frame,
        # encoder thread picks it up at its own pace. ---
        self._pending_frame = None  # numpy array or None
        self._frame_event = threading.Event()
        self._encode_stop = threading.Event()

        # Frame pacing
        self._min_frame_interval = 1.0 / fps
        self._last_push_time = 0.0
        self._pts_counter = 0

        # Pipeline recovery
        self._pipeline_ok = True
        self._last_restart_time = 0.0
        self._restart_lock = threading.Lock()

        # Detection overlay
        self._overlay_enabled = False
        self._detections = []
        self._detections_lock = threading.Lock()
        self._edge_core_url = "http://172.17.0.1:8000"
        self._edge_core_api_key = (os.environ.get("NOMAD_API_KEY") or "").strip()
        self._edge_core_internal_token = (
            os.environ.get("NOMAD_INTERNAL_TOKEN") or ""
        ).strip()
        self._overlay_thread = None
        self._overlay_stop = threading.Event()
        self._overlay_last_error_log_ts = 0.0

        # Probe encoder
        self._encoder_name, self._encoder_fragment = _probe_encoder()
        self.get_logger().info(f'Using encoder: {self._encoder_name}')

        # Build and start pipeline
        self.pipeline = None
        self.appsrc = None
        self._build_and_start_pipeline()

        # Subscribe to ROS2
        self._subscribe_to_topic(source_topic)

        # Start encoder thread (picks frames from _pending_frame, pushes to GStreamer)
        self._encode_thread = threading.Thread(target=self._encode_loop, daemon=True,
                                               name='encode-loop')
        self._encode_thread.start()

        # Start bus watchdog
        self._bus_thread = threading.Thread(target=self._bus_watch_loop, daemon=True)
        self._bus_thread.start()

        self.get_logger().info('Video bridge ready!')

    # ---- Pipeline management ----

    def _build_pipeline_string(self) -> str:
        width, height, fps, bitrate = self.width, self.height, self.fps, self.bitrate
        keyint = fps * 2
        threads = 2  # 2 threads is sufficient for 848x480@15fps ultrafast x264; frees cores for nvblox/SLAM

        encoder_str = self._encoder_fragment.format(
            bitrate=bitrate,
            bitrate_bps=bitrate * 1000,
            keyint=keyint,
            threads=threads,
        )

        return (
            f'appsrc name=source is-live=true format=time '
            f'max-buffers=1 block=false '
            f'caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! '
            f'queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream ! '
            f'videoconvert n-threads={threads} ! '
            f'{encoder_str} ! '
            f'h264parse config-interval=-1 ! '
            f'rtspclientsink location=rtsp://172.17.0.1:8554/{self.rtsp_path} protocols=tcp '
            f'latency=0'
        )

    def _build_and_start_pipeline(self):
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
            self.appsrc = None

        pipeline_str = self._build_pipeline_string()

        self.get_logger().info('Starting GStreamer pipeline')
        self.get_logger().info(f'  Resolution: {self.width}x{self.height}@{self.fps}fps')
        self.get_logger().info(f'  Encoder: {self._encoder_name}')
        self.get_logger().info(f'  Bitrate: {self.bitrate}kbps')
        self.get_logger().info(f'  Pipeline: {pipeline_str}')

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsrc = self.pipeline.get_by_name('source')

            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError('GStreamer pipeline failed to start')

            self._pipeline_ok = True
            self._pts_counter = 0
            self.get_logger().info('GStreamer pipeline started')

        except Exception as e:
            self.get_logger().error(f'Pipeline error: {e}')
            self._pipeline_ok = False
            raise

    def _restart_pipeline(self):
        with self._restart_lock:
            now = time.time()
            if now - self._last_restart_time < self._MIN_RESTART_INTERVAL:
                return
            self._last_restart_time = now

        self.get_logger().warn('Restarting GStreamer pipeline...')
        try:
            self._build_and_start_pipeline()
            self.get_logger().info('Pipeline restarted')
        except Exception as e:
            self.get_logger().error(f'Pipeline restart failed: {e}')
            self._pipeline_ok = False

    def _bus_watch_loop(self):
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
                if (self.last_frame_time > 0
                        and time.time() - self.last_frame_time > 15.0
                        and self._pipeline_ok):
                    self.get_logger().warn('No frames for 15s, restarting pipeline')
                    self._pipeline_ok = False
                    self._restart_pipeline()
                continue

            if msg.type == Gst.MessageType.ERROR:
                err, _debug = msg.parse_error()
                self.get_logger().error(f'GStreamer error: {err.message}')
                self.error_count += 1
                self._pipeline_ok = False
                self._restart_pipeline()
            elif msg.type == Gst.MessageType.EOS:
                self.get_logger().warn('EOS received, restarting')
                self._pipeline_ok = False
                self._restart_pipeline()

    # ---- ROS subscription ----

    def _subscribe_to_topic(self, topic: str):
        if self.subscription:
            self.destroy_subscription(self.subscription)

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            Image, topic, self.image_callback, image_qos)
        self.source_topic = topic
        self.get_logger().info(f'Subscribed to: {topic}')

    def switch_topic(self, new_topic: str) -> bool:
        try:
            self.get_logger().info(f'Switching topic: {self.source_topic} -> {new_topic}')
            self._subscribe_to_topic(new_topic)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to switch topic: {e}')
            return False

    # ---- Frame handling ----

    def image_callback(self, msg: Image):
        """Receive ROS image — minimal work, just stash for encoder thread."""
        now = time.time()
        if now - self._last_push_time < self._min_frame_interval * 0.8:
            return

        try:
            import cv2

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

            # Resize to target (this is fast if already the right size)
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                cv_image = cv2.resize(cv_image, (self.width, self.height),
                                      interpolation=cv2.INTER_NEAREST)

            # Overlay detections if enabled
            if self._overlay_enabled:
                self.draw_detections(cv_image)

            # Ensure contiguous C-order for zero-copy tobytes
            if not cv_image.flags['C_CONTIGUOUS']:
                cv_image = np.ascontiguousarray(cv_image)

            # Drop frame into single slot for encoder thread
            self._pending_frame = cv_image
            self._frame_event.set()
            self._last_push_time = now

        except Exception as e:
            if self.frame_count % 100 == 0:
                self.get_logger().error(f'Frame error: {e}')

    def _encode_loop(self):
        """Encoder thread: picks up frames and pushes to GStreamer pipeline."""
        frame_duration = Gst.SECOND // self.fps

        while not self._encode_stop.is_set():
            # Wait for a frame (with timeout so we can check stop flag)
            if not self._frame_event.wait(timeout=1.0):
                continue
            self._frame_event.clear()

            # Grab the latest frame (atomic swap)
            frame = self._pending_frame
            self._pending_frame = None
            if frame is None:
                continue

            if not self._pipeline_ok or self.appsrc is None:
                continue

            now = time.time()

            # Snapshot caching (every 2s, low priority)
            if (now - self._last_snapshot_encode_time) >= 2.0:
                try:
                    import cv2
                    ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if ok:
                        self._latest_jpeg = jpeg.tobytes()
                        self._last_snapshot_encode_time = now
                except Exception:
                    pass

            # Push to GStreamer
            data = frame.tobytes()
            buf = Gst.Buffer.new_wrapped(data)
            buf.pts = self._pts_counter * frame_duration
            buf.duration = frame_duration
            self._pts_counter += 1

            ret = self.appsrc.emit('push-buffer', buf)

            if ret != Gst.FlowReturn.OK:
                if ret in (Gst.FlowReturn.FLUSHING, Gst.FlowReturn.ERROR):
                    self._pipeline_ok = False
                continue

            self.frame_count += 1
            self.last_frame_time = now
            if self.frame_count % 300 == 0:
                elapsed = now - self.start_time
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                self.get_logger().info(
                    f'Streaming: {self.frame_count} frames, {fps:.1f} fps avg')

    # ---- Depth normalization ----

    def _normalize_depth_image(self, image, encoding):
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
        return cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)

    def cleanup(self):
        self.get_logger().info('Stopping video bridge...')
        self._encode_stop.set()
        self._frame_event.set()  # wake encoder thread
        self.stop_overlay()
        if self._encode_thread.is_alive():
            self._encode_thread.join(timeout=3)
        if self.pipeline is not None:
            self.appsrc.emit('end-of-stream')
            self.pipeline.get_bus().timed_pop_filtered(2 * Gst.SECOND, Gst.MessageType.EOS)
            self.pipeline.set_state(Gst.State.NULL)
        self.get_logger().info('Stopped')

    # ---- Detection overlay ----

    _LABEL_COLORS = {
        'red': (0, 0, 255), 'blue': (255, 140, 0), 'green': (0, 200, 0),
        'yellow': (0, 255, 255), 'orange': (0, 165, 255),
        'white': (255, 255, 255), 'black': (80, 80, 80),
    }
    _DEFAULT_COLOR = (0, 255, 0)

    def _color_for_label(self, label: str):
        ll = label.lower()
        for key, color in self._LABEL_COLORS.items():
            if key in ll:
                return color
        return self._DEFAULT_COLOR

    def draw_detections(self, frame):
        import cv2
        with self._detections_lock:
            detections = list(self._detections)
        if not detections:
            return

        h, w = frame.shape[:2]
        for i, det in enumerate(detections):
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

            sx = w / (det.get('_src_w', w) or w)
            sy = h / (det.get('_src_h', h) or h)
            x1, y1 = int(bx * sx), int(by * sy)
            x2, y2 = int((bx + bw) * sx), int((by + bh) * sy)

            label = str(det.get('label', 'unknown') or 'unknown')
            try:
                conf = float(det.get('confidence', 0.0) or 0.0)
            except (TypeError, ValueError):
                conf = 0.0
            color = self._color_for_label(label)

            # Draw ellipse for circle detections with fitted orientation if available
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            rx = (x2 - x1) // 2
            ry = (y2 - y1) // 2
            
            if 'circle' in label.lower() and rx > 0 and ry > 0:
                # Use ellipse fitting parameters if available, otherwise use bbox
                ellipse_w = det.get('ellipse_half_w')
                ellipse_h = det.get('ellipse_half_h')
                ellipse_angle = det.get('ellipse_angle', 0.0)
                
                if ellipse_w is not None and ellipse_h is not None:
                    # Scale ellipse parameters to frame size
                    ellipse_w = int(float(ellipse_w) * sx)
                    ellipse_h = int(float(ellipse_h) * sy)
                    ellipse_angle = float(ellipse_angle)
                    if ellipse_w > 0 and ellipse_h > 0:
                        cv2.ellipse(frame, (cx, cy), (ellipse_w, ellipse_h), ellipse_angle, 0, 360, color, 2)
                    else:
                        # Fallback to bbox-derived ellipse
                        cv2.ellipse(frame, (cx, cy), (rx, ry), 0, 0, 360, color, 2)
                else:
                    # Use bbox-derived ellipse
                    cv2.ellipse(frame, (cx, cy), (rx, ry), 0, 0, 360, color, 2)
                    
                cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 10, 1)
            else:
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            letter = chr(ord('A') + i) if i < 26 else str(i)
            hsv_color = det.get('hsv_color', '')
            display_color = hsv_color if hsv_color else label.replace('_circle', '')
            text = f"{letter}: {display_color} {conf:.0%}"
            (tw, th_), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ty = max(th_ + baseline + 4, y1)
            cv2.rectangle(frame, (x1, ty - th_ - baseline - 4), (x1 + tw + 4, ty), color, -1)
            cv2.putText(frame, text, (x1 + 2, ty - baseline - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        badge = f"ROS2: {len(detections)} target{'s' if len(detections) != 1 else ''}"
        cv2.putText(frame, badge, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

    def _fetch_detections_loop(self):
        while not self._overlay_stop.is_set():
            try:
                url = f"{self._edge_core_url}/api/detections"
                req = urllib.request.Request(url, method='GET')
                req.add_header('Accept', 'application/json')
                if self._edge_core_api_key:
                    req.add_header('X-API-Key', self._edge_core_api_key)
                if self._edge_core_internal_token:
                    req.add_header('X-NOMAD-Internal-Token', self._edge_core_internal_token)
                with urllib.request.urlopen(req, timeout=2) as resp:
                    data = json.loads(resp.read().decode())
                    current = data.get('current', {})
                    dets = current.get('detections', []) if isinstance(current, dict) else []
                    with self._detections_lock:
                        self._detections = dets
            except Exception as e:
                now = time.time()
                if now - self._overlay_last_error_log_ts > 5.0:
                    self.get_logger().warn(f"Overlay fetch failed: {e}")
                    self._overlay_last_error_log_ts = now
                with self._detections_lock:
                    self._detections = []
            self._overlay_stop.wait(1.0 / 15.0)  # Poll at 15 Hz to match video framerate

    def start_overlay(self):
        if self._overlay_enabled:
            return
        self._overlay_enabled = True
        self._overlay_stop.clear()
        self._overlay_thread = threading.Thread(
            target=self._fetch_detections_loop, daemon=True, name='overlay-fetch')
        self._overlay_thread.start()
        self.get_logger().info("Detection overlay enabled")

    def stop_overlay(self):
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
            self._send_json(200, {
                'streaming': receiving_frames,
                'pipeline_playing': pipeline_playing,
                'source_topic': self.video_node.source_topic if self.video_node else '',
                'fps': self.video_node.frame_count / max(elapsed, 1) if self.video_node else 0,
                'frame_count': self.video_node.frame_count if self.video_node else 0,
                'error_count': self.video_node.error_count if self.video_node else 0,
                'last_frame_age_s': last_frame_age,
                'rtsp_url': f'rtsp://localhost:8554/{self.video_node.rtsp_path}' if self.video_node else '',
                'encoder': self.video_node._encoder_name if self.video_node else '',
            })

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
                self._send_json(200, {'success': True, 'topic': new_topic})
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
                capture_output=True, text=True, timeout=10)
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
    ControlServer.video_node = video_node
    server = HTTPServer(('0.0.0.0', port), ControlServer)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    video_node.get_logger().info(f'HTTP control server on port {port}')
    return server


def main(args=None):
    import argparse
    import signal
    import sys

    parser = argparse.ArgumentParser(description='Simple Video Bridge for NOMAD')
    parser.add_argument('--source-topic', type=str,
                       default='/zed/zed_node/rgb/image_rect_color')
    parser.add_argument('--width', type=int, default=848)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--bitrate', type=int, default=1500,
                       help='H264 bitrate in kbps')
    parser.add_argument('--http-port', type=int, default=9200)
    parser.add_argument('--rtsp-path', type=str, default='primary')

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
