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
import math
import subprocess
import os
import json
import urllib.request
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from collections import deque


# HSV ranges mirror edge_core/target_localizer/target_localizer/detectors.py so
# the livestream overlay shows the same circles as a /capture_target call would.
# Keep these two files in sync when tuning.
_HSV_RANGES = {
    "red": [((0, 60, 60), (17, 255, 255)), ((165, 60, 60), (180, 255, 255))],
    "yellow": [((22, 90, 90), (38, 255, 255))],
    "green": [((40, 70, 60), (85, 255, 255))],
    "blue": [((95, 90, 70), (130, 255, 255))],
    "black": [((0, 0, 0), (180, 80, 60))],
    "white": [((0, 0, 180), (180, 40, 255))],
}
_HSV_PRIORITY = ["red", "yellow", "green", "blue", "black", "white"]

Gst.init(None)

# ZED 2 SDK 5.x topics that use lazy publishing — only published when someone subscribes.
# ros2 topic list won't show them until then, so we include them statically so
# the user can pre-select before ZED starts; subscribing triggers publication.
# Topic naming convention: /zed/zed_node/<side>/color/<raw|rect>/image
_ZED_KNOWN_IMAGE_TOPICS = [
    '/zed/zed_node/rgb/color/rect/image',
    '/zed/zed_node/rgb/color/raw/image',
    '/zed/zed_node/left/color/rect/image',
    '/zed/zed_node/left/color/raw/image',
    '/zed/zed_node/right/color/rect/image',
    '/zed/zed_node/right/color/raw/image',
    '/zed/zed_node/depth/depth_registered',
]


def _probe_encoder():
    """Probe available H.264 software encoder.

    Returns (element_name, pipeline_fragment_template). x264enc is strongly
    preferred — at 1080p30 openh264enc saturates the Orin Nano CPU and the
    pipeline stalls (no frames reach RTSP), while x264enc in
    ultrafast/zerolatency keeps up. Install with `apt install
    gstreamer1.0-plugins-ugly` if missing.
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


def _circles_iou(cx1: float, cy1: float, r1: float,
                 cx2: float, cy2: float, r2: float) -> float:
    """Intersection-over-Union of two circles. Returns 0.0 when disjoint."""
    d = ((cx1 - cx2) ** 2 + (cy1 - cy2) ** 2) ** 0.5
    if d >= r1 + r2:
        return 0.0
    if d <= abs(r1 - r2):
        rs = min(r1, r2); rl = max(r1, r2)
        return (rs * rs) / (rl * rl)
    r1s = r1 * r1; r2s = r2 * r2
    try:
        a1 = r1s * math.acos((d * d + r1s - r2s) / (2.0 * d * r1))
        a2 = r2s * math.acos((d * d + r2s - r1s) / (2.0 * d * r2))
        a3 = 0.5 * math.sqrt(
            (-d + r1 + r2) * (d + r1 - r2) * (d - r1 + r2) * (d + r1 + r2)
        )
    except (ValueError, ZeroDivisionError):
        return 0.0
    inter = a1 + a2 - a3
    union = math.pi * r1s + math.pi * r2s - inter
    if union <= 0.0:
        return 0.0
    return inter / union


class VideoStreamNode(Node):
    """ROS2 node that streams images to RTSP via GStreamer."""

    _MIN_RESTART_INTERVAL = 5.0

    def __init__(self, source_topic: str, width: int = 1920, height: int = 1080,
                 fps: int = 30, bitrate: int = 2500, rtsp_path: str = "primary"):
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
        self.dropped_count = 0
        self.throttled_count = 0
        self.push_fail_count = 0
        self.start_time = time.time()
        self.last_frame_time = 0.0
        self.subscription = None
        self._latest_jpeg = None
        self._last_snapshot_encode_time = 0.0
        self._latest_raw_jpeg = None
        self._last_raw_snapshot_encode_time = 0.0
        self._raw_snapshot_interval = float(os.environ.get(
            "NOMAD_RAW_SNAPSHOT_INTERVAL", "2.0"
        ))

        # --- Threaded encoder: ROS callback drops frame into _pending_frame,
        # encoder thread picks it up at its own pace. ---
        self._pending_frame = None  # numpy array or None
        self._frame_lock = threading.Lock()
        self._frame_event = threading.Event()
        self._encode_stop = threading.Event()
        self._frame_timestamps = deque(maxlen=max(120, fps * 8))
        self._stats_lock = threading.Lock()
        self._perf_stats = {
            "callback_count": 0,
            "callback_total_ms": 0.0,
            "callback_max_ms": 0.0,
            "convert_total_ms": 0.0,
            "resize_total_ms": 0.0,
            "overlay_total_ms": 0.0,
            "enqueue_total_ms": 0.0,
            "encode_count": 0,
            "wrap_total_ms": 0.0,
            "push_total_ms": 0.0,
            "push_max_ms": 0.0,
        }

        # Frame pacing
        self._min_frame_interval = 1.0 / fps
        self._last_push_time = 0.0
        self._pts_counter = 0

        # Pipeline recovery
        self._pipeline_ok = True
        self._last_restart_time = 0.0
        self._restart_lock = threading.Lock()

        # Detection overlay — two independent detectors:
        #   task1: color HSV circle detector (Task 1 reconnaissance)
        #   task2: color-agnostic shape circle detector (Task 2 spray targets)
        # Either or both can run at the same time. The legacy
        # `_overlay_enabled` flag mirrors "any detector on" for the existing
        # /overlay/enable + /overlay/disable APIs.
        self._overlay_task1 = False
        self._overlay_task2 = False
        # Mirrors "any detector on" so the old /overlay/enable API and
        # frame-loop guard keep working unchanged.
        self._overlay_enabled = False
        # Legacy mode hint — last detector explicitly switched to via /overlay/mode.
        self._overlay_mode = "task1"
        self._detections = []
        self._detections_lock = threading.Lock()
        # Local HSV circle detection runs inline at ~5 Hz so the livestream
        # shows the same boxes as /capture_target without a network round-trip.
        self._hsv_last_run_ts = 0.0
        self._hsv_min_interval = float(os.environ.get(
            "NOMAD_DETECTOR_INTERVAL_S", "0.5"
        ))
        # 0 keeps detection at source resolution. For Task 2 we want the
        # native HD720 frame for reliable paper-circle detection; CPU is saved
        # by avoiding BGRA churn and by using the color/backing detector.
        self._detector_max_width = int(os.environ.get(
            "NOMAD_DETECTOR_MAX_WIDTH", "0"
        ))
        self._detector_stop = threading.Event()
        self._detector_frame_lock = threading.Lock()
        self._detector_frame = None
        self._detector_frame_ts = 0.0
        self._detector_last_frame_store_ts = 0.0
        self._task2_cuda_checked = False
        self._task2_cuda_enabled = False
        self._task2_cuda_gray_filter = None
        self._edge_core_url = "http://172.17.0.1:8000"
        self._edge_core_api_key = (os.environ.get("NOMAD_API_KEY") or "").strip()
        self._edge_core_internal_token = (
            os.environ.get("NOMAD_INTERNAL_TOKEN") or ""
        ).strip()
        self._overlay_thread = None
        self._overlay_stop = threading.Event()
        self._overlay_last_error_log_ts = 0.0

        # Latest depth frame (Task 2 shape detector range lookup). Stored at
        # native ZED resolution; circle centers are scaled in from the
        # streaming frame size at sample time.
        self._latest_depth = None  # numpy float32 in meters
        self._latest_depth_shape = None  # (h, w) of the depth frame
        self._latest_depth_ts = 0.0
        self._depth_lock = threading.Lock()
        self._depth_topic = os.environ.get(
            "NOMAD_TASK2_DEPTH_TOPIC", "/zed/zed_node/depth/depth_registered"
        )
        self._depth_sub = None

        # Probe encoder
        self._encoder_name, self._encoder_fragment = _probe_encoder()
        self.get_logger().info(f'Using encoder: {self._encoder_name}')

        # Build and start pipeline
        self.pipeline = None
        self.appsrc = None
        self._build_and_start_pipeline()

        # Subscribe to ROS2
        self._subscribe_to_topic(source_topic)
        self._subscribe_depth(self._depth_topic)

        # Start encoder thread (picks frames from _pending_frame, pushes to GStreamer)
        self._encode_thread = threading.Thread(target=self._encode_loop, daemon=True,
                                               name='encode-loop')
        self._encode_thread.start()

        # Start bus watchdog
        self._bus_thread = threading.Thread(target=self._bus_watch_loop, daemon=True)
        self._bus_thread.start()

        # Detection worker. The ROS image callback only publishes the latest
        # full-resolution frame to this worker, so expensive target detection
        # cannot stall the RTSP stream.
        self._detector_thread = threading.Thread(
            target=self._detector_loop, daemon=True, name='detector-loop'
        )
        self._detector_thread.start()

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
            f'video/x-raw,format=I420,width={width},height={height},framerate={fps}/1 ! '
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
            # Camera streams should use sensor-style QoS. This avoids DDS
            # delivery stalls seen with ZED image topics after wrapper restarts.
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            Image, topic, self.image_callback, image_qos)
        self.source_topic = topic
        self.get_logger().info(f'Subscribed to: {topic}')

    def _subscribe_depth(self, topic: str):
        """Subscribe to the ZED depth topic so the Task 2 shape detector can
        attach a per-circle range. Best-effort — failures are non-fatal; the
        detections just won't carry range_m and the spray controller will
        keep treating Task 2 picks as image_only.
        """
        try:
            depth_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            if self._depth_sub is not None:
                self.destroy_subscription(self._depth_sub)
                self._depth_sub = None
            self._depth_sub = self.create_subscription(
                Image, topic, self._depth_callback, depth_qos
            )
            self.get_logger().info(f'Subscribed to depth: {topic}')
        except Exception as e:
            self.get_logger().warn(f'Depth subscription failed ({topic}): {e}')

    def _depth_callback(self, msg: Image):
        """Cache the latest depth frame as float32 meters."""
        try:
            enc = msg.encoding.lower()
            if enc == '32fc1':
                arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                depth = arr.astype(np.float32, copy=False)
            elif enc in ('16uc1', 'mono16'):
                arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                depth = arr.astype(np.float32) * 0.001  # mm -> m
            else:
                return
            with self._depth_lock:
                self._latest_depth = depth
                self._latest_depth_shape = depth.shape[:2]
                self._latest_depth_ts = time.time()
        except Exception as e:
            if self.frame_count % 300 == 0:
                self.get_logger().warn(f'Depth callback error: {e}')

    def _sample_depth_at(self, px_stream: float, py_stream: float,
                         stream_w: int, stream_h: int) -> "float | None":
        """Sample median depth (meters) in a small ROI around (px, py).

        Inputs are in stream-frame pixels (after the bridge's resize); they
        are rescaled into the native depth frame before sampling. Returns
        None when no valid pixels are available or no depth is yet cached.
        """
        with self._depth_lock:
            depth = self._latest_depth
            dshape = self._latest_depth_shape
            ts = self._latest_depth_ts
        if depth is None or dshape is None:
            return None
        # Stale depth (>2s) is worse than no depth.
        if time.time() - ts > 2.0:
            return None
        dh, dw = dshape
        if stream_w <= 0 or stream_h <= 0:
            return None
        sx = dw / float(stream_w)
        sy = dh / float(stream_h)
        cx = int(round(px_stream * sx))
        cy = int(round(py_stream * sy))
        if not (0 <= cx < dw and 0 <= cy < dh):
            return None
        for half in (5, 12, 20, 30):
            x1 = max(0, cx - half); x2 = min(dw, cx + half + 1)
            y1 = max(0, cy - half); y2 = min(dh, cy + half + 1)
            roi = depth[y1:y2, x1:x2]
            valid = roi[np.isfinite(roi) & (roi > 0.1) & (roi < 35.0)]
            if valid.size >= 8:
                return float(np.median(valid))
        return None

    def _estimate_task2_diameter_m(self, bbox_w: float, bbox_h: float,
                                   range_m: "float | None",
                                   frame_w: int, frame_h: int) -> "float | None":
        """Estimate physical target diameter from pixel size and ZED range."""
        if range_m is None or range_m <= 0.0 or frame_w <= 0 or frame_h <= 0:
            return None
        hfov_deg = float(os.environ.get("NOMAD_TASK2_HFOV_DEG", "110.0"))
        vfov_deg = float(os.environ.get("NOMAD_TASK2_VFOV_DEG", "70.0"))
        angular_w = math.radians(hfov_deg) * (bbox_w / float(frame_w))
        angular_h = math.radians(vfov_deg) * (bbox_h / float(frame_h))
        diam_w = 2.0 * range_m * math.tan(max(angular_w, 0.0) * 0.5)
        diam_h = 2.0 * range_m * math.tan(max(angular_h, 0.0) * 0.5)
        return float((diam_w + diam_h) * 0.5)

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
            self.throttled_count += 1
            return

        try:
            cb_start = time.perf_counter()
            convert_start = cb_start
            cv2 = None

            encoding = msg.encoding.lower()
            if encoding in ('bgr8', '8uc3'):
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            elif encoding in ('bgra8', '8uc4'):
                import cv2
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            elif encoding == 'rgba8':
                import cv2
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2BGR)
            elif encoding == 'rgb8':
                import cv2
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            elif encoding in ('16uc1', 'mono16', '32fc1'):
                raw_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                if self.frame_count % 60 == 0:
                    finite = np.isfinite(raw_depth) & (raw_depth > 0)
                    n_valid = int(finite.sum())
                    if n_valid > 0:
                        vmin = float(raw_depth[finite].min())
                        vmax = float(raw_depth[finite].max())
                    else:
                        vmin = vmax = 0.0
                    self.get_logger().info(
                        f'Depth frame #{self.frame_count}: enc={encoding} '
                        f'shape={raw_depth.shape} valid={n_valid}/{raw_depth.size} '
                        f'range=[{vmin:.2f}, {vmax:.2f}]'
                    )
                cv_image = self._normalize_depth_image(raw_depth, encoding)
            elif encoding in ('mono8', '8uc1'):
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                import cv2
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            convert_ms = (time.perf_counter() - convert_start) * 1000.0

            if (now - self._last_raw_snapshot_encode_time) >= self._raw_snapshot_interval:
                try:
                    if cv2 is None:
                        import cv2
                    ok, jpeg = cv2.imencode(
                        '.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 78]
                    )
                    if ok:
                        self._latest_raw_jpeg = jpeg.tobytes()
                        self._last_raw_snapshot_encode_time = now
                except Exception:
                    pass

            overlay_ms = 0.0
            if self._overlay_enabled and (now - self._hsv_last_run_ts) >= self._hsv_min_interval:
                if cv2 is None:
                    import cv2
                overlay_start = time.perf_counter()
                self._hsv_last_run_ts = now
                detection_frame = cv_image
                if (self._detector_max_width > 0
                        and detection_frame.shape[1] > self._detector_max_width):
                    scale = self._detector_max_width / float(detection_frame.shape[1])
                    det_h = max(1, int(round(detection_frame.shape[0] * scale)))
                    detection_frame = cv2.resize(
                        detection_frame,
                        (self._detector_max_width, det_h),
                        interpolation=cv2.INTER_AREA,
                    )
                else:
                    detection_frame = detection_frame.copy()
                with self._detector_frame_lock:
                    self._detector_frame = detection_frame
                    self._detector_frame_ts = now
                overlay_ms += (time.perf_counter() - overlay_start) * 1000.0

            # Resize to target (this is fast if already the right size)
            resize_ms = 0.0
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                if cv2 is None:
                    import cv2
                resize_start = time.perf_counter()
                # INTER_AREA for downscale: best anti-aliasing, preserves edge
                # gradients that HoughCircles/contour-circularity rely on.
                # INTER_LINEAR if upscaling.
                interp = (cv2.INTER_AREA
                          if cv_image.shape[1] > self.width
                          else cv2.INTER_LINEAR)
                cv_image = cv2.resize(cv_image, (self.width, self.height),
                                      interpolation=interp)
                resize_ms = (time.perf_counter() - resize_start) * 1000.0

            # Draw cached detections on the downscaled Mission Planner stream.
            if self._overlay_enabled:
                if cv2 is None:
                    import cv2
                overlay_start = time.perf_counter()
                self.draw_detections(cv_image)
                overlay_ms += (time.perf_counter() - overlay_start) * 1000.0

            # Ensure contiguous C-order for zero-copy tobytes
            if not cv_image.flags['C_CONTIGUOUS']:
                cv_image = np.ascontiguousarray(cv_image)

            # Drop frame into single slot for encoder thread
            enqueue_start = time.perf_counter()
            with self._frame_lock:
                if self._pending_frame is not None:
                    self.dropped_count += 1
                self._pending_frame = cv_image
            self._frame_event.set()
            self._last_push_time = now
            enqueue_ms = (time.perf_counter() - enqueue_start) * 1000.0
            callback_ms = (time.perf_counter() - cb_start) * 1000.0
            self._record_callback_perf(
                callback_ms, convert_ms, resize_ms, overlay_ms, enqueue_ms
            )

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
            with self._frame_lock:
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
            wrap_start = time.perf_counter()
            data = frame.tobytes()
            buf = Gst.Buffer.new_wrapped(data)
            buf.pts = self._pts_counter * frame_duration
            buf.dts = buf.pts
            buf.duration = frame_duration
            self._pts_counter += 1
            wrap_ms = (time.perf_counter() - wrap_start) * 1000.0

            push_start = time.perf_counter()
            ret = self.appsrc.emit('push-buffer', buf)
            push_ms = (time.perf_counter() - push_start) * 1000.0
            self._record_encode_perf(wrap_ms, push_ms)

            if ret != Gst.FlowReturn.OK:
                self.push_fail_count += 1
                if ret in (Gst.FlowReturn.FLUSHING, Gst.FlowReturn.ERROR):
                    self._pipeline_ok = False
                continue

            self.frame_count += 1
            self.last_frame_time = now
            with self._stats_lock:
                self._frame_timestamps.append(now)
            if self.frame_count % 300 == 0:
                fps = self._current_fps()
                self.get_logger().info(
                    f'Streaming: {self.frame_count} frames, {fps:.1f} fps current')

    def _record_callback_perf(self, callback_ms: float, convert_ms: float,
                              resize_ms: float, overlay_ms: float,
                              enqueue_ms: float) -> None:
        with self._stats_lock:
            self._perf_stats["callback_count"] += 1
            self._perf_stats["callback_total_ms"] += callback_ms
            self._perf_stats["callback_max_ms"] = max(
                self._perf_stats["callback_max_ms"], callback_ms
            )
            self._perf_stats["convert_total_ms"] += convert_ms
            self._perf_stats["resize_total_ms"] += resize_ms
            self._perf_stats["overlay_total_ms"] += overlay_ms
            self._perf_stats["enqueue_total_ms"] += enqueue_ms

    def _record_encode_perf(self, wrap_ms: float, push_ms: float) -> None:
        with self._stats_lock:
            self._perf_stats["encode_count"] += 1
            self._perf_stats["wrap_total_ms"] += wrap_ms
            self._perf_stats["push_total_ms"] += push_ms
            self._perf_stats["push_max_ms"] = max(
                self._perf_stats["push_max_ms"], push_ms
            )

    def _current_fps(self) -> float:
        with self._stats_lock:
            times = list(self._frame_timestamps)
        if len(times) < 2:
            return 0.0
        elapsed = times[-1] - times[0]
        if elapsed <= 0:
            return 0.0
        return (len(times) - 1) / elapsed

    def perf_snapshot(self) -> dict:
        with self._stats_lock:
            stats = dict(self._perf_stats)
        cb_count = max(int(stats["callback_count"]), 1)
        enc_count = max(int(stats["encode_count"]), 1)
        return {
            "callback_avg_ms": stats["callback_total_ms"] / cb_count,
            "callback_max_ms": stats["callback_max_ms"],
            "convert_avg_ms": stats["convert_total_ms"] / cb_count,
            "resize_avg_ms": stats["resize_total_ms"] / cb_count,
            "overlay_avg_ms": stats["overlay_total_ms"] / cb_count,
            "enqueue_avg_ms": stats["enqueue_total_ms"] / cb_count,
            "wrap_avg_ms": stats["wrap_total_ms"] / enc_count,
            "push_avg_ms": stats["push_total_ms"] / enc_count,
            "push_max_ms": stats["push_max_ms"],
            "callback_count": stats["callback_count"],
            "encode_count": stats["encode_count"],
        }

    def _detector_loop(self) -> None:
        """Run local circle detectors from the latest full-resolution frame."""
        while not self._detector_stop.is_set():
            self._detector_stop.wait(self._hsv_min_interval)
            if self._detector_stop.is_set():
                break
            if not self._overlay_enabled:
                continue

            with self._detector_frame_lock:
                frame = self._detector_frame
                frame_ts = self._detector_frame_ts
                self._detector_frame = None

            if frame is None:
                continue
            if time.time() - frame_ts > 2.0:
                continue

            dets = []
            if self._overlay_task1:
                try:
                    dets.extend(self._detect_hsv_circles(frame))
                except Exception as det_err:
                    self.get_logger().warn(f'HSV detect error: {det_err}')
            if self._overlay_task2:
                try:
                    dets.extend(self._detect_task2_circles(frame))
                except Exception as det_err:
                    self.get_logger().warn(f'Task 2 color detect error: {det_err}')
            dets = self._dedupe_detections(dets)
            with self._detections_lock:
                self._detections = dets

    # ---- Depth normalization ----

    def _normalize_depth_image(self, image, encoding):
        """Convert a raw depth frame into a colorized BGR image for streaming.

        ZED publishes depth as 32FC1 in meters. Some other drivers publish
        16UC1 in millimeters; we convert those to meters so the visible range
        in the legend (and percentile thresholds) is consistent.
        """
        import cv2
        if encoding == '32fc1':
            img = image.astype(np.float32, copy=True)
        else:
            # 16UC1 / mono16 — assume millimeters, convert to meters.
            img = image.astype(np.float32) * 0.001

        valid_mask = np.isfinite(img) & (img > 0.1) & (img < 35.0)
        if not valid_mask.any():
            # Solid dark frame so VLC knows there's still video flowing.
            return np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
        min_val = float(np.percentile(img[valid_mask], 2))
        max_val = float(np.percentile(img[valid_mask], 98))
        if max_val <= min_val:
            max_val = min_val + 1.0
        normalized = np.clip((img - min_val) / (max_val - min_val) * 255.0, 0, 255).astype(np.uint8)
        normalized[~valid_mask] = 0
        return cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)

    def cleanup(self):
        self.get_logger().info('Stopping video bridge...')
        self._detector_stop.set()
        self._encode_stop.set()
        self._frame_event.set()  # wake encoder thread
        self.stop_overlay()
        if getattr(self, '_detector_thread', None) and self._detector_thread.is_alive():
            self._detector_thread.join(timeout=2)
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
                # Prefer the fitted ellipse (rotated) when the Task 2 detector
                # provided one; otherwise fall back to the axis-aligned bbox.
                emaj = det.get('ellipse_major_px')
                emin = det.get('ellipse_minor_px')
                eang = det.get('ellipse_angle_deg')
                ecx_v = det.get('cx')
                ecy_v = det.get('cy')
                drew = False
                try:
                    if (emaj is not None and emin is not None and eang is not None
                            and ecx_v is not None and ecy_v is not None):
                        emaj_px = int(round(float(emaj) * sx))
                        emin_px = int(round(float(emin) * sy))
                        ecx_px = int(round(float(ecx_v) * sx))
                        ecy_px = int(round(float(ecy_v) * sy))
                        if emaj_px > 0 and emin_px > 0:
                            cv2.ellipse(
                                frame, (ecx_px, ecy_px),
                                (emaj_px, emin_px), float(eang),
                                0, 360, color, 2,
                            )
                            cv2.drawMarker(
                                frame, (ecx_px, ecy_px), color,
                                cv2.MARKER_CROSS, 10, 1,
                            )
                            cx, cy = ecx_px, ecy_px
                            drew = True
                except (TypeError, ValueError):
                    drew = False
                if not drew:
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

        # Label the badge with the active detector(s) so the operator can tell
        # at a glance which task's overlay is currently drawing.
        if self._overlay_task1 and self._overlay_task2:
            mode_label = "Task 1+2"
        elif self._overlay_task2:
            mode_label = "Task 2"
        elif self._overlay_task1:
            mode_label = "Task 1"
        else:
            mode_label = "Idle"
        badge = f"{mode_label}: {len(detections)} target{'s' if len(detections) != 1 else ''}"
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

    def _detect_hsv_circles(self, frame):
        """Local HSV circle detector mirroring target_localizer/detectors.py.

        Populates self._detections with dicts shaped for draw_detections:
        bbox_x/y/w/h, label, confidence, hsv_color.
        """
        import cv2
        h, w = frame.shape[:2]
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        out = []
        claimed_centers = []
        # At 848x480 stream res, a 5cm target at 10m ≈ 10px radius, at 5m ≈ 20px.
        # 12px floor keeps the smallest legitimate targets while rejecting the
        # distant noise blobs that triggered false positives in indoor tests.
        min_r = 12
        max_r = max(40, min(h, w) // 2)
        min_area = math.pi * min_r * min_r
        max_area = math.pi * max_r * max_r

        for color_name in _HSV_PRIORITY:
            mask = np.zeros((h, w), dtype=np.uint8)
            for lo, hi in _HSV_RANGES[color_name]:
                mask |= cv2.inRange(hsv, np.array(lo, dtype=np.uint8),
                                    np.array(hi, dtype=np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < min_area or area > max_area:
                    continue
                perim = cv2.arcLength(c, True)
                if perim < 1.0:
                    continue
                circ = (4.0 * math.pi * area) / (perim * perim)
                if circ < 0.70:
                    continue
                hull = cv2.convexHull(c)
                hull_area = cv2.contourArea(hull)
                if hull_area < 1.0 or (area / hull_area) < 0.80:
                    continue
                (cx_f, cy_f), radius = cv2.minEnclosingCircle(c)
                cx, cy, r = int(cx_f), int(cy_f), int(radius)
                if r < min_r or r > max_r:
                    continue
                # Reject overlaps with higher-priority detections.
                if any(((cx - ex) ** 2 + (cy - ey) ** 2) ** 0.5 < max(r, er) * 0.7
                       for ex, ey, er in claimed_centers):
                    continue
                claimed_centers.append((cx, cy, r))
                x, y, bw, bh = cv2.boundingRect(c)
                out.append({
                    "label": f"{color_name}_circle",
                    "hsv_color": color_name,
                    "confidence": float(min(circ, area / hull_area)),
                    "bbox_x": float(x),
                    "bbox_y": float(y),
                    "bbox_w": float(bw),
                    "bbox_h": float(bh),
                    "_src_w": w,
                    "_src_h": h,
                    "_detector": "task1",
                })
        return out

    def _prepare_task2_detection_images(self, frame, cv2):
        """Return grayscale and HSV images for Task 2 shape detection.

        Uses OpenCV CUDA for the color conversions and grayscale smoothing when
        available in the Isaac ROS container. HoughCircles and contour finding
        still run on CPU because OpenCV's Python CUDA bindings do not provide a
        drop-in Hough circle / findContours path in the build we provision.
        """
        use_cuda = os.environ.get("NOMAD_TASK2_USE_CUDA", "true").lower() not in (
            "0", "false", "no", "off"
        )
        if use_cuda:
            if not self._task2_cuda_checked:
                self._task2_cuda_checked = True
                try:
                    count = cv2.cuda.getCudaEnabledDeviceCount()
                    if count > 0:
                        cv2.cuda.setDevice(0)
                        self._task2_cuda_gray_filter = cv2.cuda.createMedianFilter(
                            cv2.CV_8UC1, 5
                        )
                        self._task2_cuda_enabled = True
                        self.get_logger().info("Task 2 detector using OpenCV CUDA preprocess")
                    else:
                        self.get_logger().warn("Task 2 CUDA requested but cv2 sees no CUDA device")
                except Exception as exc:
                    self.get_logger().warn(f"Task 2 CUDA preprocess unavailable: {exc}")

            if self._task2_cuda_enabled:
                try:
                    gpu_bgr = cv2.cuda_GpuMat()
                    gpu_bgr.upload(frame)
                    gpu_gray = cv2.cuda.cvtColor(gpu_bgr, cv2.COLOR_BGR2GRAY)
                    if self._task2_cuda_gray_filter is not None:
                        gpu_gray = self._task2_cuda_gray_filter.apply(gpu_gray)
                    gpu_hsv = cv2.cuda.cvtColor(gpu_bgr, cv2.COLOR_BGR2HSV)
                    return gpu_gray.download(), gpu_hsv.download()
                except Exception as exc:
                    self.get_logger().warn(f"Task 2 CUDA preprocess failed; using CPU: {exc}")
                    self._task2_cuda_enabled = False

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        return gray, hsv

    def _detect_task2_circles(self, frame):
        """Task 2 detector entry point. Single canonical implementation."""
        return self._detect_shape_circles(frame)

    def _task2_debug_reject(self, reason, value, cx_i, cy_i, radius_px):
        """Log a near-miss rejection when NOMAD_TASK2_DEBUG is enabled.

        Cheap helper so we can tell at-a-glance which gate is rejecting a
        candidate the operator expected to detect. Off by default to keep
        the log clean in normal flight.
        """
        if os.environ.get("NOMAD_TASK2_DEBUG", "0").lower() not in (
            "1", "true", "yes", "on"
        ):
            return
        try:
            self.get_logger().info(
                f"task2_reject {reason} @ ({cx_i},{cy_i}) r={int(radius_px)} "
                f"value={value}"
            )
        except Exception:
            pass

    def _detect_shape_circles(self, frame):
        """Task 2 red-cabbage-paper detector -- Hough+density pipeline.

        Approach (after the connected-component pipeline kept fragmenting
        textured paper into pieces that all failed the shape gates):
          1. Build a strict cabbage-paper colour mask (mauve OR blue, with
             a b* < 135 skin-tone reject). KEEP this raw -- no morphology --
             so its interior-density is a faithful colour signal.
          2. Build the white-plastic backing mask (5x5 close + 5x5 open).
          3. Run HoughCircles on a median-blurred grayscale. Hough finds the
             disk's geometric boundary even when the colour mask is shattered
             by paper texture.
          4. For each Hough candidate, gate by:
               * interior colour density >= 0.22 (kills circles on plain
                 surfaces -- pipe ends, wall textures, finger knuckles)
               * skin-pixel ratio < 0.30 (belt-and-suspenders skin reject)
               * frame-edge margin
               * ZED depth + physical diameter (4-35 cm) when depth present
          5. Backing-ring ratio is a soft confidence booster, not a reject.
          6. NMS by centre-distance.

        Outputs the same dict shape as the old CC pipeline so the overlay
        renderer and spray controller see no breaking change.
        """
        import cv2
        h, w = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        s_ch = hsv[:, :, 1]
        v_ch = hsv[:, :, 2]
        a_ch = lab[:, :, 1].astype(np.int16)
        b_ch = lab[:, :, 2].astype(np.int16)
        chroma = np.abs(a_ch - 128) + np.abs(b_ch - 128)

        # Calibrated against actual cabbage-paper samples:
        #   real dyed paper:        a*=146-157, b*=86-118, chroma=28-71
        #   white paper/plastic:    a*=128-130, b*=128,    chroma=3-5
        #   gray PVC pipe (cool):   a*~128,     b*~125,    chroma=4-10
        # Setting chroma>=12 + a*>134 puts the gate squarely between real
        # cabbage paper and the worst off-white false positives.
        mauve = (
            (a_ch > 134) & (b_ch < 132) & (chroma >= 12)
            & (v_ch >= 70) & (v_ch <= 245)
        )
        # Post-spray cabbage reaction is a saturated teal/cyan -- never a
        # slightly-cool gray. a*<124 + chroma>=15 keeps the genuine pH-shift
        # response while excluding gray plastic with cool lighting cast.
        blue = (
            (a_ch < 124) & (b_ch < 128) & (chroma >= 15)
            & (v_ch >= 60) & (v_ch <= 230)
        )
        not_white = ~((s_ch <= 25) & (v_ch >= 195))
        target = ((mauve | blue) & not_white).astype(np.uint8)
        # Skin-tone (warm yellow-red corner): a* high AND b* high.
        skin = ((a_ch > 130) & (b_ch > 135) & (chroma >= 6)).astype(np.uint8)

        backing_raw = (
            (s_ch <= 40) & (v_ch >= 175) & (chroma <= 22)
        ).astype(np.uint8) * 255
        kernel5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        backing_mask = cv2.morphologyEx(
            backing_raw, cv2.MORPH_CLOSE, kernel5, iterations=2
        )
        backing_mask = cv2.morphologyEx(
            backing_mask, cv2.MORPH_OPEN, kernel5, iterations=1
        )
        backing_bool = backing_mask > 0

        # Hough on the CHROMA channel, not grayscale. The dyed paper has a
        # sharp chroma boundary against the white-plastic backing (chroma 28
        # -> chroma 3), while the backing-vs-wall boundary -- the dominant
        # gradient in grayscale -- is invisible to chroma. This stops Hough
        # from fitting oversized circles around the paper sheet that then
        # fail the colored-density check.
        chroma_u8 = np.clip(chroma, 0, 80).astype(np.uint8) * 3  # boost
        chroma_u8 = cv2.medianBlur(chroma_u8, 5)
        min_r = max(8, int(round(min(h, w) * 0.015)))
        max_r = min(h, w) // 2

        try:
            circles_raw = cv2.HoughCircles(
                chroma_u8,
                cv2.HOUGH_GRADIENT,
                dp=1.2,
                minDist=max(20, min_r * 2),
                param1=80,
                param2=30,
                minRadius=min_r,
                maxRadius=max_r,
            )
        except cv2.error as exc:
            try:
                self.get_logger().warn(f"HoughCircles failed: {exc}")
            except Exception:
                pass
            return []

        if circles_raw is None:
            return []

        yy, xx = np.ogrid[:h, :w]
        candidates = []
        for ccx, ccy, cr in circles_raw[0]:
            cx_i = int(ccx); cy_i = int(ccy); r_i = int(cr)
            if not (0 <= cx_i < w and 0 <= cy_i < h):
                continue
            if r_i < min_r or r_i > max_r:
                self._task2_debug_reject("radius_px", (r_i, min_r, max_r), cx_i, cy_i, r_i)
                continue
            if (cx_i - r_i) < 2 or (cy_i - r_i) < 2 or (cx_i + r_i) >= (w - 2) or (cy_i + r_i) >= (h - 2):
                self._task2_debug_reject("edge_clipped", (cx_i, cy_i, r_i, w, h), cx_i, cy_i, r_i)
                continue

            d2 = (xx - cx_i) ** 2 + (yy - cy_i) ** 2
            inside = d2 <= (r_i * 0.85) ** 2
            ic = int(inside.sum())
            if ic <= 0:
                continue
            t_inside = int((target & inside).sum())
            color_density = t_inside / ic
            if color_density < 0.22:
                self._task2_debug_reject("color_density", color_density, cx_i, cy_i, r_i)
                continue
            # Absolute floor on coloured-pixel count: kills tiny corner/edge
            # false positives where a small region (~30 px) happens to have
            # high relative density due to JPEG chromatic cast in shadows.
            # Real disks at worst-case range (5cm @ 5m -> r~7px is unrealistic;
            # 25cm @ 5m -> r~30px, ~2500 interior px @ 30% density = 750)
            # clear this easily.
            if t_inside < 200:
                self._task2_debug_reject("min_color_pixels", t_inside, cx_i, cy_i, r_i)
                continue

            sk_inside = int((skin & inside).sum())
            skin_ratio = sk_inside / ic
            if skin_ratio > 0.30:
                self._task2_debug_reject("skin_ratio", skin_ratio, cx_i, cy_i, r_i)
                continue

            ring = (d2 >= (r_i * 1.10) ** 2) & (d2 <= (r_i * 1.85) ** 2)
            rc = int(ring.sum())
            backing_ratio = 0.0
            if rc > 0:
                rb = int((backing_bool & ring).sum())
                backing_ratio = rb / rc

            rng = self._sample_depth_at(float(cx_i), float(cy_i), w, h)
            diameter_m = None
            if rng is not None:
                diameter_m = self._estimate_task2_diameter_m(
                    float(r_i * 2), float(r_i * 2), rng, w, h
                )
                if diameter_m is not None and (diameter_m < 0.035 or diameter_m > 0.40):
                    self._task2_debug_reject("diameter_m", diameter_m, cx_i, cy_i, r_i)
                    continue

            # Size factor: rewards real targets (r >= 60 px) over slivers.
            # Saturates at r=60 so very large disks don't completely
            # dominate over a mid-range disk that's still a real target.
            size_factor = min(r_i / 60.0, 1.0)
            confidence = min(
                0.99,
                0.10
                + min(color_density, 0.65) * 0.35
                + min(backing_ratio, 0.70) * 0.25
                + size_factor * 0.20
                + (0.10 if rng is not None else 0.0),
            )

            if os.environ.get("NOMAD_TASK2_DEBUG", "0").lower() in (
                "1", "true", "yes", "on"
            ):
                try:
                    self.get_logger().info(
                        f"task2_accept @ ({cx_i},{cy_i}) r={r_i} "
                        f"conf={confidence:.2f} density={color_density:.2f} "
                        f"backing={backing_ratio:.2f} skin={skin_ratio:.2f} "
                        f"rng={rng}"
                    )
                except Exception:
                    pass

            candidates.append({
                "label": "circle",
                "hsv_color": "cabbage_paper",
                "confidence": float(confidence),
                "bbox_x": float(max(0, cx_i - r_i)),
                "bbox_y": float(max(0, cy_i - r_i)),
                "bbox_w": float(min(w - max(0, cx_i - r_i), r_i * 2)),
                "bbox_h": float(min(h - max(0, cy_i - r_i), r_i * 2)),
                "cx": float(cx_i),
                "cy": float(cy_i),
                "ellipse_major_px": float(r_i),
                "ellipse_minor_px": float(r_i),
                "ellipse_angle_deg": 0.0,
                "radius_px": float(r_i),
                "_src_w": w,
                "_src_h": h,
                "_detector": "task2",
                "_method": "hough_density_v3",
                "range_m": rng,
                "diameter_m": diameter_m,
                "has_backing": bool(backing_ratio >= 0.15),
                "backing_ratio": float(backing_ratio),
                "color_density": float(color_density),
            })

        # IoU-based NMS: Hough often returns several overlapping sub-circles
        # inside one real disk because the paper texture creates secondary
        # gradients. Suppress any circle with significant area overlap with
        # an already-accepted higher-confidence one.
        candidates.sort(key=lambda d: -float(d["confidence"]))
        out = []
        for cand in candidates:
            keep = True
            for k in out:
                iou = _circles_iou(
                    cand["cx"], cand["cy"], cand["radius_px"],
                    k["cx"], k["cy"], k["radius_px"],
                )
                if iou > 0.25:
                    keep = False
                    break
            if keep:
                out.append(cand)
            if len(out) >= 6:
                break
        return out


    def set_overlay_mode(self, mode: str) -> bool:
        """Legacy mode setter: "task1" turns on T1 only; "task2" turns on T2 only."""
        normalized = (mode or "").strip().lower()
        if normalized == "task1":
            return self.set_overlay_detectors(task1=True, task2=False)
        if normalized == "task2":
            return self.set_overlay_detectors(task1=False, task2=True)
        return False

    def set_overlay_detectors(self, task1: bool, task2: bool) -> bool:
        """Independently enable/disable the Task 1 and Task 2 detectors.

        Idempotent: repeated state pushes (from the periodic status poll) that
        don't actually change the detector mask are no-ops, so they don't
        clear the detection list — which was producing brief overlay flicker.
        """
        new_t1, new_t2 = bool(task1), bool(task2)
        if new_t1 == self._overlay_task1 and new_t2 == self._overlay_task2:
            return True
        self._overlay_task1 = new_t1
        self._overlay_task2 = new_t2
        any_on = self._overlay_task1 or self._overlay_task2
        self._overlay_enabled = any_on
        if self._overlay_task1 and self._overlay_task2:
            self._overlay_mode = "both"
        elif self._overlay_task1:
            self._overlay_mode = "task1"
        elif self._overlay_task2:
            self._overlay_mode = "task2"
        else:
            self._overlay_mode = "none"
        with self._detections_lock:
            self._detections = []
        self._hsv_last_run_ts = 0.0
        self.get_logger().info(
            f"Overlay detectors: task1={self._overlay_task1} task2={self._overlay_task2}"
        )
        return True

    @staticmethod
    def _dedupe_detections(dets):
        """Spatially dedupe across detector passes.

        Two detections collide when their bbox centers are within
        max(r_a, r_b) * 0.7 — the same minDist-style rule used inside each
        detector — to prevent the Hough + contour passes (or Task 1 + Task 2
        running together) from drawing two boxes around one physical circle.
        Higher-confidence detections win.
        """
        out = []
        for d in sorted(dets, key=lambda x: -float(x.get('confidence', 0) or 0)):
            try:
                bx = float(d.get('bbox_x', 0) or 0)
                by = float(d.get('bbox_y', 0) or 0)
                bw = float(d.get('bbox_w', 0) or 0)
                bh = float(d.get('bbox_h', 0) or 0)
            except (TypeError, ValueError):
                continue
            if bw <= 0 or bh <= 0:
                continue
            cx, cy = bx + bw / 2.0, by + bh / 2.0
            r = max(bw, bh) / 2.0
            collide = False
            for k in out:
                kbx = float(k.get('bbox_x', 0) or 0)
                kby = float(k.get('bbox_y', 0) or 0)
                kbw = float(k.get('bbox_w', 0) or 0)
                kbh = float(k.get('bbox_h', 0) or 0)
                kcx, kcy = kbx + kbw / 2.0, kby + kbh / 2.0
                kr = max(kbw, kbh) / 2.0
                if ((cx - kcx) ** 2 + (cy - kcy) ** 2) ** 0.5 < max(r, kr) * 0.7:
                    collide = True
                    break
            if not collide:
                out.append(d)
        return out

    def start_overlay(self):
        # Default: enable Task 1 (preserves prior behaviour for /overlay/enable).
        if self._overlay_enabled:
            return
        self.set_overlay_detectors(task1=True, task2=self._overlay_task2)
        self.get_logger().info("Detection overlay enabled")

    def stop_overlay(self):
        if not self._overlay_enabled and not (self._overlay_task1 or self._overlay_task2):
            return
        self.set_overlay_detectors(task1=False, task2=False)
        self._overlay_stop.set()
        if self._overlay_thread:
            self._overlay_thread.join(timeout=2)
            self._overlay_thread = None
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
            avg_fps = (
                self.video_node.frame_count / max(elapsed, 1)
                if self.video_node else 0
            )
            self._send_json(200, {
                'streaming': receiving_frames,
                'pipeline_playing': pipeline_playing,
                'source_topic': self.video_node.source_topic if self.video_node else '',
                'fps': self.video_node._current_fps() if self.video_node else 0,
                'avg_fps': avg_fps,
                'frame_count': self.video_node.frame_count if self.video_node else 0,
                'error_count': self.video_node.error_count if self.video_node else 0,
                'dropped_count': self.video_node.dropped_count if self.video_node else 0,
                'throttled_count': self.video_node.throttled_count if self.video_node else 0,
                'push_fail_count': self.video_node.push_fail_count if self.video_node else 0,
                'last_frame_age_s': last_frame_age,
                'rtsp_url': f'rtsp://localhost:8554/{self.video_node.rtsp_path}' if self.video_node else '',
                'encoder': self.video_node._encoder_name if self.video_node else '',
                'perf': self.video_node.perf_snapshot() if self.video_node else {},
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

        elif parsed.path == '/snapshot_raw':
            if self.video_node and self.video_node._latest_raw_jpeg:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(self.video_node._latest_raw_jpeg)))
                self.end_headers()
                self.wfile.write(self.video_node._latest_raw_jpeg)
            else:
                self._send_json(503, {'error': 'No raw frame available'})

        elif parsed.path == '/detections':
            query = parse_qs(parsed.query)
            wanted = (query.get('source', ['all'])[0] or 'all').lower()
            dets = []
            if self.video_node:
                with self.video_node._detections_lock:
                    src = list(self.video_node._detections)
                for d in src:
                    if wanted == 'all' or d.get('_detector') == wanted:
                        dets.append(d)
            self._send_json(200, {'count': len(dets), 'detections': dets})

        elif parsed.path == '/depth/center':
            # Sample ZED depth at the stream-frame center (the operator
            # crosshair). Mission Planner polls this for the bottom-right
            # range readout. depth_seen lets the client distinguish
            # "no depth subscribed yet" from "no valid pixels at center"
            # (e.g. NaN saturation when nothing is in range).
            range_m = None
            depth_seen = False
            if self.video_node is not None:
                with self.video_node._depth_lock:
                    depth_seen = self.video_node._latest_depth is not None
                w = int(getattr(self.video_node, 'width', 0) or 0)
                h = int(getattr(self.video_node, 'height', 0) or 0)
                if w > 0 and h > 0:
                    try:
                        range_m = self.video_node._sample_depth_at(
                            w / 2.0, h / 2.0, w, h
                        )
                    except Exception:
                        range_m = None
            self._send_json(200, {
                'range_m': float(range_m) if range_m is not None else None,
                'depth_seen': depth_seen,
            })

        elif parsed.path == '/overlay/status':
            det_count = 0
            if self.video_node:
                with self.video_node._detections_lock:
                    det_count = len(self.video_node._detections)
            self._send_json(200, {
                'enabled': self.video_node._overlay_enabled if self.video_node else False,
                'detection_count': det_count,
                'task1_enabled': self.video_node._overlay_task1 if self.video_node else False,
                'task2_enabled': self.video_node._overlay_task2 if self.video_node else False,
                'mode': self.video_node._overlay_mode if self.video_node else 'task1',
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

        elif parsed.path == '/overlay/detectors':
            query = parse_qs(parsed.query)
            t1 = query.get('task1', ['0'])[0].lower() in ('1', 'true', 'on', 'yes')
            t2 = query.get('task2', ['0'])[0].lower() in ('1', 'true', 'on', 'yes')
            if not self.video_node:
                self._send_json(503, {'success': False, 'message': 'No video node'})
            else:
                self.video_node.set_overlay_detectors(task1=t1, task2=t2)
                self._send_json(200, {
                    'success': True, 'task1_enabled': t1, 'task2_enabled': t2,
                })

        elif parsed.path == '/overlay/mode':
            query = parse_qs(parsed.query)
            mode = query.get('mode', [''])[0]
            if not self.video_node:
                self._send_json(503, {'success': False, 'message': 'No video node'})
            elif self.video_node.set_overlay_mode(mode):
                self._send_json(200, {'success': True, 'mode': mode})
            else:
                self._send_json(400, {'success': False, 'message': f'Invalid mode: {mode}'})

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
        discovered = set()
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'list', '-t'],
                capture_output=True, text=True, timeout=10)
            for line in result.stdout.splitlines():
                parts = line.split()
                if len(parts) >= 2:
                    topic, type_ = parts[0], parts[1].strip('[]')
                    if 'Image' in type_ and 'sensor_msgs' in type_:
                        discovered.add(topic)
        except Exception:
            pass
        # Merge live-discovered topics with known ZED lazy topics.
        # ZED only publishes left/right/raw when a subscriber exists, so they
        # won't appear in ros2 topic list until after the bridge subscribes.
        return sorted(discovered | set(_ZED_KNOWN_IMAGE_TOPICS))


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
    parser.add_argument('--width', type=int, default=1920)
    parser.add_argument('--height', type=int, default=1080)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--bitrate', type=int, default=2500,
                       help='H264 bitrate in kbps')
    parser.add_argument('--http-port', type=int, default=9200)
    parser.add_argument('--rtsp-path', type=str, default='primary')

    parsed_args = parser.parse_args()

    _shutdown = threading.Event()

    def signal_handler(sig, frame):
        print('\nShutting down...')
        _shutdown.set()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    http_server = None
    attempt = 0
    max_restart_delay = 30

    while not _shutdown.is_set():
        attempt += 1
        video_node = None
        try:
            if not rclpy.ok():
                rclpy.init(args=args)

            video_node = VideoStreamNode(
                source_topic=parsed_args.source_topic,
                width=parsed_args.width,
                height=parsed_args.height,
                fps=parsed_args.fps,
                bitrate=parsed_args.bitrate,
                rtsp_path=parsed_args.rtsp_path,
            )

            # HTTP server is created once and reuses the same port
            if http_server is None:
                http_server = run_http_server(video_node, parsed_args.http_port)
            else:
                ControlServer.video_node = video_node

            print(f'[bridge] Node started (attempt {attempt})')
            rclpy.spin(video_node)

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f'[bridge] Unhandled exception (attempt {attempt}): {e}', flush=True)
        finally:
            if video_node is not None:
                try:
                    video_node.cleanup()
                    video_node.destroy_node()
                except Exception:
                    pass
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

        if _shutdown.is_set():
            break

        delay = min(5 * attempt, max_restart_delay)
        print(f'[bridge] Restarting in {delay}s...', flush=True)
        _shutdown.wait(delay)

    print('[bridge] Exiting.')


if __name__ == '__main__':
    main()
