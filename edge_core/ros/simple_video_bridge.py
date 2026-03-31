#!/usr/bin/env python3
"""
Simple Video Bridge for NOMAD - Minimal Implementation

Streams ROS2 camera topics to RTSP via MediaMTX using software H.264 encoding.

Features:
- Software encoding (openh264enc) for minimal CPU usage
- Fixed RTSP URL: rtsp://localhost:8554/primary  
- Dynamic topic switching via HTTP API
- Auto-discovery of available ROS2 image topics
- MediaMTX for multi-viewer streaming (VLC, phone, Mission Planner)
- Adaptive bitrate for choppy network conditions

Architecture:
    ROS2 Image Topic -> openh264enc (software) -> RTSP -> MediaMTX

Target: Jetson Orin Nano / Isaac ROS Docker container
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
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


class VideoStreamNode(Node):
    """ROS2 node that streams images to RTSP via GStreamer."""
    
    def __init__(self, source_topic: str, width: int = 1280, height: int = 720, 
                 fps: int = 30, bitrate: int = 2000):
        super().__init__('simple_video_bridge')
        
        self.bridge = CvBridge()
        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate = bitrate  # kbps
        self.source_topic = source_topic
        self.frame_count = 0
        self.start_time = time.time()
        self.last_frame_time = 0.0  # Timestamp of most recent frame
        self.subscription = None
        self._latest_jpeg = None  # Cached JPEG bytes for snapshot requests
        self._last_snapshot_encode_time = 0.0
        
        # Detection overlay state
        self._overlay_enabled = False
        self._detections = []  # Current detections from Edge Core
        self._detections_lock = threading.Lock()
        self._edge_core_url = "http://172.17.0.1:8000"
        self._overlay_thread = None
        self._overlay_stop = threading.Event()
        
        # Build GStreamer pipeline: appsrc -> openh264enc (software) -> RTSP
        # Use openh264enc available in Isaac ROS container
        # Bitrate is in bps for openh264enc
        # appsrc is-live=true + do-timestamp=true: frames are timestamped on arrival
        # max-buffers=2: limit appsrc internal queue to 2 frames
        # queue leaky=downstream: drop old frames if encoder can't keep up (low latency)
        pipeline_str = (
            f'appsrc name=source is-live=true format=time do-timestamp=true '
            f'max-buffers=2 '
            f'caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! '
            f'queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! '
            f'videoconvert ! '
            f'openh264enc bitrate={bitrate * 1000} num-slices=4 ! '
            f'video/x-h264,profile=baseline ! '
            f'h264parse config-interval=1 ! '
            f'rtspclientsink location=rtsp://172.17.0.1:8554/primary protocols=tcp'
        )
        
        self.get_logger().info('Starting GStreamer pipeline')
        self.get_logger().info(f'  Topic: {source_topic}')
        self.get_logger().info(f'  Resolution: {width}x{height}@{fps}fps')
        self.get_logger().info(f'  Encoder: openh264enc (software)')
        self.get_logger().info(f'  Bitrate: {bitrate}kbps (adaptive)')
        self.get_logger().info(f'  RTSP: rtsp://localhost:8554/primary')
        
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsrc = self.pipeline.get_by_name('source')
            
            # Start pipeline
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                self.get_logger().error('Failed to start GStreamer pipeline')
                raise RuntimeError('GStreamer pipeline failed to start')
            
            self.get_logger().info('GStreamer pipeline started successfully')
            
        except Exception as e:
            self.get_logger().error(f'Pipeline error: {e}')
            raise
        
        # Subscribe to ROS2 image topic
        self._subscribe_to_topic(source_topic)
        
        self.get_logger().info('Video bridge ready!')
    
    def _subscribe_to_topic(self, topic: str):
        """Subscribe to a ROS2 image topic."""
        if self.subscription:
            self.destroy_subscription(self.subscription)

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
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
        """
        Switch to a different ROS2 image topic.
        
        Keeps the existing GStreamer pipeline alive and only switches
        the ROS subscription. This avoids multi-second RTSP interruptions.
        """
        try:
            self.get_logger().info(f'Switching topic: {self.source_topic} -> {new_topic}')

            # Update ROS subscription
            self._subscribe_to_topic(new_topic)
            
            # Keep counters/encoder state so the RTSP stream remains continuous.
            
            self.get_logger().info(f'Successfully switched to: {new_topic} (pipeline preserved)')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to switch topic: {e}')
            return False
    
    def image_callback(self, msg: Image):
        """Process incoming ROS2 images and push to GStreamer."""
        try:
            import cv2
            
            # Convert ROS image to OpenCV BGR format
            # Handle different encodings: color, alpha, depth, confidence, grayscale
            encoding = msg.encoding.lower()
            if encoding in ('bgra8', 'rgba8', '8uc4'):
                # Alpha channel color images (e.g., ZED cameras)
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                if encoding == 'rgba8':
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2BGR)
                else:
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            elif encoding in ('16uc1', 'mono16', '32fc1'):
                # Depth or confidence: single-channel numeric data
                # Needs normalization + colormap to produce visible frames
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv_image = self._normalize_depth_image(cv_image, encoding)
            elif encoding in ('mono8', '8uc1'):
                # Grayscale: convert to BGR for the encoder
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize if needed
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                cv_image = cv2.resize(cv_image, (self.width, self.height))
            
            # Draw detection overlay (bounding boxes) if enabled
            if self._overlay_enabled:
                self.draw_detections(cv_image)
            
            # Cache snapshots at a low rate; per-frame JPEG encoding hurts FPS.
            now = time.time()
            if (now - self._last_snapshot_encode_time) >= 0.5:
                ok, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok:
                    self._latest_jpeg = jpeg.tobytes()
                    self._last_snapshot_encode_time = now

            # Create GStreamer buffer and push to pipeline
            data = cv_image.tobytes()
            buf = Gst.Buffer.new_wrapped(data)
            ret = self.appsrc.emit('push-buffer', buf)
            
            if ret != Gst.FlowReturn.OK:
                self.get_logger().warn(f'Buffer push returned: {ret}')
            
            # Stats
            self.frame_count += 1
            self.last_frame_time = time.time()
            if self.frame_count % 300 == 0:  # Every 10 seconds at 30fps
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                self.get_logger().info(
                    f'Streaming: {self.frame_count} frames, {fps:.1f} fps avg'
                )
                
        except Exception as e:
            if self.frame_count % 100 == 0:  # Don't spam errors
                self.get_logger().error(f'Frame processing error: {e}')
    
    def _normalize_depth_image(self, image, encoding):
        """Normalize 16-bit or 32-bit depth/confidence image to visible BGR with colormap.
        
        Depth images (16UC1, 32FC1, mono16) are single-channel numeric data that appear
        black when displayed directly. This applies percentile normalization and a Turbo
        colormap to produce a visually meaningful false-color representation.
        """
        import cv2
        
        img = image.astype(np.float32) if encoding != '32fc1' else image.copy()
        
        # Mask invalid values (0, NaN, inf)
        valid_mask = np.isfinite(img) & (img > 0)
        
        if not valid_mask.any():
            return np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
        
        # Percentile clipping for robust normalization across varying depth ranges
        min_val = np.percentile(img[valid_mask], 2)
        max_val = np.percentile(img[valid_mask], 98)
        
        if max_val <= min_val:
            max_val = min_val + 1.0
        
        normalized = np.clip((img - min_val) / (max_val - min_val) * 255.0, 0, 255).astype(np.uint8)
        normalized[~valid_mask] = 0
        
        # Turbo colormap gives good visual contrast for depth data
        colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        
        return colored
    
    def cleanup(self):
        """Clean shutdown."""
        self.get_logger().info('Stopping video bridge...')
        self.stop_overlay()
        if hasattr(self, 'pipeline') and self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        self.get_logger().info('Stopped')
    
    # ---- Detection overlay ----
    
    # Color map: label substring -> BGR color
    _LABEL_COLORS = {
        'red': (0, 0, 255),
        'blue': (255, 140, 0),
        'green': (0, 200, 0),
        'yellow': (0, 255, 255),
        'orange': (0, 165, 255),
        'white': (255, 255, 255),
        'black': (80, 80, 80),
    }
    _DEFAULT_COLOR = (0, 255, 0)  # Green fallback
    
    def _color_for_label(self, label: str):
        """Return BGR color tuple for a given detection label."""
        label_lower = label.lower()
        for key, color in self._LABEL_COLORS.items():
            if key in label_lower:
                return color
        return self._DEFAULT_COLOR
    
    def draw_detections(self, frame):
        """
        Draw detection bounding boxes onto the frame (in-place).
        
        Detections come from Edge Core API with bbox_x/y/w/h in original
        camera pixels. The frame may be resized, so coords are scaled
        to match self.width x self.height.
        """
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
            
            # Scale bbox to output frame size if source resolution differs
            # (original frame is typically same res, but handle mismatch)
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
            
            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            
            # Draw label background + text
            text = f"{label} {conf:.0%}"
            (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1, y1 - th - baseline - 4), (x1 + tw + 4, y1), color, -1)
            cv2.putText(frame, text, (x1 + 2, y1 - baseline - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
        # Detection count badge in top-left
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
                    # API returns {"current": {"detections": [...]}, ...}
                    current = data.get('current', {})
                    dets = current.get('detections', []) if isinstance(current, dict) else []
                    with self._detections_lock:
                        self._detections = dets
            except Exception:
                # Edge Core unreachable -- clear detections so overlay disappears
                with self._detections_lock:
                    self._detections = []
            self._overlay_stop.wait(0.2)  # 5 Hz
    
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
    
    # Class variable to store the video node reference
    video_node = None
    
    def log_message(self, format, *args):
        """Suppress default logging."""
        pass
    
    def do_GET(self):
        """Handle GET requests."""
        parsed = urlparse(self.path)

        pipeline_playing = False
        if self.video_node and getattr(self.video_node, 'pipeline', None):
            try:
                state = self.video_node.pipeline.get_state(0)[1]
                pipeline_playing = state == Gst.State.PLAYING
            except Exception:
                pipeline_playing = False
        
        if parsed.path == '/health':
            # Report streaming only if frames arrived recently (within 5s)
            receiving_frames = (
                self.video_node is not None
                and self.video_node.last_frame_time > 0
                and (time.time() - self.video_node.last_frame_time) < 5.0
            )
            self._send_json(200, {
                'healthy': pipeline_playing,
                'streaming': receiving_frames,
                'pipeline_playing': pipeline_playing,
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
                'error_count': 0,
                'last_frame_age_s': last_frame_age,
                'rtsp_url': 'rtsp://localhost:8554/primary',
            }
            self._send_json(200, status)
        
        elif parsed.path == '/topics':
            topics = self._discover_topics()
            self._send_json(200, {'topics': topics, 'count': len(topics)})
        
        elif parsed.path == '/snapshot':
            # Return the latest frame as JPEG for fast capture
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
        """Handle POST requests."""
        parsed = urlparse(self.path)
        
        if parsed.path == '/switch':
            # Parse query parameters
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
        
        else:
            self._send_json(404, {'error': 'Not found'})
    
    def _send_json(self, code: int, data: dict):
        """Send JSON response."""
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())
    
    def _discover_topics(self) -> list:
        """Discover available ROS2 image topics."""
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
                    # Include all Image topics (sensor_msgs/msg/Image or sensor_msgs/Image)
                    if 'Image' in type_ and 'sensor_msgs' in type_:
                        topics.append(topic)
            
            return sorted(topics)
            
        except Exception as e:
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
                       help='Output video width (1280 for single, 2560 for stereo)')
    parser.add_argument('--height', type=int, default=720,
                       help='Output video height')
    parser.add_argument('--fps', type=int, default=30,
                       help='Target framerate')
    parser.add_argument('--bitrate', type=int, default=2000,
                       help='H264 bitrate in kbps (adaptive)')
    parser.add_argument('--http-port', type=int, default=9200,
                       help='HTTP control server port')
    
    parsed_args = parser.parse_args()
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print('\nShutting down...')
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create video node
    video_node = VideoStreamNode(
        source_topic=parsed_args.source_topic,
        width=parsed_args.width,
        height=parsed_args.height,
        fps=parsed_args.fps,
        bitrate=parsed_args.bitrate
    )
    
    # Start HTTP control server
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
