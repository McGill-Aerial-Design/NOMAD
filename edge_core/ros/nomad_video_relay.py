#!/usr/bin/env python3
"""
NOMAD Video Relay Node

Single-node ROS2 video streaming solution that:
1. Subscribes to a selected image topic (switchable dynamically)
2. Encodes with NVIDIA NVENC hardware acceleration (nvv4l2h264enc)
3. Pushes to MediaMTX RTSP server
4. Provides HTTP API for control (switch topic, list topics, status)

Architecture:
    ROS2 Image Topic -> GStreamer (NVENC) -> RTSP -> MediaMTX -> Ground Station

Key Features:
- Dynamic topic switching via HTTP API (no restart required)
- BEST_EFFORT QoS for low latency
- Hardware H.264 encoding with NVENC
- Built-in HTTP control server (no external ROS2 service tools needed)
- Frame rate and error tracking

Usage:
    python3 nomad_video_relay.py
    python3 nomad_video_relay.py --topic /zed/zed_node/left/image_rect_color
    python3 nomad_video_relay.py --rtsp-url rtsp://172.17.0.1:8554/secondary --http-port 9201

HTTP API (default port 9200):
    GET  /topics          - List available ROS2 image topics
    GET  /status          - Current topic, fps, frame count, errors
    POST /switch?topic=X  - Switch to new topic

Safety Critical: Runs on a flying drone. All operations must be validated.
Target: Python 3.10+ | ROS2 Humble | Jetson Orin Nano | GStreamer 1.x
"""

import argparse
import json
import logging
import signal
import sys
import threading
import time
from dataclasses import dataclass, asdict
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Optional, List, Dict, Any
from urllib.parse import urlparse, parse_qs

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    import numpy as np
except ImportError as e:
    print(f"ERROR: ROS2 dependencies not available: {e}")
    print("This script must run inside a ROS2 environment")
    sys.exit(1)

# GStreamer imports
try:
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstApp', '1.0')
    from gi.repository import Gst, GstApp, GLib
except ImportError as e:
    print(f"ERROR: GStreamer Python bindings not available: {e}")
    print("Install with: apt-get install python3-gi gstreamer1.0-python3-plugin-loader")
    sys.exit(1)

# Initialize GStreamer
Gst.init(None)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger('nomad_video_relay')

# Constants
DEFAULT_TOPIC = "/zed/zed_node/rgb/image_rect_color"
DEFAULT_RTSP_URL = "rtsp://172.17.0.1:8554/primary"
DEFAULT_HTTP_PORT = 9200
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 30
DEFAULT_BITRATE = 4  # Mbps

# Safety limits
MAX_FRAME_WIDTH = 3840  # 4K
MAX_FRAME_HEIGHT = 2160
MAX_FRAME_SIZE = MAX_FRAME_WIDTH * MAX_FRAME_HEIGHT * 4  # RGBA


@dataclass
class VideoStats:
    """Video streaming statistics."""
    current_topic: str
    frame_count: int
    error_count: int
    dropped_frames: int
    fps: float
    last_frame_time: float
    streaming: bool
    encoder_type: str


class VideoRelayHTTPHandler(BaseHTTPRequestHandler):
    """HTTP request handler for video relay control API."""
    
    # Class variable to hold reference to the video node
    video_node: Optional['VideoRelayNode'] = None
    
    def log_message(self, format, *args):
        """Override to use our logger."""
        logger.debug(f"HTTP {format % args}")
    
    def do_GET(self):
        """Handle GET requests."""
        parsed = urlparse(self.path)
        path = parsed.path
        
        if path == '/topics':
            self._handle_list_topics()
        elif path == '/status':
            self._handle_get_status()
        elif path == '/health':
            self._handle_health()
        else:
            self._send_error(404, "Not Found")
    
    def do_POST(self):
        """Handle POST requests."""
        parsed = urlparse(self.path)
        path = parsed.path
        
        if path == '/switch':
            self._handle_switch_topic(parsed.query)
        else:
            self._send_error(404, "Not Found")
    
    def _handle_list_topics(self):
        """List available image topics."""
        if not self.video_node:
            self._send_error(500, "Video node not initialized")
            return
        
        try:
            topics = self.video_node.get_image_topics()
            self._send_json(200, {"topics": topics, "count": len(topics)})
        except Exception as e:
            logger.error(f"Error listing topics: {e}")
            self._send_error(500, str(e))
    
    def _handle_get_status(self):
        """Get current video relay status."""
        if not self.video_node:
            self._send_error(500, "Video node not initialized")
            return
        
        try:
            stats = self.video_node.get_stats()
            self._send_json(200, asdict(stats))
        except Exception as e:
            logger.error(f"Error getting status: {e}")
            self._send_error(500, str(e))
    
    def _handle_health(self):
        """Health check endpoint for video stream manager."""
        if not self.video_node:
            self._send_json(200, {"healthy": False, "reason": "Video node not initialized"})
            return
        
        try:
            is_healthy = self.video_node._pipeline_running
            self._send_json(200, {
                "healthy": is_healthy,
                "streaming": is_healthy
            })
        except Exception as e:
            self._send_json(200, {"healthy": False, "reason": str(e)})
    
    def _handle_switch_topic(self, query_string: str):
        """Switch to a different topic."""
        if not self.video_node:
            self._send_error(500, "Video node not initialized")
            return
        
        try:
            params = parse_qs(query_string)
            if 'topic' not in params:
                self._send_error(400, "Missing 'topic' parameter")
                return
            
            new_topic = params['topic'][0]
            success = self.video_node.switch_topic(new_topic)
            
            if success:
                self._send_json(200, {
                    "success": True,
                    "topic": new_topic,
                    "message": f"Switched to {new_topic}"
                })
            else:
                self._send_json(500, {
                    "success": False,
                    "message": "Failed to switch topic"
                })
        except Exception as e:
            logger.error(f"Error switching topic: {e}")
            self._send_error(500, str(e))
    
    def _send_json(self, status_code: int, data: dict):
        """Send JSON response."""
        self.send_response(status_code)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())
    
    def _send_error(self, status_code: int, message: str):
        """Send error response."""
        self._send_json(status_code, {"error": message})


class VideoRelayNode(Node):
    """
    ROS2 node for video relay with GStreamer NVENC encoding.
    
    Subscribes to ROS2 image topics and streams via RTSP using hardware encoding.
    Provides HTTP API for dynamic control.
    """
    
    def __init__(
        self,
        initial_topic: str,
        rtsp_url: str,
        http_port: int,
        width: int,
        height: int,
        fps: int,
        bitrate: int,
    ):
        super().__init__('nomad_video_relay')
        
        # Configuration
        self.current_topic = initial_topic
        self.rtsp_url = rtsp_url
        self.http_port = http_port
        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate = bitrate
        
        # State
        self.frame_count = 0
        self.error_count = 0
        self.dropped_frames = 0
        self.last_frame_time = 0.0
        self.frame_times = []  # For FPS calculation
        self._pipeline_running = False
        self._shutdown_requested = False
        self._topic_lock = threading.Lock()
        self._topic_switch_pending = False
        
        # GStreamer pipeline
        self.pipeline: Optional[Gst.Pipeline] = None
        self.appsrc: Optional[GstApp.AppSrc] = None
        self.encoder_type = "unknown"
        
        # ROS2 subscription
        self.image_sub = None
        
        # HTTP server
        self.http_server: Optional[HTTPServer] = None
        self.http_thread: Optional[threading.Thread] = None
        
        # Initialize
        self._check_nvenc_available()
        self._init_gstreamer_pipeline()
        self._setup_ros_subscription()
        self._start_http_server()
        
        logger.info(f"Video relay initialized: {self.current_topic} -> {self.rtsp_url}")
        logger.info(f"HTTP control API: http://0.0.0.0:{self.http_port}")
    
    def _check_nvenc_available(self) -> bool:
        """Check if NVIDIA hardware encoder is available."""
        try:
            test_pipeline = Gst.parse_launch("nvv4l2h264enc ! fakesink")
            if test_pipeline:
                test_pipeline.set_state(Gst.State.NULL)
                logger.info("NVENC hardware encoder available")
                return True
        except Exception as e:
            logger.warning(f"NVENC not available: {e}")
        return False
    
    def _init_gstreamer_pipeline(self):
        """Initialize GStreamer pipeline with NVENC encoding."""
        # Check NVENC availability first
        use_nvenc = self._check_nvenc_available()
        
        if use_nvenc:
            # NVENC hardware pipeline
            pipeline_str = (
                f"appsrc name=src format=time is-live=true block=true do-timestamp=true "
                f"caps=video/x-raw,format=BGR,width={self.width},height={self.height},framerate={self.fps}/1 ! "
                f"queue max-size-buffers=2 leaky=downstream ! "
                f"videoconvert ! video/x-raw,format=BGRx ! "
                f"nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
                f"nvv4l2h264enc bitrate={self.bitrate * 1000000} preset-level=1 "
                f"iframeinterval={self.fps} insert-sps-pps=true control-rate=1 ! "
                f"h264parse config-interval=1 ! "
                f"rtspclientsink location={self.rtsp_url} protocols=tcp latency=0"
            )
            self.encoder_type = "nvv4l2h264enc (NVENC)"
            logger.info("Using NVIDIA NVENC hardware encoder")
        else:
            # Fallback to OpenH264 software encoder (available in Isaac ROS container)
            logger.warning("NVENC not available, falling back to software encoding (openh264enc)")
            pipeline_str = (
                f"appsrc name=src format=time is-live=true block=true do-timestamp=true "
                f"caps=video/x-raw,format=BGR,width={self.width},height={self.height},framerate={self.fps}/1 ! "
                f"queue max-size-buffers=2 leaky=downstream ! "
                f"videoconvert ! video/x-raw,format=I420 ! "
                f"openh264enc bitrate={self.bitrate * 1000000} complexity=low enable-frame-skip=true ! "
                f"h264parse config-interval=1 ! "
                f"rtspclientsink location={self.rtsp_url} protocols=tcp latency=0"
            )
            self.encoder_type = "openh264enc (software)"
        
        logger.info(f"Creating GStreamer pipeline")
        logger.debug(f"Pipeline: {pipeline_str}")
        
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsrc = self.pipeline.get_by_name('src')
            
            if not self.appsrc:
                logger.error("Failed to get appsrc element from pipeline")
                return
            
            # Configure appsrc
            self.appsrc.set_property('format', Gst.Format.TIME)
            self.appsrc.set_property('is-live', True)
            self.appsrc.set_property('block', True)
            
            # Connect to bus messages
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect('message', self._on_bus_message)
            
            # Start GLib main loop in separate thread
            self.glib_loop = GLib.MainLoop()
            self.glib_thread = threading.Thread(target=self.glib_loop.run, daemon=True)
            self.glib_thread.start()
            
            # Start pipeline
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                logger.error("Failed to start GStreamer pipeline")
                return
            
            self._pipeline_running = True
            logger.info(f"GStreamer pipeline started successfully ({self.encoder_type})")
            
        except Exception as e:
            logger.error(f"Failed to create GStreamer pipeline: {e}")
            raise
    
    def _on_bus_message(self, bus, message):
        """Handle GStreamer bus messages."""
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            logger.error(f"GStreamer error: {err}, debug: {debug}")
            self.error_count += 1
        elif t == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            logger.warning(f"GStreamer warning: {warn}, debug: {debug}")
        elif t == Gst.MessageType.EOS:
            logger.info("GStreamer EOS received")
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old, new, pending = message.parse_state_changed()
                logger.debug(f"Pipeline state: {old.value_nick} -> {new.value_nick}")
    
    def _setup_ros_subscription(self):
        """Set up ROS2 image subscription."""
        # QoS for real-time video - best effort, keep last
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        
        self.image_sub = self.create_subscription(
            Image,
            self.current_topic,
            self._image_callback,
            sensor_qos
        )
        logger.info(f"Subscribed to: {self.current_topic}")
    
    def _start_http_server(self):
        """Start HTTP control server."""
        try:
            # Set the video node reference for the handler
            VideoRelayHTTPHandler.video_node = self
            
            self.http_server = HTTPServer(('0.0.0.0', self.http_port), VideoRelayHTTPHandler)
            self.http_thread = threading.Thread(
                target=self.http_server.serve_forever,
                daemon=True
            )
            self.http_thread.start()
            logger.info(f"HTTP server started on port {self.http_port}")
        except Exception as e:
            logger.error(f"Failed to start HTTP server: {e}")
    
    def _image_callback(self, msg: Image):
        """
        Process incoming ROS image and push to GStreamer pipeline.
        
        SAFETY CRITICAL: All buffer operations must validate sizes.
        """
        if self._shutdown_requested or not self._pipeline_running:
            return
        
        if not self.appsrc or self._topic_switch_pending:
            return
        
        try:
            # SAFETY: Validate message
            if msg is None or msg.data is None:
                self.error_count += 1
                return
            
            encoding = msg.encoding
            height = msg.height
            width = msg.width
            
            # SAFETY: Validate dimensions
            if width <= 0 or height <= 0:
                self.error_count += 1
                logger.warning(f"Invalid dimensions: {width}x{height}")
                return
            
            if width > MAX_FRAME_WIDTH or height > MAX_FRAME_HEIGHT:
                self.error_count += 1
                logger.warning(f"Frame too large: {width}x{height}")
                return
            
            # Convert to BGR numpy array
            frame = self._convert_to_bgr(msg)
            if frame is None:
                self.error_count += 1
                return
            
            # Resize if needed
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                import cv2
                frame = cv2.resize(frame, (self.width, self.height))
            
            # Push to GStreamer
            self._push_frame(frame)
            
            # Update stats
            self.frame_count += 1
            current_time = time.time()
            self.last_frame_time = current_time
            self.frame_times.append(current_time)
            
            # Keep only last second of frame times for FPS calculation
            cutoff = current_time - 1.0
            self.frame_times = [t for t in self.frame_times if t > cutoff]
            
        except Exception as e:
            self.error_count += 1
            logger.error(f"Error processing frame: {e}")
    
    def _convert_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        """Convert ROS Image message to BGR numpy array."""
        try:
            encoding = msg.encoding
            height = msg.height
            width = msg.width
            data = bytes(msg.data)
            
            # Calculate expected size
            if encoding == 'bgr8':
                expected_size = height * width * 3
                if len(data) != expected_size:
                    logger.warning(f"Size mismatch for bgr8: expected {expected_size}, got {len(data)}")
                    return None
                frame = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
                return frame
            
            elif encoding == 'rgb8':
                expected_size = height * width * 3
                if len(data) != expected_size:
                    logger.warning(f"Size mismatch for rgb8: expected {expected_size}, got {len(data)}")
                    return None
                frame = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
                # Convert RGB to BGR
                return frame[:, :, ::-1]
            
            elif encoding == 'bgra8':
                expected_size = height * width * 4
                if len(data) != expected_size:
                    logger.warning(f"Size mismatch for bgra8: expected {expected_size}, got {len(data)}")
                    return None
                frame = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 4))
                # Drop alpha channel
                return frame[:, :, :3]
            
            elif encoding == 'rgba8':
                expected_size = height * width * 4
                if len(data) != expected_size:
                    logger.warning(f"Size mismatch for rgba8: expected {expected_size}, got {len(data)}")
                    return None
                frame = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 4))
                # Convert RGBA to BGR
                return frame[:, :, [2, 1, 0]]
            
            else:
                logger.warning(f"Unsupported encoding: {encoding}")
                return None
                
        except Exception as e:
            logger.error(f"Error converting image: {e}")
            return None
    
    def _push_frame(self, frame: np.ndarray):
        """Push a frame to the GStreamer pipeline."""
        try:
            # SAFETY: Validate frame
            if frame is None or frame.size == 0:
                return
            
            if frame.size > MAX_FRAME_SIZE:
                logger.warning(f"Frame too large: {frame.size} bytes")
                self.dropped_frames += 1
                return
            
            # Create GStreamer buffer
            data = frame.tobytes()
            buf = Gst.Buffer.new_wrapped(data)
            
            # Push to appsrc
            ret = self.appsrc.emit('push-buffer', buf)
            
            if ret != Gst.FlowReturn.OK:
                logger.warning(f"Failed to push buffer: {ret}")
                self.dropped_frames += 1
                
        except Exception as e:
            logger.error(f"Error pushing frame: {e}")
            self.error_count += 1
    
    def get_image_topics(self) -> List[str]:
        """Get list of available image topics."""
        topics = []
        try:
            # Get all topics and their types
            topic_list = self.get_topic_names_and_types()
            
            for topic_name, topic_types in topic_list:
                # Check if it's an image topic
                if 'sensor_msgs/msg/Image' in topic_types:
                    topics.append(topic_name)
            
            topics.sort()
            logger.info(f"Found {len(topics)} image topics")
            
        except Exception as e:
            logger.error(f"Error getting topics: {e}")
        
        return topics
    
    def get_stats(self) -> VideoStats:
        """Get current video statistics."""
        # Calculate FPS from recent frame times
        fps = len(self.frame_times) if self.frame_times else 0.0
        
        return VideoStats(
            current_topic=self.current_topic,
            frame_count=self.frame_count,
            error_count=self.error_count,
            dropped_frames=self.dropped_frames,
            fps=fps,
            last_frame_time=self.last_frame_time,
            streaming=self._pipeline_running,
            encoder_type=self.encoder_type,
        )
    
    def switch_topic(self, new_topic: str) -> bool:
        """
        Switch to a different ROS image topic.
        
        This allows dynamic topic switching without restarting the pipeline.
        The GStreamer pipeline continues running, only the ROS subscription changes.
        
        Args:
            new_topic: The new ROS image topic to subscribe to
            
        Returns:
            True if switch was successful
        """
        with self._topic_lock:
            if new_topic == self.current_topic:
                logger.info(f"Already subscribed to {new_topic}")
                return True
            
            logger.info(f"Switching topic: {self.current_topic} -> {new_topic}")
            self._topic_switch_pending = True
            
            try:
                # Destroy old subscription
                if self.image_sub:
                    self.destroy_subscription(self.image_sub)
                
                # Create new subscription
                sensor_qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    durability=DurabilityPolicy.VOLATILE,
                )
                
                self.image_sub = self.create_subscription(
                    Image,
                    new_topic,
                    self._image_callback,
                    sensor_qos
                )
                
                self.current_topic = new_topic
                self._topic_switch_pending = False
                
                logger.info(f"Successfully switched to {new_topic}")
                return True
                
            except Exception as e:
                logger.error(f"Failed to switch topic: {e}")
                self._topic_switch_pending = False
                return False
    
    def shutdown(self):
        """Clean shutdown of the video relay."""
        logger.info("Shutting down video relay...")
        self._shutdown_requested = True
        
        # Stop HTTP server
        if self.http_server:
            self.http_server.shutdown()
            logger.info("HTTP server stopped")
        
        # Stop GStreamer pipeline
        if self.pipeline:
            self.pipeline.send_event(Gst.Event.new_eos())
            self.pipeline.set_state(Gst.State.NULL)
            logger.info("GStreamer pipeline stopped")
        
        # Stop GLib loop
        if hasattr(self, 'glib_loop'):
            self.glib_loop.quit()
            logger.info("GLib loop stopped")
        
        logger.info("Video relay shutdown complete")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='NOMAD Video Relay - ROS2 to RTSP with NVENC encoding'
    )
    parser.add_argument(
        '--topic',
        type=str,
        default=DEFAULT_TOPIC,
        help=f'Initial ROS image topic (default: {DEFAULT_TOPIC})'
    )
    parser.add_argument(
        '--rtsp-url',
        type=str,
        default=DEFAULT_RTSP_URL,
        help=f'RTSP destination URL (default: {DEFAULT_RTSP_URL})'
    )
    parser.add_argument(
        '--http-port',
        type=int,
        default=DEFAULT_HTTP_PORT,
        help=f'HTTP control API port (default: {DEFAULT_HTTP_PORT})'
    )
    parser.add_argument(
        '--width',
        type=int,
        default=DEFAULT_WIDTH,
        help=f'Output video width (default: {DEFAULT_WIDTH})'
    )
    parser.add_argument(
        '--height',
        type=int,
        default=DEFAULT_HEIGHT,
        help=f'Output video height (default: {DEFAULT_HEIGHT})'
    )
    parser.add_argument(
        '--fps',
        type=int,
        default=DEFAULT_FPS,
        help=f'Output framerate (default: {DEFAULT_FPS})'
    )
    parser.add_argument(
        '--bitrate',
        type=int,
        default=DEFAULT_BITRATE,
        help=f'Bitrate in Mbps (default: {DEFAULT_BITRATE})'
    )
    parser.add_argument(
        '--log-level',
        type=str,
        default='INFO',
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
        help='Logging level'
    )
    
    args = parser.parse_args()
    
    # Set log level
    logger.setLevel(getattr(logging, args.log_level))
    
    # Initialize ROS2
    rclpy.init()
    
    # Create node
    node = None
    try:
        node = VideoRelayNode(
            initial_topic=args.topic,
            rtsp_url=args.rtsp_url,
            http_port=args.http_port,
            width=args.width,
            height=args.height,
            fps=args.fps,
            bitrate=args.bitrate,
        )
        
        logger.info("Video relay running. Press Ctrl+C to exit.")
        logger.info(f"Streaming: {args.topic} -> {args.rtsp_url}")
        logger.info(f"HTTP API: http://0.0.0.0:{args.http_port}")
        
        # Spin the node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        logger.info("Received shutdown signal")
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()
        logger.info("Video relay terminated")


if __name__ == '__main__':
    main()
