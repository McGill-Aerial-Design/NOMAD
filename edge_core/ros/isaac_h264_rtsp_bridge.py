#!/usr/bin/env python3
"""
Isaac ROS H264 to RTSP Bridge

Simple bridge that:
1. Subscribes to CompressedImage topic (H264 from isaac_ros_h264_encoder)
2. Pushes raw H264 NAL units to MediaMTX RTSP server

Architecture:
    ZED Camera -> Isaac ROS H264 Encoder -> CompressedImage (H264) -> This Bridge -> RTSP

This replaces the previous GStreamer-based encoding approach with Isaac ROS native H264 encoding.
The Isaac ROS H264 encoder uses NVIDIA hardware acceleration for encoding.

Usage:
    # First, launch the Isaac ROS H264 encoder (separate terminal/launch file):
    ros2 run isaac_ros_h264_encoder encoder_node --ros-args \
        -r image_raw:=/zed/zed_node/rgb/image_rect_color \
        -p input_width:=1280 -p input_height:=720

    # Then run this bridge:
    python3 isaac_h264_rtsp_bridge.py

HTTP API (default port 9200):
    GET  /topics          - List available ROS2 image topics
    GET  /status          - Current status, fps, frame count
    POST /switch?topic=X  - Switch to new image topic (restarts encoder)
    GET  /health          - Health check

Target: Python 3.10+ | ROS2 Humble | Jetson Orin Nano
"""

import argparse
import json
import logging
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, asdict
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Optional, List
from urllib.parse import urlparse, parse_qs

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage, Image
except ImportError as e:
    print(f"ERROR: ROS2 dependencies not available: {e}")
    print("This script must run inside a ROS2 environment")
    sys.exit(1)

# GStreamer imports (for RTSP push only - no encoding)
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
logger = logging.getLogger('isaac_h264_rtsp_bridge')

# Constants
DEFAULT_SOURCE_TOPIC = "/zed/zed_node/rgb/image_rect_color"
DEFAULT_H264_TOPIC = "/image_compressed"  # Isaac ROS H264 encoder output
DEFAULT_RTSP_URL = "rtsp://172.17.0.1:8554/primary"
DEFAULT_HTTP_PORT = 9200
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720


@dataclass
class VideoStats:
    """Video streaming statistics."""
    source_topic: str
    h264_topic: str
    frame_count: int
    error_count: int
    fps: float
    last_frame_time: float
    streaming: bool
    encoder_running: bool


class HTTPHandler(BaseHTTPRequestHandler):
    """HTTP request handler for control API."""
    
    bridge_node: Optional['IsaacH264RtspBridge'] = None
    
    def log_message(self, format, *args):
        logger.debug(f"HTTP {format % args}")
    
    def do_GET(self):
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
        parsed = urlparse(self.path)
        path = parsed.path
        
        if path == '/switch':
            self._handle_switch_topic(parsed.query)
        else:
            self._send_error(404, "Not Found")
    
    def _handle_list_topics(self):
        if not self.bridge_node:
            self._send_error(500, "Bridge not initialized")
            return
        
        try:
            topics = self.bridge_node.get_image_topics()
            self._send_json(200, {"topics": topics, "count": len(topics)})
        except Exception as e:
            logger.error(f"Error listing topics: {e}")
            self._send_error(500, str(e))
    
    def _handle_get_status(self):
        if not self.bridge_node:
            self._send_error(500, "Bridge not initialized")
            return
        
        try:
            stats = self.bridge_node.get_stats()
            self._send_json(200, asdict(stats))
        except Exception as e:
            logger.error(f"Error getting status: {e}")
            self._send_error(500, str(e))
    
    def _handle_health(self):
        if not self.bridge_node:
            self._send_json(200, {"healthy": False, "reason": "Bridge not initialized"})
            return
        
        try:
            is_healthy = self.bridge_node._pipeline_running
            self._send_json(200, {"healthy": is_healthy, "streaming": is_healthy})
        except Exception as e:
            self._send_json(200, {"healthy": False, "reason": str(e)})
    
    def _handle_switch_topic(self, query_string: str):
        if not self.bridge_node:
            self._send_error(500, "Bridge not initialized")
            return
        
        try:
            params = parse_qs(query_string)
            if 'topic' not in params:
                self._send_error(400, "Missing 'topic' parameter")
                return
            
            new_topic = params['topic'][0]
            success = self.bridge_node.switch_source_topic(new_topic)
            
            if success:
                self._send_json(200, {
                    "success": True,
                    "topic": new_topic,
                    "message": f"Switching to {new_topic} - encoder restarting"
                })
            else:
                self._send_json(500, {"success": False, "message": "Failed to switch topic"})
        except Exception as e:
            logger.error(f"Error switching topic: {e}")
            self._send_error(500, str(e))
    
    def _send_json(self, status_code: int, data: dict):
        self.send_response(status_code)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())
    
    def _send_error(self, status_code: int, message: str):
        self._send_json(status_code, {"error": message})


class IsaacH264RtspBridge(Node):
    """
    ROS2 node that bridges Isaac ROS H264 encoder output to RTSP.
    
    Subscribes to CompressedImage (H264) from isaac_ros_h264_encoder and
    pushes raw NAL units to MediaMTX RTSP server via GStreamer.
    """
    
    def __init__(
        self,
        source_topic: str,
        h264_topic: str,
        rtsp_url: str,
        http_port: int,
        width: int,
        height: int,
    ):
        super().__init__('isaac_h264_rtsp_bridge')
        
        self.source_topic = source_topic
        self.h264_topic = h264_topic
        self.rtsp_url = rtsp_url
        self.http_port = http_port
        self.width = width
        self.height = height
        
        # State
        self.frame_count = 0
        self.error_count = 0
        self.last_frame_time = 0.0
        self.frame_times = []
        self._pipeline_running = False
        self._shutdown_requested = False
        self._encoder_process: Optional[subprocess.Popen] = None
        self._topic_lock = threading.Lock()
        
        # GStreamer pipeline (for RTSP push only)
        self.pipeline: Optional[Gst.Pipeline] = None
        self.appsrc: Optional[GstApp.AppSrc] = None
        
        # ROS2 subscription
        self.h264_sub = None
        
        # HTTP server
        self.http_server: Optional[HTTPServer] = None
        self.http_thread: Optional[threading.Thread] = None
        
        # Initialize
        self._init_gstreamer_pipeline()
        self._start_encoder()
        self._setup_ros_subscription()
        self._start_http_server()
        
        logger.info(f"Isaac H264 RTSP Bridge initialized")
        logger.info(f"  Source topic: {self.source_topic}")
        logger.info(f"  H264 topic: {self.h264_topic}")
        logger.info(f"  RTSP URL: {self.rtsp_url}")
        logger.info(f"  HTTP API: http://0.0.0.0:{self.http_port}")
    
    def _init_gstreamer_pipeline(self):
        """
        Initialize GStreamer pipeline for RTSP push.
        
        This pipeline takes raw H264 NAL units and pushes to RTSP.
        No encoding - Isaac ROS H264 encoder already encoded the data.
        """
        # Simple pipeline: raw H264 -> parse -> RTSP
        pipeline_str = (
            f"appsrc name=src format=time is-live=true block=true do-timestamp=true "
            f"caps=video/x-h264,stream-format=byte-stream,alignment=au ! "
            f"h264parse config-interval=1 ! "
            f"rtspclientsink location={self.rtsp_url} protocols=tcp latency=0"
        )
        
        logger.info(f"Creating GStreamer pipeline for RTSP push")
        logger.debug(f"Pipeline: {pipeline_str}")
        
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsrc = self.pipeline.get_by_name('src')
            
            if not self.appsrc:
                logger.error("Failed to get appsrc element from pipeline")
                return
            
            self.appsrc.set_property('format', Gst.Format.TIME)
            self.appsrc.set_property('is-live', True)
            self.appsrc.set_property('block', True)
            
            # Connect to bus messages
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect('message', self._on_bus_message)
            
            # Start GLib main loop
            self.glib_loop = GLib.MainLoop()
            self.glib_thread = threading.Thread(target=self.glib_loop.run, daemon=True)
            self.glib_thread.start()
            
            # Start pipeline
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                logger.error("Failed to start GStreamer pipeline")
                return
            
            self._pipeline_running = True
            logger.info("GStreamer RTSP pipeline started")
            
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
    
    def _start_encoder(self):
        """
        Start the Isaac ROS H264 encoder node.
        
        The encoder subscribes to the source image topic and publishes
        H264 compressed images to the h264_topic.
        """
        logger.info(f"Starting Isaac ROS H264 encoder for {self.source_topic}")
        
        # Build encoder command
        cmd = [
            "ros2", "run", "isaac_ros_h264_encoder", "encoder_node",
            "--ros-args",
            "-r", f"image_raw:={self.source_topic}",
            "-r", f"image_compressed:={self.h264_topic}",
            "-p", f"input_width:={self.width}",
            "-p", f"input_height:={self.height}",
            "-p", "config:=pframe_cqp",  # Use P-frame constant QP preset
            "-p", "qp:=20",              # Quality factor
            "-p", "iframe_interval:=30", # I-frame every 30 frames (1 sec at 30fps)
        ]
        
        try:
            self._encoder_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            logger.info(f"Isaac ROS H264 encoder started (PID: {self._encoder_process.pid})")
        except Exception as e:
            logger.error(f"Failed to start encoder: {e}")
            raise
    
    def _stop_encoder(self):
        """Stop the Isaac ROS H264 encoder node."""
        if self._encoder_process:
            logger.info("Stopping Isaac ROS H264 encoder")
            self._encoder_process.terminate()
            try:
                self._encoder_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._encoder_process.kill()
            self._encoder_process = None
    
    def _setup_ros_subscription(self):
        """Set up ROS2 subscription to H264 compressed image topic."""
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        
        self.h264_sub = self.create_subscription(
            CompressedImage,
            self.h264_topic,
            self._h264_callback,
            sensor_qos
        )
        logger.info(f"Subscribed to H264 topic: {self.h264_topic}")
    
    def _start_http_server(self):
        """Start HTTP control server."""
        try:
            HTTPHandler.bridge_node = self
            self.http_server = HTTPServer(('0.0.0.0', self.http_port), HTTPHandler)
            self.http_thread = threading.Thread(
                target=self.http_server.serve_forever,
                daemon=True
            )
            self.http_thread.start()
            logger.info(f"HTTP server started on port {self.http_port}")
        except Exception as e:
            logger.error(f"Failed to start HTTP server: {e}")
    
    def _h264_callback(self, msg: CompressedImage):
        """
        Process incoming H264 compressed image and push to RTSP.
        
        The CompressedImage from Isaac ROS H264 encoder contains raw H264 NAL units.
        """
        if self._shutdown_requested or not self._pipeline_running:
            return
        
        if not self.appsrc:
            return
        
        try:
            # Get raw H264 data from CompressedImage
            h264_data = bytes(msg.data)
            
            if len(h264_data) == 0:
                self.error_count += 1
                return
            
            # Push H264 NAL units to GStreamer
            self._push_h264(h264_data)
            
            # Update stats
            self.frame_count += 1
            current_time = time.time()
            self.last_frame_time = current_time
            self.frame_times.append(current_time)
            
            # Keep only last second for FPS calculation
            cutoff = current_time - 1.0
            self.frame_times = [t for t in self.frame_times if t > cutoff]
            
        except Exception as e:
            self.error_count += 1
            logger.error(f"Error processing H264 frame: {e}")
    
    def _push_h264(self, h264_data: bytes):
        """Push H264 NAL units to the GStreamer pipeline."""
        try:
            buf = Gst.Buffer.new_wrapped(h264_data)
            ret = self.appsrc.emit('push-buffer', buf)
            
            if ret != Gst.FlowReturn.OK:
                logger.warning(f"Failed to push H264 buffer: {ret}")
                self.error_count += 1
                
        except Exception as e:
            logger.error(f"Error pushing H264: {e}")
            self.error_count += 1
    
    def get_image_topics(self) -> List[str]:
        """Get list of available image topics."""
        topics = []
        try:
            topic_list = self.get_topic_names_and_types()
            for topic_name, topic_types in topic_list:
                if 'sensor_msgs/msg/Image' in topic_types:
                    topics.append(topic_name)
            topics.sort()
            logger.info(f"Found {len(topics)} image topics")
        except Exception as e:
            logger.error(f"Error getting topics: {e}")
        return topics
    
    def get_stats(self) -> VideoStats:
        """Get current video statistics."""
        fps = len(self.frame_times) if self.frame_times else 0.0
        encoder_running = self._encoder_process is not None and self._encoder_process.poll() is None
        
        return VideoStats(
            source_topic=self.source_topic,
            h264_topic=self.h264_topic,
            frame_count=self.frame_count,
            error_count=self.error_count,
            fps=fps,
            last_frame_time=self.last_frame_time,
            streaming=self._pipeline_running,
            encoder_running=encoder_running,
        )
    
    def switch_source_topic(self, new_topic: str) -> bool:
        """
        Switch to a different source image topic.
        
        This restarts the Isaac ROS H264 encoder with the new topic.
        """
        with self._topic_lock:
            if new_topic == self.source_topic:
                logger.info(f"Already using {new_topic}")
                return True
            
            logger.info(f"Switching source topic: {self.source_topic} -> {new_topic}")
            
            try:
                # Stop current encoder
                self._stop_encoder()
                
                # Update topic
                self.source_topic = new_topic
                
                # Start encoder with new topic
                self._start_encoder()
                
                logger.info(f"Successfully switched to {new_topic}")
                return True
                
            except Exception as e:
                logger.error(f"Failed to switch topic: {e}")
                return False
    
    def shutdown(self):
        """Clean shutdown."""
        logger.info("Shutting down Isaac H264 RTSP Bridge...")
        self._shutdown_requested = True
        
        # Stop encoder
        self._stop_encoder()
        
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
        
        logger.info("Shutdown complete")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Isaac ROS H264 to RTSP Bridge'
    )
    parser.add_argument(
        '--source-topic',
        type=str,
        default=DEFAULT_SOURCE_TOPIC,
        help=f'Source image topic (default: {DEFAULT_SOURCE_TOPIC})'
    )
    parser.add_argument(
        '--h264-topic',
        type=str,
        default=DEFAULT_H264_TOPIC,
        help=f'H264 compressed image topic (default: {DEFAULT_H264_TOPIC})'
    )
    parser.add_argument(
        '--rtsp-url',
        type=str,
        default=DEFAULT_RTSP_URL,
        help=f'RTSP URL to push to (default: {DEFAULT_RTSP_URL})'
    )
    parser.add_argument(
        '--http-port',
        type=int,
        default=DEFAULT_HTTP_PORT,
        help=f'HTTP control port (default: {DEFAULT_HTTP_PORT})'
    )
    parser.add_argument(
        '--width',
        type=int,
        default=DEFAULT_WIDTH,
        help=f'Video width (default: {DEFAULT_WIDTH})'
    )
    parser.add_argument(
        '--height',
        type=int,
        default=DEFAULT_HEIGHT,
        help=f'Video height (default: {DEFAULT_HEIGHT})'
    )
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    # Create bridge node
    bridge = IsaacH264RtspBridge(
        source_topic=args.source_topic,
        h264_topic=args.h264_topic,
        rtsp_url=args.rtsp_url,
        http_port=args.http_port,
        width=args.width,
        height=args.height,
    )
    
    # Handle shutdown signals
    def signal_handler(sig, frame):
        logger.info("Received shutdown signal")
        bridge.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Spin
    try:
        logger.info("Isaac H264 RTSP Bridge running. Press Ctrl+C to stop.")
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
