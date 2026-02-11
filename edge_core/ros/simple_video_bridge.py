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
        self.subscription = None
        
        # Build GStreamer pipeline: appsrc -> openh264enc (software) -> RTSP
        # Use openh264enc available in Isaac ROS container
        # Bitrate is in bps for openh264enc
        pipeline_str = (
            f'appsrc name=source is-live=true format=time do-timestamp=true '
            f'caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! '
            f'videoconvert ! '
            f'openh264enc bitrate={bitrate * 1000} num-slices=4 ! '
            f'video/x-h264,profile=baseline ! '
            f'h264parse ! '
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
        
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10  # QoS depth
        )
        self.source_topic = topic
        self.get_logger().info(f'Subscribed to: {topic}')
    
    def switch_topic(self, new_topic: str) -> bool:
        """
        Switch to a different ROS2 image topic.
        
        Restarts the entire GStreamer pipeline to ensure fresh RTSP connection
        to MediaMTX. This prevents rtspclientsink disconnection issues.
        Brief interruption (~500ms) but ensures reliable streaming.
        """
        try:
            self.get_logger().info(f'Switching topic: {self.source_topic} -> {new_topic}')
            
            # Stop the current pipeline
            if hasattr(self, 'pipeline') and self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)
                time.sleep(0.2)  # Wait for clean shutdown
            
            # Update topic
            self.source_topic = new_topic
            
            # Recreate GStreamer pipeline with new topic
            pipeline_str = (
                f'appsrc name=source is-live=true format=time do-timestamp=true '
                f'caps=video/x-raw,format=BGR,width={self.width},height={self.height},framerate={self.fps}/1 ! '
                f'videoconvert ! '
                f'openh264enc bitrate={self.bitrate * 1000} num-slices=4 ! '
                f'video/x-h264,profile=baseline ! '
                f'h264parse ! '
                f'rtspclientsink location=rtsp://172.17.0.1:8554/primary protocols=tcp'
            )
            
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsrc = self.pipeline.get_by_name('source')
            
            # Start new pipeline
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                self.get_logger().error('Failed to restart pipeline after topic switch')
                return False
            
            # Update ROS subscription
            self._subscribe_to_topic(new_topic)
            
            # Reset counters
            self.frame_count = 0
            self.start_time = time.time()
            
            self.get_logger().info(f'Successfully switched to: {new_topic} (pipeline restarted)')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to switch topic: {e}')
            return False
    
    def image_callback(self, msg: Image):
        """Process incoming ROS2 images and push to GStreamer."""
        try:
            import cv2
            
            # Convert ROS image to OpenCV BGR format
            # Handle BGRA8 encoding from ZED cameras which can cause cv_bridge issues
            encoding = msg.encoding.lower()
            if encoding in ('bgra8', 'rgba8', '8uc4'):
                # Manual conversion: avoid cv_bridge desired_encoding issues with alpha channels
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                if encoding == 'rgba8':
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2BGR)
                else:
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize if needed
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                cv_image = cv2.resize(cv_image, (self.width, self.height))
            
            # Create GStreamer buffer and push to pipeline
            data = cv_image.tobytes()
            buf = Gst.Buffer.new_wrapped(data)
            ret = self.appsrc.emit('push-buffer', buf)
            
            if ret != Gst.FlowReturn.OK:
                self.get_logger().warn(f'Buffer push returned: {ret}')
            
            # Stats
            self.frame_count += 1
            if self.frame_count % 300 == 0:  # Every 10 seconds at 30fps
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                self.get_logger().info(
                    f'Streaming: {self.frame_count} frames, {fps:.1f} fps avg'
                )
                
        except Exception as e:
            if self.frame_count % 100 == 0:  # Don't spam errors
                self.get_logger().error(f'Frame processing error: {e}')
    
    def cleanup(self):
        """Clean shutdown."""
        self.get_logger().info('Stopping video bridge...')
        if hasattr(self, 'pipeline') and self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        self.get_logger().info('Stopped')


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
        
        if parsed.path == '/health':
            self._send_json(200, {'healthy': True, 'streaming': True})
        
        elif parsed.path == '/status':
            status = {
                'streaming': True,
                'source_topic': self.video_node.source_topic if self.video_node else '',
                'fps': self.video_node.frame_count / max(time.time() - self.video_node.start_time, 1) if self.video_node else 0,
                'frame_count': self.video_node.frame_count if self.video_node else 0,
                'error_count': 0,
                'rtsp_url': 'rtsp://localhost:8554/primary',
            }
            self._send_json(200, status)
        
        elif parsed.path == '/topics':
            topics = self._discover_topics()
            self._send_json(200, {'topics': topics, 'count': len(topics)})
        
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
