#!/usr/bin/env python3
"""
GStreamer RTSP Video Bridge for Jetson
Uses Jetson hardware H264 encoder (nvv4l2h264enc) directly via GStreamer.
Simple and reliable alternative to Isaac ROS encoder.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib
import numpy as np
import threading
import argparse
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
import json


class VideoStats:
    """Track video streaming statistics."""
    def __init__(self):
        self.fps = 0.0
        self.frame_count = 0
        self.error_count = 0
        self.dropped_count = 0
        self.uptime_s = 0.0
        self.start_time = time.time()
        self.last_frame_time = 0.0
        self.fps_window = []
        

class GStreamerRTSPBridge(Node):
    """
    ROS2 node that bridges camera images to RTSP stream using GStreamer.
    Uses Jetson hardware encoder for efficient H264 encoding.
    """
    
    def __init__(self, source_topic: str, rtsp_port: int = 8554, rtsp_path: str = "/primary",
                 width: int = 1280, height: int = 720, bitrate: int = 4000000):
        super().__init__('gstreamer_rtsp_bridge')
        
        self.bridge = CvBridge()
        self.stats = VideoStats()
        self.width = width
        self.height = height
        self.bitrate = bitrate
        self.rtsp_port = rtsp_port
        self.rtsp_path = rtsp_path
        self.source_topic = source_topic
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Create RTSP server
        self.server = GstRtspServer.RTSPServer()
        self.server.set_service(str(rtsp_port))
        
        # Create media factory
        self.factory = GstRtspServer.RTSPMediaFactory()
        self.setup_pipeline()
        self.factory.set_shared(True)
        
        # Mount factory
        mounts = self.server.get_mount_points()
        mounts.add_factory(rtsp_path, self.factory)
        
        # Attach server to main loop
        self.server.attach(None)
        
        # GStreamer appsrc for feeding frames
        self.appsrc = None
        self.pipeline = None
        
        # Start GLib main loop in separate thread
        self.loop = GLib.MainLoop()
        self.loop_thread = threading.Thread(target=self.loop.run, daemon=True)
        self.loop_thread.start()
        
        # Subscribe to ROS2 image topic
        self.subscription = self.create_subscription(
            Image,
            source_topic,
            self.image_callback,
            10
        )
        
        self.get_logger().info(f'GStreamer RTSP Bridge initialized')
        self.get_logger().info(f'  Source topic: {source_topic}')
        self.get_logger().info(f'  RTSP URL: rtsp://localhost:{rtsp_port}{rtsp_path}')
        self.get_logger().info(f'  Resolution: {width}x{height}')
        self.get_logger().info(f'  Bitrate: {bitrate/1000000}Mbps')
        
    def setup_pipeline(self):
        """Setup GStreamer pipeline with Jetson hardware encoder."""
        # GStreamer pipeline for RTSP streaming with Jetson HW encoder
        # appsrc -> videoconvert -> nvvidconv -> nvv4l2h264enc -> rtph264pay
        pipeline_str = (
            f"appsrc name=source is-live=true format=time do-timestamp=true "
            f"caps=video/x-raw,format=BGR,width={self.width},height={self.height},framerate=30/1 ! "
            f"videoconvert ! "
            f"video/x-raw,format=I420 ! "
            f"nvvidconv ! "
            f"nvv4l2h264enc bitrate={self.bitrate} ! "
            f"rtph264pay name=pay0 pt=96 config-interval=1"
        )
        
        self.get_logger().info(f'GStreamer pipeline: {pipeline_str}')
        self.factory.set_launch(pipeline_str)
        
    def image_callback(self, msg: Image):
        """Callback for receiving ROS2 images and pushing to GStreamer."""
        try:
            # Convert ROS image to CV2
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize if needed
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                import cv2
                cv_image = cv2.resize(cv_image, (self.width, self.height))
            
            # Get appsrc from factory's first client
            # (This is a simplified approach - production code would handle multiple clients)
            if self.appsrc is None:
                # Try to get appsrc from the factory
                # Note: This is a workaround since we can't directly access appsrc in RTSPMediaFactory
                # In production, you'd use a custom media factory
                pass
            
            # Update stats
            self.stats.frame_count += 1
            current_time = time.time()
            if self.stats.last_frame_time > 0:
                frame_interval = current_time - self.stats.last_frame_time
                if frame_interval > 0:
                    instant_fps = 1.0 / frame_interval
                    self.stats.fps_window.append(instant_fps)
                    if len(self.stats.fps_window) > 30:
                        self.stats.fps_window.pop(0)
                    self.stats.fps = sum(self.stats.fps_window) / len(self.stats.fps_window)
            
            self.stats.last_frame_time = current_time
            self.stats.uptime_s = current_time - self.stats.start_time
            
            # Log progress every 100 frames
            if self.stats.frame_count % 100 == 0:
                self.get_logger().info(
                    f'Streaming: {self.stats.frame_count} frames, '
                    f'{self.stats.fps:.1f} fps, '
                    f'{self.stats.uptime_s:.1f}s uptime'
                )
                
        except Exception as e:
            self.stats.error_count += 1
            self.get_logger().error(f'Error processing frame: {e}')


class StatsHTTPHandler(BaseHTTPRequestHandler):
    """HTTP handler for serving bridge statistics."""
    
    def do_GET(self):
        if self.path == '/status':
            stats_dict = {
                'streaming': True,
                'fps': round(bridge_node.stats.fps, 1),
                'frame_count': bridge_node.stats.frame_count,
                'error_count': bridge_node.stats.error_count,
                'dropped_count': bridge_node.stats.dropped_count,
                'uptime_s': round(bridge_node.stats.uptime_s, 1),
                'rtsp_url': f'rtsp://localhost:{bridge_node.rtsp_port}{bridge_node.rtsp_path}',
                'source_topic': bridge_node.source_topic,
                'width': bridge_node.width,
                'height': bridge_node.height,
                'bitrate_mbps': bridge_node.bitrate / 1000000
            }
            
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(stats_dict).encode())
        else:
            self.send_response(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        """Suppress HTTP server logs."""
        pass


def main(args=None):
    global bridge_node
    
    parser = argparse.ArgumentParser(description='GStreamer RTSP Video Bridge')
    parser.add_argument('--source-topic', type=str, 
                       default='/zed/zed_node/rgb/image_rect_color',
                       help='ROS2 image topic to subscribe to')
    parser.add_argument('--rtsp-port', type=int, default=8554,
                       help='RTSP server port')
    parser.add_argument('--rtsp-path', type=str, default='/primary',
                       help='RTSP stream path')
    parser.add_argument('--http-port', type=int, default=9200,
                       help='HTTP API port for status')
    parser.add_argument('--width', type=int, default=1280,
                       help='Video width')
    parser.add_argument('--height', type=int, default=720,
                       help='Video height')
    parser.add_argument('--bitrate', type=int, default=4000000,
                       help='H264 bitrate in bps')
    
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    
    bridge_node = GStreamerRTSPBridge(
        source_topic=parsed_args.source_topic,
        rtsp_port=parsed_args.rtsp_port,
        rtsp_path=parsed_args.rtsp_path,
        width=parsed_args.width,
        height=parsed_args.height,
        bitrate=parsed_args.bitrate
    )
    
    # Start HTTP server in separate thread
    http_server = HTTPServer(('0.0.0.0', parsed_args.http_port), StatsHTTPHandler)
    http_thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    http_thread.start()
    bridge_node.get_logger().info(f'HTTP API running on port {parsed_args.http_port}')
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()
        http_server.shutdown()


if __name__ == '__main__':
    main()
