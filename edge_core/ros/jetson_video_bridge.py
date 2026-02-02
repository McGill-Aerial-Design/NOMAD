#!/usr/bin/env python3
"""
Jetson Hardware Video Bridge - Clean & Efficient

Uses Jetson's native nvv4l2h264enc hardware encoder for zero-overhead streaming.
This encoder uses dedicated hardware (not GPU/CPU), so it's completely resource-free.

Architecture:
    ROS2 Image -> OpenCV -> GStreamer (nvv4l2h264enc) -> RTSP -> MediaMTX
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import subprocess
import signal
import sys
import argparse
import time

class JetsonVideoStreamer(Node):
    """Minimal ROS2 node for hardware-encoded video streaming."""
    
    def __init__(self, source_topic: str, rtsp_url: str, width: int = 1280, height: int = 720, 
                 bitrate: int = 4000000, framerate: int = 30):
        super().__init__('jetson_video_streamer')
        
        self.bridge = CvBridge()
        self.width = width
        self.height = height
        self.frame_count = 0
        self.start_time = time.time()
        
        # Build GStreamer pipeline using Jetson hardware encoder
        # appsrc -> nvvideoconvert -> nvv4l2h264enc (HW encoder) -> h264parse -> rtspclientsink
        self.gst_pipeline = (
            f"appsrc name=source is-live=true format=time do-timestamp=true "
            f"caps=video/x-raw,format=BGR,width={width},height={height},framerate={framerate}/1 ! "
            f"nvvideoconvert ! "
            f"video/x-raw(memory:NVMM) ! "
            f"nvv4l2h264enc bitrate={bitrate} insert-sps-pps=true insert-vui=true idrinterval=30 ! "
            f"h264parse ! "
            f"rtspclientsink location={rtsp_url} protocols=tcp"
        )
        
        self.get_logger().info(f"Starting Jetson HW encoder pipeline")
        self.get_logger().info(f"  Source: {source_topic}")
        self.get_logger().info(f"  RTSP: {rtsp_url}")
        self.get_logger().info(f"  Resolution: {width}x{height}@{framerate}fps")
        self.get_logger().info(f"  Bitrate: {bitrate/1000000}Mbps")
        
        # Start GStreamer pipeline
        try:
            self.gst_process = subprocess.Popen(
                ['gst-launch-1.0', '-e'] + self.gst_pipeline.split(),
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.get_logger().info("GStreamer pipeline started")
        except Exception as e:
            self.get_logger().error(f"Failed to start GStreamer: {e}")
            sys.exit(1)
        
        # Subscribe to ROS2 image topic
        self.subscription = self.create_subscription(
            Image,
            source_topic,
            self.image_callback,
            10  # QoS depth
        )
        
        self.get_logger().info("Jetson Video Streamer ready!")
        
    def image_callback(self, msg: Image):
        """Process incoming ROS2 images and feed to GStreamer."""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize if needed
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                import cv2
                cv_image = cv2.resize(cv_image, (self.width, self.height))
            
            # Send to GStreamer (write raw BGR data to stdin)
            self.gst_process.stdin.write(cv_image.tobytes())
            self.gst_process.stdin.flush()
            
            # Stats
            self.frame_count += 1
            if self.frame_count % 300 == 0:  # Every 10 seconds at 30fps
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed
                self.get_logger().info(
                    f"Streaming: {self.frame_count} frames, {fps:.1f} fps avg"
                )
                
        except Exception as e:
            if self.frame_count % 100 == 0:  # Don't spam errors
                self.get_logger().error(f"Frame processing error: {e}")
    
    def cleanup(self):
        """Clean shutdown of GStreamer pipeline."""
        self.get_logger().info("Stopping video streamer...")
        if hasattr(self, 'gst_process') and self.gst_process:
            self.gst_process.terminate()
            self.gst_process.wait(timeout=5)
        self.get_logger().info("Stopped")


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print("\nShutting down...")
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    parser = argparse.ArgumentParser(description='Jetson Hardware Video Streamer')
    parser.add_argument('--source-topic', type=str, 
                       default='/zed/zed_node/rgb/image_rect_color',
                       help='ROS2 image topic to subscribe to')
    parser.add_argument('--rtsp-url', type=str, 
                       default='rtsp://172.17.0.1:8554/primary',
                       help='RTSP destination URL')
    parser.add_argument('--width', type=int, default=1280,
                       help='Output video width')
    parser.add_argument('--height', type=int, default=720,
                       help='Output video height')
    parser.add_argument('--bitrate', type=int, default=4000000,
                       help='H264 bitrate in bps')
    parser.add_argument('--framerate', type=int, default=30,
                       help='Target framerate')
    
    parsed_args = parser.parse_args()
    
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create and run streamer
    streamer = JetsonVideoStreamer(
        source_topic=parsed_args.source_topic,
        rtsp_url=parsed_args.rtsp_url,
        width=parsed_args.width,
        height=parsed_args.height,
        bitrate=parsed_args.bitrate,
        framerate=parsed_args.framerate
    )
    
    try:
        rclpy.spin(streamer)
    except KeyboardInterrupt:
        pass
    finally:
        streamer.cleanup()
        streamer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
