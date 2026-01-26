#!/usr/bin/env python3
"""
ROS-to-RTSP Video Bridge

Subscribes to ROS image topics and streams them via FFmpeg to MediaMTX RTSP server.
Runs inside the Isaac ROS container.

Usage:
    python3 ros_video_bridge.py --topic /zed/zed_node/rgb/image_rect_color --stream live

Target: Python 3.10+ | ROS 2 Humble | NVIDIA Jetson Orin Nano
"""

import argparse
import logging
import signal
import subprocess
import sys
import threading
import time
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
    import numpy as np
except ImportError as e:
    print(f"ERROR: ROS 2 dependencies not available: {e}")
    print("This script must run inside a ROS 2 environment")
    sys.exit(1)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger("ros_video_bridge")


class ROSVideoPublisher(Node):
    """ROS 2 node that subscribes to image topics and publishes to RTSP via FFmpeg."""

    def __init__(
        self,
        topic: str,
        rtsp_url: str,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
    ):
        super().__init__('ros_video_bridge')
        
        self.topic = topic
        self.rtsp_url = rtsp_url
        self.width = width
        self.height = height
        self.fps = fps
        
        self.bridge = CvBridge()
        self.ffmpeg_process: Optional[subprocess.Popen] = None
        self.frame_count = 0
        self.last_frame_time = time.time()
        self.lock = threading.Lock()
        
        # Start FFmpeg process
        self._start_ffmpeg()
        
        # Subscribe to image topic
        self.subscription = self.create_subscription(
            Image,
            topic,
            self._image_callback,
            10  # QoS depth
        )
        
        # Status timer
        self.status_timer = self.create_timer(5.0, self._status_callback)
        
        logger.info(f"Subscribed to {topic}, publishing to {rtsp_url}")

    def _start_ffmpeg(self):
        """Start FFmpeg process for RTSP streaming."""
        # FFmpeg command for RTSP streaming via MediaMTX
        # Uses pipe input for raw video frames
        # Try hardware encoder first, fall back to software
        cmd = [
            'ffmpeg',
            '-y',  # Overwrite output
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{self.width}x{self.height}',
            '-r', str(self.fps),
            '-i', '-',  # Input from pipe
            '-c:v', 'libx264',  # Use software encoder (more compatible)
            '-preset', 'ultrafast',  # Fastest preset for low latency
            '-tune', 'zerolatency',  # Ultra low latency tuning
            '-x264-params', 'keyint=60:min-keyint=60',  # GOP settings
            '-b:v', '4M',  # 4 Mbps bitrate
            '-maxrate', '4M',
            '-bufsize', '500k',  # Small buffer for low latency
            '-f', 'rtsp',
            '-rtsp_transport', 'tcp',
            self.rtsp_url
        ]
        
        logger.info(f"Starting FFmpeg: {' '.join(cmd)}")
        
        try:
            self.ffmpeg_process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            logger.info("FFmpeg started successfully")
        except Exception as e:
            logger.error(f"Failed to start FFmpeg: {e}")
            raise

    def _image_callback(self, msg: Image):
        """Process incoming ROS image and send to FFmpeg."""
        if self.ffmpeg_process is None or self.ffmpeg_process.stdin is None:
            return
        
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize if needed
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                cv_image = cv2.resize(cv_image, (self.width, self.height))
            
            # Write to FFmpeg stdin
            with self.lock:
                self.ffmpeg_process.stdin.write(cv_image.tobytes())
                self.frame_count += 1
                self.last_frame_time = time.time()
                
        except Exception as e:
            logger.error(f"Error processing image: {e}")

    def _status_callback(self):
        """Log status periodically."""
        elapsed = time.time() - self.last_frame_time
        if elapsed > 2.0:
            logger.warning(f"No frames received for {elapsed:.1f}s")
        else:
            logger.info(f"Frames processed: {self.frame_count}")

    def shutdown(self):
        """Cleanup resources."""
        logger.info("Shutting down ROS Video Bridge...")
        
        if self.ffmpeg_process:
            with self.lock:
                if self.ffmpeg_process.stdin:
                    self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.terminate()
                try:
                    self.ffmpeg_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.ffmpeg_process.kill()
        
        logger.info("Shutdown complete")


def main():
    parser = argparse.ArgumentParser(description='ROS-to-RTSP Video Bridge')
    parser.add_argument('--topic', default='/zed/zed_node/rgb/image_rect_color',
                        help='ROS image topic to subscribe to')
    parser.add_argument('--stream', default='live',
                        help='MediaMTX stream name')
    parser.add_argument('--host', default='localhost',
                        help='MediaMTX host')
    parser.add_argument('--port', default=8554, type=int,
                        help='MediaMTX RTSP port')
    parser.add_argument('--width', default=1280, type=int,
                        help='Output video width')
    parser.add_argument('--height', default=720, type=int,
                        help='Output video height')
    parser.add_argument('--fps', default=30, type=int,
                        help='Output video FPS')
    
    args = parser.parse_args()
    
    rtsp_url = f"rtsp://{args.host}:{args.port}/{args.stream}"
    
    logger.info("=" * 60)
    logger.info("ROS-to-RTSP Video Bridge")
    logger.info("=" * 60)
    logger.info(f"  Topic: {args.topic}")
    logger.info(f"  Stream: {rtsp_url}")
    logger.info(f"  Resolution: {args.width}x{args.height}@{args.fps}fps")
    logger.info("=" * 60)
    
    rclpy.init()
    
    node = ROSVideoPublisher(
        topic=args.topic,
        rtsp_url=rtsp_url,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )
    
    def signal_handler(sig, frame):
        logger.info("Received shutdown signal")
        node.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
