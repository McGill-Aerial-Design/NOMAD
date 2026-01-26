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
        tcp_port: int = 9999,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
    ):
        super().__init__('ros_video_bridge')
        
        self.topic = topic
        self.rtsp_url = rtsp_url
        self.tcp_port = tcp_port
        self.width = width
        self.height = height
        self.fps = fps
        
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
        """Start TCP server to send raw frames to host encoder."""
        # We'll send raw BGR frames via TCP to the host
        # The host runs FFmpeg with libx264 to encode and stream
        # TCP server binds to 0.0.0.0:PORT for external access
        import socket
        
        self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tcp_server.bind(('0.0.0.0', self.tcp_port))
        self.tcp_server.listen(1)
        self.tcp_server.setblocking(False)
        self.tcp_client = None
        
        logger.info(f"TCP server started on port {self.tcp_port} - waiting for encoder connection")
        logger.info(f"Run on host: ffmpeg -f rawvideo -pix_fmt bgr24 -s {self.width}x{self.height} -r {self.fps} -i tcp://localhost:{self.tcp_port} -c:v libx264 -preset ultrafast -tune zerolatency -f rtsp {self.rtsp_url}")
        
        # Set ffmpeg_process to a dummy for compatibility
        self.ffmpeg_process = type('obj', (object,), {'stdin': None, 'poll': lambda: None, 'stderr': None})()
    
    def _get_encoder_command(self) -> list:
        """Not used with TCP approach."""
        return []

    def _image_callback(self, msg: Image):
        """Process incoming ROS image and send via TCP."""
        # Try to accept new client connection
        if self.tcp_client is None:
            try:
                self.tcp_client, addr = self.tcp_server.accept()
                self.tcp_client.setblocking(True)
                logger.info(f"Encoder connected from {addr}")
            except BlockingIOError:
                # No client connected yet
                return
            except Exception as e:
                logger.debug(f"No encoder connected yet: {e}")
                return
        
        try:
            # Convert ROS image to numpy array directly (avoid cv_bridge ABI issues)
            # Determine encoding and convert to BGR
            encoding = msg.encoding
            height = msg.height
            width = msg.width
            
            # Convert bytes to numpy array
            if encoding in ('bgr8', 'rgb8', 'bgra8', 'rgba8'):
                channels = 4 if 'a' in encoding else 3
                dtype = np.uint8
                img_array = np.frombuffer(msg.data, dtype=dtype).reshape(height, width, channels)
                
                # Convert to BGR if needed
                if encoding == 'rgb8':
                    cv_image = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
                elif encoding == 'rgba8':
                    cv_image = cv2.cvtColor(img_array, cv2.COLOR_RGBA2BGR)
                elif encoding == 'bgra8':
                    cv_image = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)
                else:  # bgr8
                    cv_image = img_array
            elif encoding == '32FC1':
                # Depth image (32-bit float, single channel)
                # Convert to colorized visualization
                dtype = np.float32
                depth_array = np.frombuffer(msg.data, dtype=dtype).reshape(height, width)
                
                # Normalize to 0-255 range for visualization
                # Filter out invalid depths (inf, nan)
                valid_depth = np.isfinite(depth_array)
                if np.any(valid_depth):
                    min_depth = np.min(depth_array[valid_depth])
                    max_depth = np.max(depth_array[valid_depth])
                    
                    # Normalize and convert to uint8
                    if max_depth > min_depth:
                        normalized = (depth_array - min_depth) / (max_depth - min_depth)
                    else:
                        normalized = np.zeros_like(depth_array)
                    
                    # Set invalid depths to 0
                    normalized[~valid_depth] = 0
                    
                    # Convert to uint8
                    depth_uint8 = (normalized * 255).astype(np.uint8)
                    
                    # Apply colormap for better visualization
                    cv_image = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
                else:
                    # All invalid depths - return black image
                    cv_image = np.zeros((height, width, 3), dtype=np.uint8)
            else:
                logger.warning(f"Unsupported encoding: {encoding}, skipping frame")
                return
            
            # Resize if needed
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                cv_image = cv2.resize(cv_image, (self.width, self.height))
            
            # Send via TCP
            with self.lock:
                try:
                    self.tcp_client.sendall(cv_image.tobytes())
                    self.frame_count += 1
                    self.last_frame_time = time.time()
                except (BrokenPipeError, ConnectionResetError):
                    logger.warning("Encoder disconnected, waiting for reconnection...")
                    self.tcp_client.close()
                    self.tcp_client = None
                
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
        
        with self.lock:
            if hasattr(self, 'tcp_client') and self.tcp_client:
                try:
                    self.tcp_client.close()
                except:
                    pass
            if hasattr(self, 'tcp_server') and self.tcp_server:
                try:
                    self.tcp_server.close()
                except:
                    pass
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
    parser.add_argument('--tcp-port', default=9999, type=int,
                        help='TCP port for raw video frames')
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
    logger.info(f"  TCP Port: {args.tcp_port}")
    logger.info(f"  Resolution: {args.width}x{args.height}@{args.fps}fps")
    logger.info("=" * 60)
    
    rclpy.init()
    
    node = ROSVideoPublisher(
        topic=args.topic,
        rtsp_url=rtsp_url,
        tcp_port=args.tcp_port,
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
