#!/usr/bin/env python3
"""
NOMAD FFmpeg Video Bridge with Hardware H.264 Encoding

Uses FFmpeg with h264_v4l2m2m (V4L2 Memory-to-Memory) hardware encoder
for optimal performance on NVIDIA Jetson platforms.

Architecture:
    ZED Camera -> ROS Topic -> Python Bridge -> FFmpeg (h264_v4l2m2m) -> RTSP (MediaMTX)

Key Features:
- Hardware H.264 encoding via V4L2 M2M on Jetson
- Low-latency streaming (~150-200ms glass-to-glass)
- ROS2 image subscription with real-time processing
- HTTP control API for topic switching

Target: Python 3.10+ | ROS 2 Humble | NVIDIA Jetson Orin | FFmpeg with h264_v4l2m2m

Usage:
    python3 nomad_video_bridge_ffmpeg.py --instance primary --topic /zed/zed_node/rgb/image_rect_color
"""

import argparse
import json
import logging
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, asdict
from enum import Enum
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Optional, Dict, Any
from urllib.parse import urlparse, parse_qs

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger("nomad_video_bridge_ffmpeg")

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    import cv2
    import numpy as np
except ImportError as e:
    print(f"ERROR: ROS 2 dependencies not available: {e}")
    print("This script must run inside a ROS 2 environment")
    sys.exit(1)

# ========================================
# Constants
# ========================================

DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 30
DEFAULT_BITRATE = 4000  # kbps


class StreamState(Enum):
    """Stream pipeline state."""
    STOPPED = "stopped"
    STARTING = "starting"
    PLAYING = "playing"
    ERROR = "error"


@dataclass
class BridgeStatus:
    """Bridge status information."""
    instance: str
    state: StreamState
    current_topic: str
    width: int
    height: int
    fps: int
    bitrate_kbps: int
    frame_count: int
    error_count: int
    dropped_count: int
    uptime_s: float
    encoder: str
    
    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d['state'] = self.state.value
        return d


class FFmpegVideoBridge(Node):
    """
    ROS2 to RTSP video bridge using FFmpeg with h264_v4l2m2m hardware encoder.
    
    This provides hardware-accelerated H.264 encoding on NVIDIA Jetson platforms
    through the V4L2 Memory-to-Memory interface, which is more reliable than
    trying to use nvv4l2h264enc through GStreamer.
    """
    
    def __init__(
        self,
        instance_name: str,
        initial_topic: str,
        rtsp_host: str = "localhost",
        rtsp_port: int = 8554,
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        fps: int = DEFAULT_FPS,
        bitrate: int = DEFAULT_BITRATE,
        http_port: int = 0,
    ):
        super().__init__(f'nomad_ffmpeg_bridge_{instance_name}')
        
        # Configuration
        self.instance_name = instance_name
        self.rtsp_url = f"rtsp://{rtsp_host}:{rtsp_port}/{instance_name}"
        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate = bitrate
        self.http_port = http_port
        
        # Topic management
        self.current_topic = initial_topic
        self._topic_lock = threading.RLock()
        
        # FFmpeg subprocess
        self._ffmpeg_proc: Optional[subprocess.Popen] = None
        self._pipeline_running = False
        self._encoder_type = "unknown"
        
        # Statistics
        self.frame_count = 0
        self.error_count = 0
        self.dropped_count = 0
        self.start_time = time.time()
        self._last_frame_time = time.time()
        
        # Shutdown handling
        self._shutdown_requested = False
        
        # Initialize FFmpeg process
        self._start_ffmpeg()
        
        # Subscribe to ROS topic
        self._setup_subscription()
        
        # Start status logging timer
        self.create_timer(5.0, self._log_status)
        
        # Start HTTP server if requested
        if http_port > 0:
            self._start_http_server()
        
        logger.info(f"FFmpegVideoBridge '{instance_name}' initialized")
    
    def _check_v4l2m2m_available(self) -> bool:
        """Check if h264_v4l2m2m encoder is available."""
        try:
            result = subprocess.run(
                ["ffmpeg", "-hide_banner", "-encoders"],
                capture_output=True, text=True, timeout=5
            )
            return "h264_v4l2m2m" in result.stdout
        except Exception:
            return False
    
    def _start_ffmpeg(self):
        """Start FFmpeg subprocess for video encoding."""
        try:
            # Check for hardware encoder
            use_v4l2m2m = self._check_v4l2m2m_available()
            
            if use_v4l2m2m:
                logger.info("Using h264_v4l2m2m hardware encoder")
                self._encoder_type = "h264_v4l2m2m (hardware)"
                encoder_opts = [
                    "-c:v", "h264_v4l2m2m",
                    "-b:v", f"{self.bitrate}k",
                ]
            else:
                logger.warning("h264_v4l2m2m not available, falling back to libx264")
                self._encoder_type = "libx264 (software)"
                encoder_opts = [
                    "-c:v", "libx264",
                    "-preset", "ultrafast",
                    "-tune", "zerolatency",
                    "-b:v", f"{self.bitrate}k",
                ]
            
            # Build FFmpeg command
            cmd = [
                "ffmpeg",
                "-hide_banner",
                "-loglevel", "warning",
                # Input: raw video from stdin
                "-f", "rawvideo",
                "-pix_fmt", "bgr24",
                "-s", f"{self.width}x{self.height}",
                "-r", str(self.fps),
                "-i", "-",
                # Encoder options
                *encoder_opts,
                "-g", str(self.fps),  # Keyframe interval
                "-pix_fmt", "yuv420p",
                # Output: RTSP to MediaMTX
                "-f", "rtsp",
                "-rtsp_transport", "tcp",
                self.rtsp_url
            ]
            
            logger.info(f"Starting FFmpeg: {' '.join(cmd[:15])}...")
            
            self._ffmpeg_proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=self.width * self.height * 3 * 2,  # Buffer 2 frames
            )
            
            # Start stderr reader thread
            threading.Thread(target=self._read_stderr, daemon=True).start()
            
            self._pipeline_running = True
            logger.info(f"FFmpeg started successfully ({self._encoder_type})")
            
        except Exception as e:
            logger.error(f"Failed to start FFmpeg: {e}")
            self._pipeline_running = False
    
    def _read_stderr(self):
        """Read and log FFmpeg stderr output."""
        if not self._ffmpeg_proc:
            return
        try:
            for line in self._ffmpeg_proc.stderr:
                if self._shutdown_requested:
                    break
                line_str = line.decode().strip()
                if line_str:
                    logger.debug(f"FFmpeg: {line_str}")
        except Exception:
            pass
    
    def _setup_subscription(self):
        """Set up ROS2 image subscription."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.subscription = self.create_subscription(
            Image,
            self.current_topic,
            self._image_callback,
            qos
        )
        logger.info(f"Subscribed to: {self.current_topic}")
    
    def _image_callback(self, msg: Image):
        """Handle incoming image messages."""
        if self._shutdown_requested or not self._pipeline_running:
            return
        
        try:
            # Convert ROS image to numpy
            frame = self._convert_image(msg)
            if frame is None:
                return
            
            # Resize if needed
            if frame.shape[:2] != (self.height, self.width):
                frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
            
            # Ensure BGR format and contiguous
            if len(frame.shape) == 2:  # Grayscale
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            
            if not frame.flags['C_CONTIGUOUS']:
                frame = np.ascontiguousarray(frame)
            
            # Send to FFmpeg
            self._push_frame(frame)
            
        except Exception as e:
            logger.error(f"Error processing image: {e}")
            self.error_count += 1
    
    def _convert_image(self, msg: Image) -> Optional[np.ndarray]:
        """Convert ROS image message to numpy array."""
        try:
            encoding = msg.encoding.lower()
            
            if 'bgr' in encoding:
                frame = np.frombuffer(msg.data, dtype=np.uint8)
                frame = frame.reshape((msg.height, msg.width, 3))
            elif 'rgb' in encoding:
                frame = np.frombuffer(msg.data, dtype=np.uint8)
                frame = frame.reshape((msg.height, msg.width, 3))
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif 'mono8' in encoding or '8uc1' in encoding:
                frame = np.frombuffer(msg.data, dtype=np.uint8)
                frame = frame.reshape((msg.height, msg.width))
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif '16' in encoding:
                # Depth image - normalize to 8-bit
                raw = np.frombuffer(msg.data, dtype=np.uint16)
                raw = raw.reshape((msg.height, msg.width))
                frame = cv2.normalize(raw, None, 0, 255, cv2.NORM_MINMAX)
                frame = frame.astype(np.uint8)
                frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            elif '32fc1' in encoding:
                # Float depth
                raw = np.frombuffer(msg.data, dtype=np.float32)
                raw = raw.reshape((msg.height, msg.width))
                valid = np.isfinite(raw)
                raw[~valid] = 0
                frame = cv2.normalize(raw, None, 0, 255, cv2.NORM_MINMAX)
                frame = frame.astype(np.uint8)
                frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            else:
                logger.warning(f"Unsupported encoding: {encoding}")
                return None
            
            return frame
            
        except Exception as e:
            logger.error(f"Conversion error: {e}")
            return None
    
    def _push_frame(self, frame: np.ndarray):
        """Push a frame to FFmpeg stdin."""
        if not self._ffmpeg_proc or not self._pipeline_running:
            return
        
        try:
            # Check if process is still running
            if self._ffmpeg_proc.poll() is not None:
                logger.error("FFmpeg process terminated unexpectedly")
                self._pipeline_running = False
                return
            
            # Write frame to FFmpeg stdin
            self._ffmpeg_proc.stdin.write(frame.tobytes())
            
            self.frame_count += 1
            self._last_frame_time = time.time()
            
        except BrokenPipeError:
            logger.error("FFmpeg pipe broken, restarting...")
            self._restart_ffmpeg()
        except Exception as e:
            logger.error(f"Error pushing frame: {e}")
            self.error_count += 1
    
    def _restart_ffmpeg(self):
        """Restart FFmpeg subprocess."""
        logger.info("Restarting FFmpeg process...")
        self._stop_ffmpeg()
        time.sleep(0.5)
        self._start_ffmpeg()
    
    def _stop_ffmpeg(self):
        """Stop FFmpeg subprocess."""
        if self._ffmpeg_proc:
            self._pipeline_running = False
            try:
                self._ffmpeg_proc.stdin.close()
            except Exception:
                pass
            try:
                self._ffmpeg_proc.terminate()
                self._ffmpeg_proc.wait(timeout=2)
            except Exception:
                self._ffmpeg_proc.kill()
            self._ffmpeg_proc = None
    
    def _log_status(self):
        """Log periodic status."""
        elapsed = time.time() - self._last_frame_time
        if elapsed > 2.0:
            logger.warning(f"No frames received for {elapsed:.1f}s on {self.current_topic}")
        else:
            fps_actual = self.frame_count / max(1, time.time() - self.start_time)
            logger.info(f"[{self.instance_name}] Frames: {self.frame_count}, FPS: {fps_actual:.1f}")
    
    def switch_topic(self, new_topic: str) -> bool:
        """Switch to a different ROS topic."""
        with self._topic_lock:
            if new_topic == self.current_topic:
                return True
            
            logger.info(f"Switching topic: {self.current_topic} -> {new_topic}")
            
            # Destroy old subscription
            self.destroy_subscription(self.subscription)
            
            # Create new subscription
            self.current_topic = new_topic
            self._setup_subscription()
            
            return True
    
    def get_status(self) -> BridgeStatus:
        """Get current bridge status."""
        return BridgeStatus(
            instance=self.instance_name,
            state=StreamState.PLAYING if self._pipeline_running else StreamState.STOPPED,
            current_topic=self.current_topic,
            width=self.width,
            height=self.height,
            fps=self.fps,
            bitrate_kbps=self.bitrate,
            frame_count=self.frame_count,
            error_count=self.error_count,
            dropped_count=self.dropped_count,
            uptime_s=time.time() - self.start_time,
            encoder=self._encoder_type,
        )
    
    def shutdown(self):
        """Clean shutdown."""
        logger.info(f"Shutting down bridge '{self.instance_name}'")
        self._shutdown_requested = True
        self._stop_ffmpeg()
    
    # ========================================
    # HTTP Control Server
    # ========================================
    
    def _start_http_server(self):
        """Start lightweight HTTP server for control API."""
        bridge = self
        
        class ControlHandler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                pass
            
            def _send_json(self, data: dict, status: int = 200):
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(data).encode())
            
            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == '/status':
                    self._send_json(bridge.get_status().to_dict())
                elif parsed.path == '/health':
                    self._send_json({"healthy": bridge._pipeline_running})
                else:
                    self._send_json({"error": "Not found"}, 404)
            
            def do_POST(self):
                parsed = urlparse(self.path)
                params = parse_qs(parsed.query)
                
                if parsed.path == '/switch_topic':
                    topic = params.get('topic', [None])[0]
                    if topic:
                        success = bridge.switch_topic(topic)
                        self._send_json({
                            "success": success,
                            "topic": bridge.current_topic
                        })
                    else:
                        self._send_json({"error": "No topic specified"}, 400)
                else:
                    self._send_json({"error": "Not found"}, 404)
        
        def run_server():
            server = HTTPServer(('0.0.0.0', bridge.http_port), ControlHandler)
            bridge.http_port = server.server_port
            logger.info(f"HTTP control server on port {bridge.http_port}")
            server.serve_forever()
        
        threading.Thread(target=run_server, daemon=True).start()


def main():
    parser = argparse.ArgumentParser(description='NOMAD FFmpeg Video Bridge')
    parser.add_argument('--instance', type=str, required=True, help='Instance name (primary/secondary)')
    parser.add_argument('--topic', type=str, required=True, help='ROS image topic')
    parser.add_argument('--host', type=str, default='localhost', help='MediaMTX host')
    parser.add_argument('--port', type=int, default=8554, help='MediaMTX port')
    parser.add_argument('--width', type=int, default=1280, help='Output width')
    parser.add_argument('--height', type=int, default=720, help='Output height')
    parser.add_argument('--fps', type=int, default=30, help='Output FPS')
    parser.add_argument('--bitrate', type=int, default=4000, help='Bitrate in kbps')
    parser.add_argument('--http-port', type=int, default=0, help='HTTP control port')
    
    args = parser.parse_args()
    
    logger.info("=" * 60)
    logger.info("NOMAD FFmpeg Video Bridge (h264_v4l2m2m)")
    logger.info("=" * 60)
    logger.info(f"  Instance: {args.instance}")
    logger.info(f"  Topic: {args.topic}")
    logger.info(f"  RTSP: rtsp://{args.host}:{args.port}/{args.instance}")
    logger.info(f"  Resolution: {args.width}x{args.height}@{args.fps}fps")
    logger.info(f"  Bitrate: {args.bitrate} kbps")
    logger.info("=" * 60)
    
    rclpy.init()
    
    node = FFmpegVideoBridge(
        instance_name=args.instance,
        initial_topic=args.topic,
        rtsp_host=args.host,
        rtsp_port=args.port,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate=args.bitrate,
        http_port=args.http_port,
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
