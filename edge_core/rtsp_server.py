#!/usr/bin/env python3
"""
NOMAD Edge Core - Python GStreamer RTSP Server

Provides a robust RTSP server for ZED camera streaming using GstRtspServer.
Supports multiple simultaneous viewers (Mission Planner, VLC, phone, etc.).

Target: Python 3.10+ | NVIDIA Jetson Orin Nano | GStreamer 1.x
Stream URL: rtsp://<JETSON_IP>:8554/zed

Usage:
    python3 -m edge_core.rtsp_server
    # or
    ./rtsp_server.py
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import os

# GObject Introspection imports
try:
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstRtspServer', '1.0')
    from gi.repository import Gst, GstRtspServer, GLib
except ImportError as e:
    print(f"ERROR: GStreamer Python bindings not available: {e}")
    print("Install with: sudo apt install python3-gi gstreamer1.0-rtsp")
    sys.exit(1)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger("edge_core.rtsp_server")


class ZEDRtspServer:
    """
    GStreamer RTSP Server for ZED Camera.
    
    Creates an RTSP endpoint at rtsp://<ip>:8554/zed that:
    - Captures from ZED camera (/dev/video0)
    - Crops to left eye only (for stereo camera)
    - Encodes with H.264 zerolatency for minimum lag
    - Supports multiple simultaneous viewers
    """
    
    DEFAULT_PORT = 8554
    DEFAULT_MOUNT = "/zed"
    DEFAULT_DEVICE = "/dev/video0"
    
    def __init__(
        self,
        port: int = DEFAULT_PORT,
        mount_point: str = DEFAULT_MOUNT,
        device: str = DEFAULT_DEVICE,
        width: int = 1280,
        height: int = 720,
        framerate: int = 30,
        bitrate: int = 4000,
    ):
        """
        Initialize RTSP server.
        
        Args:
            port: RTSP server port (default 8554)
            mount_point: Stream mount point (default /zed)
            device: V4L2 device path (default /dev/video0)
            width: Output video width (default 1280)
            height: Output video height (default 720)
            framerate: Target framerate (default 30)
            bitrate: H.264 bitrate in kbps (default 4000)
        """
        self.port = port
        self.mount_point = mount_point
        self.device = device
        self.width = width
        self.height = height
        self.framerate = framerate
        self.bitrate = bitrate
        
        self._server = None
        self._loop = None
        self._running = False
        
    def _build_pipeline(self) -> str:
        """
        Build GStreamer pipeline string for ZED camera.
        
        ZED 2i outputs side-by-side stereo at 2560x720 (for HD720 mode).
        We crop to left eye (1280x720) for single-view streaming.
        """
        # ZED camera capture - stereo side-by-side
        # For HD720: input is 2560x720, we crop left 1280x720
        pipeline = (
            f"v4l2src device={self.device} ! "
            f"video/x-raw,width=2560,height={self.height},framerate={self.framerate}/1 ! "
            f"videocrop left=0 right=1280 ! "  # Crop to left eye
            f"videoconvert ! "
            f"x264enc tune=zerolatency bitrate={self.bitrate} "
            f"speed-preset=ultrafast sliced-threads=true "
            f"key-int-max=15 bframes=0 ! "
            f"video/x-h264,profile=baseline ! "
            f"rtph264pay name=pay0 pt=96 config-interval=1"
        )
        return pipeline
    
    def _build_pipeline_nvidia(self) -> str:
        """
        Build hardware-accelerated pipeline using NVIDIA NVENC.
        Falls back to software encoding if NVENC unavailable.
        """
        # Try NVIDIA hardware encoder first (nvv4l2h264enc on Jetson)
        pipeline = (
            f"v4l2src device={self.device} ! "
            f"video/x-raw,width=2560,height={self.height},framerate={self.framerate}/1 ! "
            f"videocrop left=0 right=1280 ! "
            f"nvvidconv ! "
            f"video/x-raw(memory:NVMM) ! "
            f"nvv4l2h264enc bitrate={self.bitrate * 1000} "
            f"preset-level=1 iframeinterval=15 insert-sps-pps=true ! "
            f"video/x-h264,stream-format=byte-stream ! "
            f"rtph264pay name=pay0 pt=96 config-interval=1"
        )
        return pipeline
        
    def start(self, use_nvidia: bool = True) -> None:
        """
        Start the RTSP server.
        
        Args:
            use_nvidia: Try NVIDIA hardware encoding (default True)
        """
        # Initialize GStreamer
        Gst.init(None)
        
        # Create RTSP server
        self._server = GstRtspServer.RTSPServer()
        self._server.set_service(str(self.port))
        
        # Create media factory
        factory = GstRtspServer.RTSPMediaFactory()
        
        # Try NVIDIA pipeline first, fall back to software
        if use_nvidia:
            try:
                pipeline = self._build_pipeline_nvidia()
                # Test if pipeline parses successfully
                test_pipe = Gst.parse_launch(f"( {pipeline} )")
                if test_pipe:
                    logger.info("Using NVIDIA hardware encoding")
                    del test_pipe
            except Exception as e:
                logger.warning(f"NVIDIA encoding unavailable: {e}")
                logger.info("Falling back to software encoding")
                pipeline = self._build_pipeline()
        else:
            pipeline = self._build_pipeline()
            
        logger.debug(f"Pipeline: {pipeline}")
        factory.set_launch(f"( {pipeline} )")
        
        # Enable multiple clients
        factory.set_shared(True)
        
        # Mount the stream
        mount_points = self._server.get_mount_points()
        mount_points.add_factory(self.mount_point, factory)
        
        # Attach to main context
        self._server.attach(None)
        
        self._running = True
        logger.info(f"RTSP server started at rtsp://0.0.0.0:{self.port}{self.mount_point}")
        logger.info(f"Device: {self.device} | Resolution: {self.width}x{self.height}@{self.framerate}fps")
        
    def run_loop(self) -> None:
        """Run the GLib main loop (blocking)."""
        self._loop = GLib.MainLoop()
        
        # Handle shutdown signals
        def signal_handler(signum, frame):
            logger.info(f"Received signal {signum}, shutting down...")
            self.stop()
            
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        try:
            logger.info("RTSP server running. Press Ctrl+C to stop.")
            self._loop.run()
        except KeyboardInterrupt:
            self.stop()
            
    def stop(self) -> None:
        """Stop the RTSP server."""
        self._running = False
        if self._loop:
            self._loop.quit()
        logger.info("RTSP server stopped")


def get_local_ip() -> str:
    """Get the local IP address for display purposes."""
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="NOMAD ZED Camera RTSP Server",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--port", "-p",
        type=int,
        default=ZEDRtspServer.DEFAULT_PORT,
        help="RTSP server port"
    )
    parser.add_argument(
        "--device", "-d",
        default=ZEDRtspServer.DEFAULT_DEVICE,
        help="V4L2 device path"
    )
    parser.add_argument(
        "--width", "-W",
        type=int,
        default=1280,
        help="Output video width"
    )
    parser.add_argument(
        "--height", "-H",
        type=int,
        default=720,
        help="Output video height"
    )
    parser.add_argument(
        "--framerate", "-f",
        type=int,
        default=30,
        help="Target framerate"
    )
    parser.add_argument(
        "--bitrate", "-b",
        type=int,
        default=4000,
        help="H.264 bitrate (kbps)"
    )
    parser.add_argument(
        "--software",
        action="store_true",
        help="Force software encoding (disable NVENC)"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging"
    )
    
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
        os.environ["GST_DEBUG"] = "2"
    
    # Display banner
    local_ip = get_local_ip()
    print()
    print("=" * 50)
    print("  NOMAD ZED Camera RTSP Server")
    print("  AEAC 2026 - McGill Aerial Design")
    print("=" * 50)
    print(f"  Stream URL: rtsp://{local_ip}:{args.port}/zed")
    print(f"  Device:     {args.device}")
    print(f"  Resolution: {args.width}x{args.height}@{args.framerate}fps")
    print(f"  Bitrate:    {args.bitrate} kbps")
    print("=" * 50)
    print()
    
    # Create and start server
    server = ZEDRtspServer(
        port=args.port,
        device=args.device,
        width=args.width,
        height=args.height,
        framerate=args.framerate,
        bitrate=args.bitrate,
    )
    
    try:
        server.start(use_nvidia=not args.software)
        server.run_loop()
    except Exception as e:
        logger.error(f"Server error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
