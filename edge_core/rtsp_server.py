#!/usr/bin/env python3
"""
NOMAD Edge Core - Python GStreamer RTSP Server

Provides a robust RTSP server for ZED camera streaming using GstRtspServer.
Supports multiple simultaneous viewers (Mission Planner, VLC, phone, etc.).

Target: Python 3.10+ | NVIDIA Jetson Orin Nano | GStreamer 1.x

Available Streams:
  rtsp://<JETSON_IP>:8554/zed       - Left camera only (default for navigation)
  rtsp://<JETSON_IP>:8554/zed-right - Right camera only
  rtsp://<JETSON_IP>:8554/zed-both  - Side-by-side stereo (wide angle)

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
from typing import Optional, Dict

# GObject Introspection imports
try:
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstRtspServer', '1.0')
    from gi.repository import Gst, GstRtspServer, GLib
except ImportError as e:
    print(f"ERROR: GStreamer Python bindings not available: {e}")
    print("Install with: sudo apt install python3-gi gstreamer1.0-rtsp gir1.2-gst-rtsp-server-1.0")
    sys.exit(1)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger("edge_core.rtsp_server")


# ============================================================
# Pipeline Builders for Different Camera Views
# ============================================================

def build_software_pipeline_left(device: str, bitrate: int) -> str:
    """Build software encoding pipeline for LEFT camera only (1280x720)."""
    return (
        f"( v4l2src device={device} ! "
        f"video/x-raw,width=2560,height=720,framerate=30/1 ! "
        f"videocrop left=0 right=1280 ! "
        f"videoconvert ! "
        f"x264enc tune=zerolatency bitrate={bitrate} speed-preset=ultrafast key-int-max=15 ! "
        f"h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )"
    )


def build_software_pipeline_right(device: str, bitrate: int) -> str:
    """Build software encoding pipeline for RIGHT camera only (1280x720)."""
    return (
        f"( v4l2src device={device} ! "
        f"video/x-raw,width=2560,height=720,framerate=30/1 ! "
        f"videocrop left=1280 right=0 ! "
        f"videoconvert ! "
        f"x264enc tune=zerolatency bitrate={bitrate} speed-preset=ultrafast key-int-max=15 ! "
        f"h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )"
    )


def build_software_pipeline_both(device: str, bitrate: int) -> str:
    """Build software encoding pipeline for BOTH cameras side-by-side (2560x720)."""
    # Higher bitrate for wider frame
    return (
        f"( v4l2src device={device} ! "
        f"video/x-raw,width=2560,height=720,framerate=30/1 ! "
        f"videoconvert ! "
        f"x264enc tune=zerolatency bitrate={bitrate * 2} speed-preset=ultrafast key-int-max=15 ! "
        f"h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )"
    )


def build_nvidia_pipeline_left(device: str, bitrate: int) -> str:
    """Build NVIDIA hardware encoding pipeline for LEFT camera only."""
    return (
        f"( v4l2src device={device} ! "
        f"video/x-raw,width=2560,height=720,framerate=30/1 ! "
        f"videocrop left=0 right=1280 ! "
        f"nvvidconv ! video/x-raw(memory:NVMM) ! "
        f"nvv4l2h264enc bitrate={bitrate * 1000} preset-level=1 iframeinterval=15 insert-sps-pps=true ! "
        f"h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )"
    )


def build_nvidia_pipeline_right(device: str, bitrate: int) -> str:
    """Build NVIDIA hardware encoding pipeline for RIGHT camera only."""
    return (
        f"( v4l2src device={device} ! "
        f"video/x-raw,width=2560,height=720,framerate=30/1 ! "
        f"videocrop left=1280 right=0 ! "
        f"nvvidconv ! video/x-raw(memory:NVMM) ! "
        f"nvv4l2h264enc bitrate={bitrate * 1000} preset-level=1 iframeinterval=15 insert-sps-pps=true ! "
        f"h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )"
    )


def build_nvidia_pipeline_both(device: str, bitrate: int) -> str:
    """Build NVIDIA hardware encoding pipeline for BOTH cameras side-by-side."""
    return (
        f"( v4l2src device={device} ! "
        f"video/x-raw,width=2560,height=720,framerate=30/1 ! "
        f"nvvidconv ! video/x-raw(memory:NVMM) ! "
        f"nvv4l2h264enc bitrate={bitrate * 2000} preset-level=1 iframeinterval=15 insert-sps-pps=true ! "
        f"h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )"
    )


# Legacy aliases for backward compatibility
def build_software_pipeline(device: str, bitrate: int) -> str:
    """Build software encoding pipeline string (default: left camera)."""
    return build_software_pipeline_left(device, bitrate)


def build_nvidia_pipeline(device: str, bitrate: int) -> str:
    """Build NVIDIA hardware encoding pipeline string (default: left camera)."""
    return build_nvidia_pipeline_left(device, bitrate)


class ZEDRtspServer:
    """
    GStreamer RTSP Server for ZED Camera.
    
    Creates multiple RTSP endpoints for different camera views:
    - /zed       - Left camera only (1280x720) - default for navigation/VIO
    - /zed-right - Right camera only (1280x720)
    - /zed-both  - Side-by-side stereo (2560x720) - wide angle view
    
    Features:
    - Captures from ZED camera (/dev/video0)
    - H.264 zerolatency encoding for minimum lag
    - Supports multiple simultaneous viewers on each stream
    - NVIDIA hardware encoding when available
    """
    
    DEFAULT_PORT = 8554
    DEFAULT_MOUNT = "/zed"
    DEFAULT_DEVICE = "/dev/video0"
    
    # Camera view configurations
    VIEWS = {
        "/zed": ("left", "Left camera (navigation)"),
        "/zed-right": ("right", "Right camera"),
        "/zed-both": ("both", "Side-by-side stereo"),
    }
    
    def __init__(
        self,
        port: int = DEFAULT_PORT,
        mount_point: str = DEFAULT_MOUNT,
        device: str = DEFAULT_DEVICE,
        bitrate: int = 4000,
        multi_stream: bool = True,
    ):
        self.port = port
        self.mount_point = mount_point
        self.device = device
        self.bitrate = bitrate
        self.multi_stream = multi_stream
        
        self._server = None
        self._factories: Dict[str, GstRtspServer.RTSPMediaFactory] = {}
        self._loop = None
        self._running = False
        self._use_nvidia = False
        
    def _build_pipeline(self, view: str) -> Optional[str]:
        """Build pipeline for the specified view."""
        nvidia_builders = {
            "left": build_nvidia_pipeline_left,
            "right": build_nvidia_pipeline_right,
            "both": build_nvidia_pipeline_both,
        }
        software_builders = {
            "left": build_software_pipeline_left,
            "right": build_software_pipeline_right,
            "both": build_software_pipeline_both,
        }
        
        # Try NVIDIA first
        if self._use_nvidia and view in nvidia_builders:
            try:
                pipeline = nvidia_builders[view](self.device, self.bitrate)
                test = Gst.parse_launch(pipeline.strip("() "))
                if test:
                    del test
                    return pipeline
            except Exception:
                pass
        
        # Fall back to software
        if view in software_builders:
            return software_builders[view](self.device, self.bitrate)
        
        return None
        
    def start(self) -> None:
        """Start the RTSP server with multiple camera view streams."""
        # Initialize GStreamer
        Gst.init(None)
        
        # Create RTSP server
        self._server = GstRtspServer.RTSPServer.new()
        self._server.set_service(str(self.port))
        
        # Detect NVIDIA hardware encoding capability
        try:
            nvidia_test = build_nvidia_pipeline_left(self.device, self.bitrate)
            test = Gst.parse_launch(nvidia_test.strip("() "))
            if test:
                self._use_nvidia = True
                logger.info("NVIDIA hardware encoding available")
                del test
        except Exception as e:
            logger.info(f"NVIDIA encoding unavailable, using software: {e}")
            self._use_nvidia = False
        
        # Get mount points
        mounts = self._server.get_mount_points()
        
        # Create factories for each view
        if self.multi_stream:
            for mount_path, (view, description) in self.VIEWS.items():
                pipeline = self._build_pipeline(view)
                if pipeline:
                    factory = GstRtspServer.RTSPMediaFactory.new()
                    factory.set_launch(pipeline)
                    factory.set_shared(True)
                    mounts.add_factory(mount_path, factory)
                    self._factories[mount_path] = factory
                    logger.info(f"  {mount_path} -> {description}")
        else:
            # Single stream mode (backward compatible)
            pipeline = self._build_pipeline("left")
            if pipeline:
                factory = GstRtspServer.RTSPMediaFactory.new()
                factory.set_launch(pipeline)
                factory.set_shared(True)
                mounts.add_factory(self.mount_point, factory)
                self._factories[self.mount_point] = factory
        
        # Attach server to default main context
        self._server.attach(None)
        
        self._running = True
        encoding_type = "NVIDIA hardware" if self._use_nvidia else "software (x264)"
        logger.info(f"RTSP server started on port {self.port}")
        logger.info(f"Device: {self.device} | Bitrate: {self.bitrate} kbps | Encoding: {encoding_type}")
        
    def run_loop(self) -> None:
        """Run the GLib main loop (blocking)."""
        self._loop = GLib.MainLoop.new(None, False)
        
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
    parser.add_argument("--port", "-p", type=int, default=ZEDRtspServer.DEFAULT_PORT, help="RTSP server port")
    parser.add_argument("--device", "-d", default=ZEDRtspServer.DEFAULT_DEVICE, help="V4L2 device path")
    parser.add_argument("--bitrate", "-b", type=int, default=4000, help="H.264 bitrate (kbps)")
    parser.add_argument("--single", "-s", action="store_true", help="Single stream mode (only /zed)")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
        os.environ["GST_DEBUG"] = "3"
    
    # Display banner
    local_ip = get_local_ip()
    print()
    print("=" * 60)
    print("  NOMAD ZED Camera RTSP Server")
    print("  AEAC 2026 - McGill Aerial Design")
    print("=" * 60)
    print()
    print("  Available Streams:")
    print(f"    rtsp://{local_ip}:{args.port}/zed       - Left camera (1280x720)")
    if not args.single:
        print(f"    rtsp://{local_ip}:{args.port}/zed-right - Right camera (1280x720)")
        print(f"    rtsp://{local_ip}:{args.port}/zed-both  - Side-by-side (2560x720)")
    print()
    print(f"  Device:     {args.device}")
    print(f"  Bitrate:    {args.bitrate} kbps")
    print("=" * 60)
    print()
    
    # Create and start server
    server = ZEDRtspServer(
        port=args.port,
        device=args.device,
        bitrate=args.bitrate,
        multi_stream=not args.single,
    )
    
    try:
        server.start()
        server.run_loop()
    except Exception as e:
        logger.error(f"Server error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
