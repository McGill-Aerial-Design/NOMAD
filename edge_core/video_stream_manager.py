"""
Video Stream Manager - Isaac ROS H.264 Streaming

Manages the video relay node running inside the Isaac ROS Docker container
for low-latency ROS-to-RTSP streaming using NVIDIA NVENC hardware encoding.

Architecture:
    ZED Camera -> ROS Topic -> Video Relay Node (NVENC) -> MediaMTX -> Mission Planner

Key Features:
- Single persistent stream with dynamic topic switching
- No RTSP URL change when switching - only content changes
- Hardware H.264 encoding via nvv4l2h264enc (~150ms latency)
- HTTP API control for topic switching
- Multiple viewer support via MediaMTX

Runs on Jetson Edge Core host, controls the relay inside Docker container.
"""

import logging
import subprocess
import threading
import time
import os
import json
from typing import List, Dict, Optional, Any
from dataclasses import dataclass, asdict
from urllib.request import urlopen, Request
from urllib.error import URLError
from urllib.parse import quote

logger = logging.getLogger("edge_core.video_stream_manager")

# Configuration
DEFAULT_CONTAINER_NAME = "nomad_isaac_ros_32"
DEFAULT_RELAY_HTTP_PORT = 9200
DEFAULT_RTSP_URL = "rtsp://172.17.0.1:8554/primary"
DEFAULT_TOPIC = "/zed/zed_node/rgb/image_rect_color"

# Stream settings
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 30
DEFAULT_BITRATE = 4  # Mbps


@dataclass
class StreamStatus:
    """Status information for the video stream."""
    streaming: bool
    current_topic: str
    rtsp_url: str
    fps: float
    frame_count: int
    error_count: int
    dropped_count: int
    uptime_s: float
    width: int
    height: int
    bitrate_mbps: int
    
    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class TopicInfo:
    """Information about an available video topic."""
    name: str
    display_name: str  # Trimmed for UI display
    
    def to_dict(self) -> dict:
        return asdict(self)


def trim_topic_name(topic: str) -> str:
    """
    Trim topic name for display in UI dropdown.
    
    Removes common prefixes:
    - /zed/zed_node/ -> zed:
    - /camera/ -> cam:
    
    Examples:
        /zed/zed_node/rgb/image_rect_color -> zed: rgb/image_rect_color
        /zed/zed_node/left/image_rect_color -> zed: left/image_rect_color
        /zed/zed_node/depth/depth_registered -> zed: depth/depth_registered
    """
    prefixes = [
        ("/zed/zed_node/", "zed: "),
        ("/camera/", "cam: "),
        ("/", ""),
    ]
    
    for prefix, replacement in prefixes:
        if topic.startswith(prefix):
            return replacement + topic[len(prefix):]
    
    return topic


class VideoStreamManager:
    """
    Manages the video streaming pipeline for NOMAD.
    
    This class controls the video relay node running inside the Isaac ROS
    Docker container. The relay node:
    - Subscribes to a ROS2 image topic (ZED camera)
    - Encodes with NVENC hardware (H.264)
    - Streams to MediaMTX RTSP server
    
    Topic switching is done via HTTP API to the relay node, which changes
    the subscription without restarting the pipeline. The RTSP URL stays
    constant - only the content changes.
    """
    
    def __init__(
        self,
        container_name: str = DEFAULT_CONTAINER_NAME,
        relay_http_port: int = DEFAULT_RELAY_HTTP_PORT,
        rtsp_url: str = DEFAULT_RTSP_URL,
        default_topic: str = DEFAULT_TOPIC,
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        fps: int = DEFAULT_FPS,
        bitrate: int = DEFAULT_BITRATE,
    ):
        self.container_name = container_name
        self.relay_http_port = relay_http_port
        self.rtsp_url = rtsp_url
        self.default_topic = default_topic
        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate = bitrate
        
        self._started = False
        self._lock = threading.RLock()
        
        logger.info(f"VideoStreamManager initialized")
        logger.info(f"  Container: {container_name}")
        logger.info(f"  RTSP URL: {rtsp_url}")
        logger.info(f"  Default topic: {default_topic}")

    def is_container_running(self) -> bool:
        """Check if the Isaac ROS container is running."""
        try:
            result = subprocess.run(
                ["docker", "ps", "--filter", f"name={self.container_name}", "--format", "{{.Names}}"],
                capture_output=True,
                text=True,
                timeout=5
            )
            return self.container_name in result.stdout
        except Exception as e:
            logger.error(f"Error checking container status: {e}")
            return False

    def is_relay_running(self) -> bool:
        """Check if the video relay node is running inside the container."""
        try:
            # Check if the HTTP API is responsive
            url = f"http://localhost:{self.relay_http_port}/health"
            with urlopen(url, timeout=2) as response:
                data = json.loads(response.read().decode())
                return data.get("healthy", False)
        except Exception:
            return False

    def start(self) -> bool:
        """
        Start the video streaming pipeline.
        
        Copies the relay script to the container and launches it.
        Returns True if successful.
        """
        with self._lock:
            if self._started and self.is_relay_running():
                logger.info("Video relay already running")
                return True
            
            if not self.is_container_running():
                logger.warning("Cannot start video: container not running")
                return False
            
            # Copy relay script to container
            script_path = os.path.join(os.path.dirname(__file__), "ros", "nomad_video_relay.py")
            if not os.path.exists(script_path):
                logger.error(f"Relay script not found: {script_path}")
                return False
            
            try:
                subprocess.run(
                    ["docker", "cp", script_path, f"{self.container_name}:/tmp/nomad_video_relay.py"],
                    capture_output=True,
                    timeout=10,
                    check=True
                )
                logger.info("Copied video relay script to container")
            except subprocess.CalledProcessError as e:
                logger.error(f"Failed to copy relay script: {e}")
                return False
            
            # Kill any existing relay process
            try:
                subprocess.run(
                    ["docker", "exec", self.container_name, "pkill", "-f", "nomad_video_relay"],
                    capture_output=True,
                    timeout=5
                )
            except Exception:
                pass  # OK if nothing to kill
            
            # Start the relay node
            cmd = [
                "docker", "exec", "-d", self.container_name,
                "bash", "-c",
                f"source /opt/ros/humble/setup.bash && "
                f"source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null ; "
                f"python3 /tmp/nomad_video_relay.py "
                f"--topic '{self.default_topic}' "
                f"--rtsp-url '{self.rtsp_url}' "
                f"--http-port {self.relay_http_port} "
                f"--width {self.width} "
                f"--height {self.height} "
                f"--fps {self.fps} "
                f"--bitrate {self.bitrate} "
                f"> /tmp/nomad_video_relay.log 2>&1"
            ]
            
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                if result.returncode != 0:
                    logger.error(f"Failed to start relay: {result.stderr}")
                    return False
            except Exception as e:
                logger.error(f"Error starting relay: {e}")
                return False
            
            # Wait for relay to be ready
            for i in range(10):
                time.sleep(1)
                if self.is_relay_running():
                    self._started = True
                    logger.info("Video relay started successfully")
                    return True
            
            logger.error("Video relay did not start in time")
            return False

    def stop(self) -> bool:
        """Stop the video streaming pipeline."""
        with self._lock:
            try:
                subprocess.run(
                    ["docker", "exec", self.container_name, "pkill", "-f", "nomad_video_relay"],
                    capture_output=True,
                    timeout=5
                )
                self._started = False
                logger.info("Video relay stopped")
                return True
            except Exception as e:
                logger.error(f"Error stopping relay: {e}")
                return False

    def restart(self) -> bool:
        """Restart the video streaming pipeline."""
        self.stop()
        time.sleep(2)
        return self.start()

    def switch_topic(self, topic: str) -> bool:
        """
        Switch the video stream to a different ROS topic.
        
        This calls the relay's HTTP API to change its subscription.
        The GStreamer pipeline keeps running - only the content changes.
        
        Args:
            topic: Full ROS topic path (e.g., /zed/zed_node/left/image_rect_color)
            
        Returns:
            True if switch was successful
        """
        with self._lock:
            if not self.is_relay_running():
                logger.warning("Cannot switch topic: relay not running")
                return False
            
            try:
                url = f"http://localhost:{self.relay_http_port}/switch?topic={quote(topic, safe='')}"
                req = Request(url, method='POST')
                
                with urlopen(req, timeout=5) as response:
                    data = json.loads(response.read().decode())
                    
                if data.get("success"):
                    logger.info(f"Switched video source to: {topic}")
                    return True
                else:
                    logger.error(f"Failed to switch topic: {data.get('error', 'Unknown error')}")
                    return False
                    
            except URLError as e:
                logger.error(f"HTTP error switching topic: {e}")
                return False
            except Exception as e:
                logger.error(f"Error switching topic: {e}")
                return False

    def list_topics(self) -> List[TopicInfo]:
        """
        List available ROS image topics.
        
        Queries the relay node's HTTP API which uses ros2 topic list
        to find sensor_msgs/Image topics.
        
        Returns:
            List of TopicInfo with name and display_name
        """
        if not self.is_relay_running():
            # Fallback: query topics directly from container
            return self._query_topics_docker()
        
        try:
            url = f"http://localhost:{self.relay_http_port}/topics"
            with urlopen(url, timeout=10) as response:
                data = json.loads(response.read().decode())
                
            topics = []
            for topic in data.get("topics", []):
                topics.append(TopicInfo(
                    name=topic,
                    display_name=trim_topic_name(topic)
                ))
            return topics
            
        except Exception as e:
            logger.error(f"Error listing topics: {e}")
            return self._query_topics_docker()

    def _query_topics_docker(self) -> List[TopicInfo]:
        """Query topics directly via docker exec (fallback)."""
        try:
            cmd = [
                "docker", "exec", self.container_name,
                "bash", "-c",
                "source /opt/ros/humble/setup.bash && ros2 topic list -t 2>/dev/null"
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
            if result.returncode != 0:
                return []
            
            topics = []
            for line in result.stdout.splitlines():
                parts = line.split()
                if len(parts) >= 2:
                    topic, type_ = parts[0], parts[1].strip('[]')
                    if "sensor_msgs/msg/Image" in type_:
                        topics.append(TopicInfo(
                            name=topic,
                            display_name=trim_topic_name(topic)
                        ))
            
            return sorted(topics, key=lambda t: t.name)
            
        except Exception as e:
            logger.error(f"Error querying topics via docker: {e}")
            return []

    def get_status(self) -> StreamStatus:
        """
        Get current stream status.
        
        Queries the relay node's HTTP API for detailed status.
        """
        default_status = StreamStatus(
            streaming=False,
            current_topic="",
            rtsp_url=self.rtsp_url.replace("172.17.0.1", "localhost"),
            fps=0.0,
            frame_count=0,
            error_count=0,
            dropped_count=0,
            uptime_s=0.0,
            width=self.width,
            height=self.height,
            bitrate_mbps=self.bitrate,
        )
        
        if not self.is_relay_running():
            return default_status
        
        try:
            url = f"http://localhost:{self.relay_http_port}/status"
            with urlopen(url, timeout=5) as response:
                data = json.loads(response.read().decode())
            
            return StreamStatus(
                streaming=data.get("streaming", False),
                current_topic=data.get("current_topic", ""),
                rtsp_url=self.rtsp_url.replace("172.17.0.1", "localhost"),
                fps=data.get("fps", 0.0),
                frame_count=data.get("frame_count", 0),
                error_count=data.get("error_count", 0),
                dropped_count=data.get("dropped_count", 0),
                uptime_s=data.get("uptime_s", 0.0),
                width=data.get("width", self.width),
                height=data.get("height", self.height),
                bitrate_mbps=data.get("bitrate_mbps", self.bitrate),
            )
            
        except Exception as e:
            logger.error(f"Error getting status: {e}")
            return default_status

    def get_rtsp_url(self) -> str:
        """Get the constant RTSP URL for the video stream."""
        # Replace Docker host IP with accessible IP
        return self.rtsp_url.replace("172.17.0.1", "localhost")

    def get_logs(self, lines: int = 50) -> str:
        """Get recent logs from the video relay process."""
        try:
            cmd = [
                "docker", "exec", self.container_name,
                "tail", f"-{lines}", "/tmp/nomad_video_relay.log"
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            return result.stdout
        except Exception as e:
            return f"Error getting logs: {e}"


# Global instance
_video_stream_manager: Optional[VideoStreamManager] = None


def get_video_stream_manager() -> Optional[VideoStreamManager]:
    """Get the global video stream manager instance."""
    global _video_stream_manager
    return _video_stream_manager


def init_video_stream_manager(
    container_name: str = DEFAULT_CONTAINER_NAME,
    auto_start: bool = True,
    **kwargs
) -> VideoStreamManager:
    """
    Initialize the global video stream manager.
    
    Args:
        container_name: Name of the Isaac ROS Docker container
        auto_start: Whether to auto-start the relay when container is ready
        **kwargs: Additional arguments passed to VideoStreamManager
        
    Returns:
        The initialized VideoStreamManager instance
    """
    global _video_stream_manager
    _video_stream_manager = VideoStreamManager(container_name=container_name, **kwargs)
    
    if auto_start:
        # Start in background thread to not block startup
        def _delayed_start():
            # Wait for container to be ready
            for i in range(45):  # Wait up to 90 seconds
                if _video_stream_manager.is_container_running():
                    logger.info("Container ready, starting video relay...")
                    time.sleep(5)  # Wait for ZED to initialize
                    _video_stream_manager.start()
                    return
                time.sleep(2)
            logger.warning("Container not ready, video relay not started")
        
        thread = threading.Thread(target=_delayed_start, daemon=True)
        thread.start()
        logger.info("Video stream manager auto-start scheduled")
    
    return _video_stream_manager
