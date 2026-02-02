"""
Video Stream Manager - Zero-Copy GStreamer Pipeline

Manages persistent video bridge instances for low-latency ROS-to-RTSP streaming.
Uses the new nomad_video_bridge.py with NVENC hardware encoding.

Architecture:
    ZED Camera -> ROS Topic (NVMM) -> GStreamer Bridge (NVENC) -> MediaMTX -> Mission Planner

Key Features:
- Two persistent bridge instances (primary, secondary) - always running
- Dynamic topic switching via HTTP API (no process restart)
- ~150ms glass-to-glass latency with hardware encoding
- Overlay support for object detection visualization

Runs on Jetson Edge Core.
"""

import logging
import subprocess
import threading
import time
import os
import json
from typing import List, Dict, Optional
from dataclasses import dataclass, asdict
from urllib.request import urlopen, Request
from urllib.error import URLError
from urllib.parse import quote

logger = logging.getLogger("edge_core.video_manager")

# Default ROS topics to try for auto-start (in order of preference)
DEFAULT_ZED_TOPICS = [
    "/zed/zed_node/rgb/image_rect_color",
    "/zed/zed_node/left/image_rect_color", 
    "/zed/zed_node/stereo/image_rect_color",
]

# Bridge instance configuration
BRIDGE_INSTANCES = {
    "primary": {
        "http_port": 9100,
        "default_topic": "/zed/zed_node/rgb/image_rect_color",
        "description": "Main camera view for Mission Planner",
    },
    "secondary": {
        "http_port": 9101,
        "default_topic": "/zed/zed_node/depth/depth_registered",
        "description": "Secondary view (depth/gimbal)",
    },
}


@dataclass
class StreamInfo:
    """Information about an active video stream."""
    stream_name: str
    topic: str
    http_port: int
    width: int
    height: int
    fps: int
    rtsp_url: str
    bridge_pid: Optional[int] = None
    started_at: float = 0.0
    overlay_enabled: bool = True
    
    def to_dict(self) -> dict:
        return asdict(self)


class VideoStreamManager:
    """
    Manages persistent video bridge instances with zero-copy GStreamer pipelines.
    
    Architecture:
    - Two persistent bridge instances (primary, secondary) run inside Docker container
    - Each bridge uses nomad_video_bridge.py with NVENC hardware encoding
    - Topic switching is done via HTTP API calls to the bridge (no restart needed)
    - RTSP URLs stay constant, only the content changes
    
    This replaces the old approach of:
    1. Starting/stopping FFmpeg processes
    2. TCP pipe between ROS bridge and FFmpeg
    3. Process restarts on topic switch
    """
    
    def __init__(self, container_name="nomad_isaac_ros_32"):
        self.container_name = container_name
        self.streams: Dict[str, StreamInfo] = {}
        self.lock = threading.RLock()
        self.mediamtx_host = "localhost"
        self.mediamtx_port = 8554
        self._auto_started = False
        self._container_ready = False
        self._persistent_bridges_started = False
        
        logger.info(f"VideoStreamManager initialized for container: {container_name}")
        logger.info("Using zero-copy GStreamer pipeline with NVENC hardware encoding")

    def set_container_name(self, name: str) -> None:
        """Update the container name (useful when container is discovered dynamically)."""
        self.container_name = name
        logger.info(f"VideoStreamManager container updated to: {name}")

    def is_container_running(self) -> bool:
        """Check if the Isaac ROS container is running."""
        try:
            result = subprocess.run(
                ["docker", "ps", "--filter", f"name={self.container_name}", "--format", "{{.Names}}"],
                capture_output=True,
                text=True,
                timeout=5
            )
            running = self.container_name in result.stdout
            self._container_ready = running
            return running
        except Exception as e:
            logger.error(f"Error checking container status: {e}")
            return False

    def wait_for_container(self, timeout: int = 60) -> bool:
        """Wait for the container to be running."""
        logger.info(f"Waiting for container '{self.container_name}' (max {timeout}s)...")
        start = time.time()
        while time.time() - start < timeout:
            if self.is_container_running():
                logger.info(f"Container ready after {int(time.time() - start)}s")
                return True
            time.sleep(2)
        logger.warning(f"Container not ready after {timeout}s")
        return False

    def start_persistent_bridges(self) -> Dict[str, bool]:
        """
        Start the two persistent video bridge instances inside the Docker container.
        
        These bridges run continuously and handle topic switching via HTTP API.
        The RTSP stream URLs remain constant:
        - primary: rtsp://localhost:8554/primary
        - secondary: rtsp://localhost:8554/secondary
        
        Returns:
            Dict mapping instance name to start success status
        """
        if self._persistent_bridges_started:
            logger.info("Persistent bridges already started")
            return {"primary": True, "secondary": True}
        
        if not self.is_container_running():
            logger.warning("Cannot start bridges: container not running")
            return {"primary": False, "secondary": False}
        
        results = {}
        
        # Choose bridge script - FFmpeg (h264_v4l2m2m) for hardware encoding, GStreamer as fallback
        # FFmpeg bridge is preferred as it reliably uses Jetson's V4L2 hardware encoder
        use_ffmpeg = os.environ.get("NOMAD_VIDEO_USE_FFMPEG", "true").lower() == "true"
        
        if use_ffmpeg:
            bridge_script_name = 'nomad_video_bridge_ffmpeg.py'
            logger.info("Using FFmpeg video bridge (h264_v4l2m2m hardware encoder)")
        else:
            bridge_script_name = 'nomad_video_bridge.py'
            logger.info("Using GStreamer video bridge (software encoder fallback)")
        
        bridge_script = os.path.join(os.path.dirname(__file__), bridge_script_name)
        if os.path.exists(bridge_script):
            try:
                # Copy to same destination name for compatibility
                subprocess.run(
                    ["docker", "cp", bridge_script, f"{self.container_name}:/tmp/nomad_video_bridge.py"],
                    capture_output=True,
                    timeout=5
                )
                logger.info("Copied nomad_video_bridge.py to container")
            except Exception as e:
                logger.error(f"Failed to copy bridge script: {e}")
                return {"primary": False, "secondary": False}
        else:
            logger.error(f"Bridge script not found: {bridge_script}")
            return {"primary": False, "secondary": False}
        
        # Start each bridge instance
        for instance_name, config in BRIDGE_INSTANCES.items():
            success = self._start_bridge_instance(
                instance_name=instance_name,
                topic=config["default_topic"],
                http_port=config["http_port"],
            )
            results[instance_name] = success
            
            if success:
                # Track the stream
                rtsp_url = f"rtsp://{self.mediamtx_host}:{self.mediamtx_port}/{instance_name}"
                self.streams[instance_name] = StreamInfo(
                    stream_name=instance_name,
                    topic=config["default_topic"],
                    http_port=config["http_port"],
                    width=1280,
                    height=720,
                    fps=30,
                    rtsp_url=rtsp_url,
                    bridge_pid=1,  # Dummy PID (docker exec -d)
                    started_at=time.time(),
                    overlay_enabled=True,
                )
            
            time.sleep(2)  # Allow bridge to initialize
        
        self._persistent_bridges_started = all(results.values())
        return results

    def _start_bridge_instance(
        self,
        instance_name: str,
        topic: str,
        http_port: int,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        bitrate: int = 4000,
    ) -> bool:
        """Start a single bridge instance inside the container."""
        try:
            cmd = [
                "docker", "exec", "-d", self.container_name,
                "bash", "-c",
                f"source /opt/ros/humble/setup.bash && "
                f"source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null ; "
                f"python3 /tmp/nomad_video_bridge.py "
                f"--instance '{instance_name}' "
                f"--topic '{topic}' "
                f"--host '{self.mediamtx_host}' "
                f"--port {self.mediamtx_port} "
                f"--width {width} "
                f"--height {height} "
                f"--fps {fps} "
                f"--bitrate {bitrate} "
                f"--http-port {http_port} "
                f"> /tmp/nomad_bridge_{instance_name}.log 2>&1"
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                logger.info(f"Bridge '{instance_name}' started (HTTP port: {http_port})")
                return True
            else:
                logger.error(f"Failed to start bridge '{instance_name}': {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Error starting bridge '{instance_name}': {e}")
            return False

    def switch_video_source(self, instance: str, topic: str) -> bool:
        """
        Switch a bridge instance to a different ROS topic.
        
        This uses the bridge's HTTP control API to change the subscription
        without restarting the pipeline. The RTSP stream stays up.
        
        Args:
            instance: Bridge instance name (primary, secondary)
            topic: New ROS image topic to subscribe to
            
        Returns:
            True if switch was successful
        """
        with self.lock:
            stream = self.streams.get(instance)
            if not stream:
                logger.error(f"Stream '{instance}' not found")
                return False
            
            if stream.topic == topic:
                logger.info(f"'{instance}' already subscribed to {topic}")
                return True
            
            try:
                # Call bridge HTTP API to switch topic (URL-encode topic for safety)
                url = f"http://localhost:{stream.http_port}/switch_topic?topic={quote(topic, safe='')}"
                req = Request(url, method='POST')
                
                with urlopen(req, timeout=5) as response:
                    data = json.loads(response.read().decode())
                    
                if data.get("success"):
                    stream.topic = data.get("topic", topic)
                    logger.info(f"'{instance}' switched to topic: {stream.topic}")
                    return True
                else:
                    logger.error(f"Failed to switch topic: {data}")
                    return False
                    
            except URLError as e:
                logger.error(f"HTTP error switching topic: {e}")
                return False
            except Exception as e:
                logger.error(f"Error switching topic: {e}")
                return False

    def set_overlay_enabled(self, instance: str, enabled: bool) -> bool:
        """
        Enable or disable object detection overlay for a bridge instance.
        
        Args:
            instance: Bridge instance name
            enabled: Whether to enable overlay
            
        Returns:
            True if setting was applied
        """
        with self.lock:
            stream = self.streams.get(instance)
            if not stream:
                logger.error(f"Stream '{instance}' not found")
                return False
            
            try:
                url = f"http://localhost:{stream.http_port}/set_overlay?enabled={str(enabled).lower()}"
                req = Request(url, method='POST')
                
                with urlopen(req, timeout=5) as response:
                    data = json.loads(response.read().decode())
                
                stream.overlay_enabled = data.get("overlay_enabled", enabled)
                logger.info(f"'{instance}' overlay: {stream.overlay_enabled}")
                return True
                
            except Exception as e:
                logger.error(f"Error setting overlay: {e}")
                return False

    def get_bridge_status(self, instance: str) -> Optional[Dict]:
        """Get status of a specific bridge instance via its HTTP API."""
        with self.lock:
            stream = self.streams.get(instance)
            if not stream:
                return None
            
            try:
                url = f"http://localhost:{stream.http_port}/status"
                with urlopen(url, timeout=2) as response:
                    return json.loads(response.read().decode())
            except Exception as e:
                logger.debug(f"Failed to get status for '{instance}': {e}")
                return None

    def auto_start_default_stream(self) -> Optional[Dict]:
        """
        Auto-start the persistent video bridges when edge_core initializes.
        
        Returns:
            Stream info dict if successful, None otherwise.
        """
        if self._auto_started:
            logger.info("Auto-start already completed, skipping")
            return None
        
        if not self.is_container_running():
            logger.warning("Cannot auto-start: container not running")
            return None
        
        logger.info("Auto-starting persistent video bridges...")
        results = self.start_persistent_bridges()
        
        if results.get("primary"):
            self._auto_started = True
            stream = self.streams.get("primary")
            if stream:
                logger.info(f"Auto-start successful: {stream.rtsp_url}")
                return stream.to_dict()
        
        logger.warning("Auto-start failed")
        return None

    def list_topics(self) -> List[str]:
        """List available image topics from ROS 2."""
        try:
            cmd = [
                "docker", "exec", self.container_name,
                "bash", "-c",
                "source /opt/ros/humble/setup.bash && ros2 topic list -t 2>/dev/null"
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode != 0:
                logger.error(f"Failed to list topics: {result.stderr}")
                return []
                
            topics = []
            for line in result.stdout.splitlines():
                parts = line.split()
                if len(parts) >= 2:
                    topic, type_ = parts[0], parts[1].strip('[]')
                    # Filter for image topics
                    if "sensor_msgs/msg/Image" in type_ or "CompressedImage" in type_:
                        topics.append(topic)
                        
            return sorted(topics)
            
        except Exception as e:
            logger.error(f"Error listing topics: {e}")
            return []

    def list_streams(self) -> List[Dict]:
        """List all active streams."""
        with self.lock:
            return [stream.to_dict() for stream in self.streams.values()]

    def get_stream(self, stream_name: str) -> Optional[Dict]:
        """Get info about a specific stream."""
        with self.lock:
            stream = self.streams.get(stream_name)
            return stream.to_dict() if stream else None

    # Legacy compatibility methods
    
    def start_stream(
        self,
        stream_name: str,
        topic: str,
        width: int = 1280,
        height: int = 720,
        fps: int = 30
    ) -> Dict:
        """
        Legacy method for starting streams.
        
        For backward compatibility, this maps to the new persistent bridge architecture:
        - 'zed', 'primary', 'live' -> primary bridge
        - 'secondary', 'depth', 'gimbal' -> secondary bridge
        - Other names -> creates a new bridge instance (deprecated behavior)
        """
        # Map legacy names to new persistent instances
        primary_aliases = {'zed', 'primary', 'live', 'zed_left', 'dynamic'}
        secondary_aliases = {'secondary', 'depth', 'gimbal', 'zed_depth'}
        
        if stream_name.lower() in primary_aliases:
            self.switch_video_source("primary", topic)
            return self.get_stream("primary") or {}
        elif stream_name.lower() in secondary_aliases:
            self.switch_video_source("secondary", topic)
            return self.get_stream("secondary") or {}
        else:
            # For other stream names, use primary and switch topic
            logger.warning(f"Legacy stream name '{stream_name}' mapped to 'primary'")
            self.switch_video_source("primary", topic)
            return self.get_stream("primary") or {}

    def stop_stream(self, stream_name: str) -> bool:
        """
        Legacy method for stopping streams.
        
        In the new architecture, bridges are persistent and don't stop.
        This method is kept for API compatibility but doesn't actually stop anything.
        """
        logger.info(f"stop_stream('{stream_name}'): Bridges are now persistent - ignoring stop request")
        return True

    def stop_all_streams(self) -> int:
        """Legacy method - persistent bridges don't stop. Returns 0."""
        logger.info("stop_all_streams(): Bridges are now persistent - ignoring stop request")
        return 0

    def set_use_nvenc(self, use_nvenc: bool) -> None:
        """Legacy method - NVENC is always used in new architecture."""
        logger.info("set_use_nvenc(): NVENC is always enabled in zero-copy architecture")


# Global instance
_video_manager: Optional[VideoStreamManager] = None


def get_video_manager() -> VideoStreamManager:
    """Get the global video manager instance."""
    global _video_manager
    if _video_manager is None:
        # Default to nomad_isaac_ros_32 (the common container name)
        _video_manager = VideoStreamManager(container_name="nomad_isaac_ros_32")
    return _video_manager


def init_video_manager(container_name: str = "nomad_isaac_ros_32", auto_start: bool = True) -> VideoStreamManager:
    """
    Initialize the global video manager with specific settings.
    
    Args:
        container_name: Name of the Isaac ROS Docker container
        auto_start: Whether to auto-start persistent bridges when container is ready
        
    Returns:
        The initialized VideoStreamManager instance
    """
    global _video_manager
    _video_manager = VideoStreamManager(container_name=container_name)
    
    # Clean up any legacy video processes from old approach
    _cleanup_legacy_video_processes(container_name)
    
    if auto_start:
        # Start persistent bridges in background thread to not block startup
        def _delayed_auto_start():
            # Wait for container to be ready
            if _video_manager.wait_for_container(timeout=90):
                # Wait a bit more for ZED to publish topics
                logger.info("Container ready, waiting for ZED topics...")
                time.sleep(10)
                _video_manager.start_persistent_bridges()
            else:
                logger.warning("Skipping video auto-start: container not available")
        
        thread = threading.Thread(target=_delayed_auto_start, daemon=True)
        thread.start()
        logger.info("Video bridge auto-start scheduled in background")
    
    return _video_manager


def _cleanup_legacy_video_processes(container_name: str) -> None:
    """
    Clean up any legacy video bridge or FFmpeg processes from old static approach.
    
    This ensures the new persistent bridge architecture has a clean slate.
    """
    logger.info("Cleaning up any legacy video processes...")
    
    try:
        # Kill any old FFmpeg RTSP processes on host
        subprocess.run(
            ["pkill", "-f", "ffmpeg.*rtsp://localhost:8554"],
            capture_output=True,
            timeout=5
        )
    except Exception as e:
        logger.debug(f"FFmpeg cleanup: {e}")
    
    try:
        # Kill any old video bridge processes in container
        subprocess.run(
            ["docker", "exec", container_name, "pkill", "-f", "ros_video_bridge"],
            capture_output=True,
            timeout=5
        )
    except Exception as e:
        logger.debug(f"Container video bridge cleanup: {e}")
    
    try:
        # Also kill any nomad_video_bridge processes (from previous runs)
        subprocess.run(
            ["docker", "exec", container_name, "pkill", "-f", "nomad_video_bridge"],
            capture_output=True,
            timeout=5
        )
    except Exception as e:
        logger.debug(f"Container nomad_video_bridge cleanup: {e}")
    
    logger.info("Legacy video process cleanup complete")
