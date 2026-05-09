"""
Video Stream Manager - Simple Video Bridge for NOMAD

Manages the simple video bridge running inside the Isaac ROS Docker container
for low-latency ROS-to-RTSP streaming using software H.264 encoding.

Architecture:
    ZED Camera (ROS2) -> x264enc (software, zerolatency) -> RTSP -> MediaMTX -> Viewers

Key Features:
- Software H.264 encoding (x264enc zerolatency, openh264enc fallback)
- Single persistent stream with dynamic topic switching
- Fixed RTSP URL (never changes when switching topics)
- HTTP API control for topic switching and status
- Multiple viewer support via MediaMTX
- Pipeline watchdog with automatic error recovery
- Auto-discovery of available ROS2 image topics

Runs on Jetson Edge Core host, controls the bridge inside Docker container.
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
DEFAULT_CONTAINER_NAME = "nomad_isaac_ros"
DEFAULT_RELAY_HTTP_PORT = 9200
DEFAULT_RTSP_URL = "rtsp://172.17.0.1:8554/primary"
DEFAULT_TOPIC = "/zed/zed_node/rgb/color/rect/image"

# Stream settings — tuned for Orin Nano (no hardware encoder)
DEFAULT_WIDTH = 848
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 15
DEFAULT_BITRATE = 1500  # kbps — sufficient for 480p15


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
    bitrate_kbps: int
    
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
        /zed/zed_node/rgb/color/rect/image -> zed: rgb/color/rect/image
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
    
    This class controls the simple video bridge running inside the Isaac ROS
    Docker container. The bridge:
    - Subscribes to ROS2 image topics from ZED camera
    - Encodes video using x264enc software encoder (zerolatency tuning)
    - Streams to MediaMTX RTSP server at fixed URL
    
    Topic switching is done via HTTP API to the bridge, which changes its
    ROS2 subscription. The RTSP URL stays constant - only the content changes.
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
        self._edge_core_api_key = (os.environ.get("NOMAD_API_KEY") or "").strip()
        self._edge_core_internal_token = (
            os.environ.get("NOMAD_INTERNAL_TOKEN") or ""
        ).strip()
        
        self._started = False
        self._lock = threading.RLock()

        # Watchdog state
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: Optional[threading.Thread] = None
        self._user_stopped = False  # set True when operator explicitly stops bridge

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

    def _get_relay_status_data(self, timeout_s: float = 2.0) -> Optional[Dict[str, Any]]:
        """Fetch bridge /status JSON, or None if unavailable."""
        try:
            url = f"http://localhost:{self.relay_http_port}/status"
            with urlopen(url, timeout=timeout_s) as response:
                return json.loads(response.read().decode())
        except Exception:
            return None

    def is_relay_running(self, require_recent_frames: bool = False) -> bool:
        """Check if the simple video bridge is running inside the container."""
        try:
            # Check if the HTTP API is responsive
            url = f"http://localhost:{self.relay_http_port}/health"
            with urlopen(url, timeout=2) as response:
                data = json.loads(response.read().decode())
                healthy = data.get("healthy", False)
                pipeline_playing = data.get("pipeline_playing", healthy)
                if not (healthy and pipeline_playing):
                    return False

                if not require_recent_frames:
                    return True

                status = self._get_relay_status_data(timeout_s=2.0)
                if not status:
                    return False

                age = status.get("last_frame_age_s")
                if age is None:
                    return False
                return age < 10.0
        except Exception:
            return False

    def start(self) -> bool:
        """
        Start the video streaming pipeline.
        
        Copies the simple video bridge script to the container and launches it.
        Returns True if successful.
        """
        result = self._start_internal()
        return result[0]
    
    def start_with_reason(self) -> tuple:
        """
        Start the video streaming pipeline with failure reason.
        
        Returns:
            Tuple of (success: bool, message: str)
        """
        return self._start_internal()
    
    def _start_internal(self) -> tuple:
        """Internal start implementation returning (success, message)."""
        with self._lock:
            # Check if a bridge is already running (from any launch path:
            # start_isaac_ros_auto.sh, auto_start thread, or prior API call).
            # Don't kill a working bridge just because _started is False
            # (e.g., after Edge Core restart).
            if self.is_relay_running(require_recent_frames=True):
                self._started = True
                logger.info("Simple video bridge already running, adopting existing instance")
                return (True, "Already running")
            elif self.is_relay_running(require_recent_frames=False):
                logger.warning(
                    "Simple video bridge process is alive but not receiving fresh frames; restarting bridge"
                )
            
            if not self.is_container_running():
                msg = f"Docker container '{self.container_name}' is not running. Start Isaac ROS first."
                logger.warning(msg)
                return (False, msg)
            
            # Copy simple bridge script to container
            script_name = "simple_video_bridge.py"
            script_path = os.path.join(os.path.dirname(__file__), "ros", script_name)
            if not os.path.exists(script_path):
                msg = f"Bridge script not found: {script_path}"
                logger.error(msg)
                return (False, msg)
            
            try:
                subprocess.run(
                    ["docker", "cp", script_path, f"{self.container_name}:/tmp/{script_name}"],
                    capture_output=True,
                    timeout=10,
                    check=True
                )
                logger.info(f"Copied {script_name} to container")
            except subprocess.CalledProcessError as e:
                msg = f"Failed to copy bridge script to container: {e.stderr or e}"
                logger.error(msg)
                return (False, msg)
            
            # Kill any existing bridge processes
            try:
                subprocess.run(
                    ["docker", "exec", self.container_name, "pkill", "-f", "simple_video_bridge"],
                    capture_output=True,
                    timeout=5
                )
            except Exception:
                pass  # OK if nothing to kill

            # Start the simple video bridge
            cmd = ["docker", "exec", "-d"]
            if self._edge_core_api_key:
                cmd.extend(["-e", f"NOMAD_API_KEY={self._edge_core_api_key}"])
            if self._edge_core_internal_token:
                cmd.extend(
                    ["-e", f"NOMAD_INTERNAL_TOKEN={self._edge_core_internal_token}"]
                )
            cmd.extend([
                self.container_name,
                "bash", "-c",
                f"source /opt/ros/humble/setup.bash 2>/dev/null; source /opt/ros/humble/install/setup.bash 2>/dev/null; "
                f"source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null; "
                f"python3 /tmp/{script_name} "
                f"--source-topic '{self.default_topic}' "
                f"--width {self.width} "
                f"--height {self.height} "
                f"--fps {self.fps} "
                f"--bitrate {self.bitrate} "
                f"--http-port {self.relay_http_port} "
                f"> /tmp/video_bridge.log 2>&1"
            ])
            
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                if result.returncode != 0:
                    msg = f"Docker exec failed: {result.stderr.strip() or 'unknown error'}"
                    logger.error(msg)
                    return (False, msg)
            except subprocess.TimeoutExpired:
                msg = "Bridge start command timed out after 10s"
                logger.error(msg)
                return (False, msg)
            except Exception as e:
                msg = f"Error starting bridge: {e}"
                logger.error(msg)
                return (False, msg)
            
            # Wait for bridge HTTP API and GStreamer pipeline to be ready.
            # We do NOT require fresh frames here — the ZED topic may not be
            # publishing yet (e.g. container just started). Frames will arrive
            # once ROS graph discovery completes. Checking pipeline_playing is
            # sufficient to confirm the bridge process is healthy and encoding.
            for i in range(15):  # Wait up to 15 seconds
                time.sleep(1)
                if self.is_relay_running(require_recent_frames=False):
                    self._started = True
                    logger.info(f"{script_name} started successfully")
                    return (True, "Started successfully")
            
            # Bridge didn't respond - check if the process is still running
            try:
                check = subprocess.run(
                    ["docker", "exec", self.container_name, "pgrep", "-f", "simple_video_bridge"],
                    capture_output=True, text=True, timeout=5
                )
                if check.returncode != 0:
                    # Process died - get the log
                    log_result = subprocess.run(
                        ["docker", "exec", self.container_name, "tail", "-20", "/tmp/video_bridge.log"],
                        capture_output=True, text=True, timeout=5
                    )
                    msg = f"Bridge process crashed. Log: {log_result.stdout.strip()[-200:]}"
                else:
                    msg = "Bridge process is running but no fresh frames were received within 25s"
            except Exception:
                msg = "Bridge did not start in time and could not check process status"
            
            logger.error(msg)
            return (False, msg)

    def stop(self) -> bool:
        """Stop the video streaming pipeline (operator-initiated; watchdog will not auto-restart)."""
        self._user_stopped = True
        with self._lock:
            try:
                # Stop simple video bridge
                subprocess.run(
                    ["docker", "exec", self.container_name, "pkill", "-f", "simple_video_bridge"],
                    capture_output=True,
                    timeout=5
                )
                self._started = False
                logger.info("Simple video bridge stopped")
                return True
            except Exception as e:
                logger.error(f"Error stopping bridge: {e}")
                return False

    def restart(self) -> bool:
        """Restart the video streaming pipeline."""
        self._user_stopped = False
        self.stop()
        time.sleep(2)
        return self.start()

    def _watchdog_loop(self):
        """
        Persistent watchdog: checks bridge health every 30 s.
        Restarts the bridge automatically if it has crashed or stopped
        receiving frames, unless the operator explicitly stopped it.
        Handles container restarts transparently.
        """
        logger.info("Video bridge watchdog started")
        while not self._watchdog_stop.is_set():
            self._watchdog_stop.wait(30)
            if self._watchdog_stop.is_set():
                break
            if self._user_stopped:
                continue
            try:
                if not self.is_container_running():
                    continue
                if not self.is_relay_running(require_recent_frames=False):
                    logger.warning("Watchdog: video bridge not running, restarting...")
                    ok, msg = self._start_internal()
                    if ok:
                        logger.info("Watchdog: video bridge restarted successfully")
                    else:
                        logger.error(f"Watchdog: restart failed — {msg}")
                elif not self.is_relay_running(require_recent_frames=True):
                    # Process alive but no frames — GStreamer pipeline stalled
                    logger.warning("Watchdog: bridge running but no recent frames, triggering pipeline restart")
                    try:
                        from urllib.request import urlopen, Request
                        req = Request(
                            f"http://localhost:{self.relay_http_port}/restart",
                            method="POST"
                        )
                        urlopen(req, timeout=5)
                    except Exception as e:
                        logger.warning(f"Watchdog: pipeline restart request failed ({e}), killing and relaunching bridge")
                        ok, msg = self._start_internal()
                        if not ok:
                            logger.error(f"Watchdog: relaunch failed — {msg}")
            except Exception as e:
                logger.error(f"Watchdog: unexpected error — {e}")

    def switch_topic(self, topic: str) -> bool:
        """
        Switch the video stream to a different ROS topic.
        
        This calls the bridge's HTTP API to change its source topic.
        The Isaac ROS H264 encoder restarts with the new topic.
        The ROS2 subscription changes instantly.
        
        Args:
            topic: Full ROS topic path (e.g., /zed/zed_node/left/image_rect_color)
            
        Returns:
            True if switch was successful
        """
        with self._lock:
            if not self.is_relay_running():
                logger.warning("Cannot switch topic: bridge not running")
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
                    logger.error(f"Failed to switch topic: {data.get('message', 'Unknown error')}")
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
        
        Queries the bridge's HTTP API which uses ros2 topic list
        to find sensor_msgs/Image topics.
        
        Returns:
            List of TopicInfo with name and display_name
        """
        if not self.is_relay_running():
            return []

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
            return []

    def _query_topics_docker(self) -> List[TopicInfo]:
        """Query topics directly via docker exec (fallback)."""
        try:
            cmd = [
                "docker", "exec", self.container_name,
                "bash", "-c",
                "source /opt/ros/humble/setup.bash 2>/dev/null; source /opt/ros/humble/install/setup.bash 2>/dev/null; ros2 topic list -t 2>/dev/null"
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
        
        Queries the bridge's HTTP API for detailed status.
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
            bitrate_kbps=self.bitrate,
        )
        
        if not self.is_relay_running():
            return default_status
        
        try:
            url = f"http://localhost:{self.relay_http_port}/status"
            with urlopen(url, timeout=5) as response:
                data = json.loads(response.read().decode())
            
            return StreamStatus(
                streaming=data.get("streaming", False),
                current_topic=data.get("source_topic", ""),  # Updated field name
                rtsp_url=self.rtsp_url.replace("172.17.0.1", "localhost"),
                fps=data.get("fps", 0.0),
                frame_count=data.get("frame_count", 0),
                error_count=data.get("error_count", 0),
                dropped_count=0,  # Not tracked in new bridge
                uptime_s=0.0,     # Not tracked in new bridge
                width=self.width,
                height=self.height,
                bitrate_kbps=self.bitrate,
            )
            
        except Exception as e:
            logger.error(f"Error getting status: {e}")
            return default_status

    def get_rtsp_url(self) -> str:
        """Get the constant RTSP URL for the video stream."""
        # Replace Docker host IP with accessible IP
        return self.rtsp_url.replace("172.17.0.1", "localhost")

    def get_logs(self, lines: int = 50) -> str:
        """Get recent logs from the simple video bridge process."""
        try:
            cmd = [
                "docker", "exec", self.container_name,
                "tail", f"-{lines}", "/tmp/video_bridge.log"
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            return result.stdout
        except Exception as e:
            return f"Error getting logs: {e}"

    def set_overlay(self, enabled: bool) -> bool:
        """
        Enable or disable the ROS2 detection overlay on the video stream.

        When enabled, the video bridge draws bounding boxes from Edge Core
        detections directly onto the video frames before encoding to RTSP.
        """
        if not self.is_relay_running():
            logger.warning("Cannot toggle overlay: video bridge not running")
            return False
        
        action = "enable" if enabled else "disable"
        try:
            url = f"http://localhost:{self.relay_http_port}/overlay/{action}"
            req = Request(url, method='POST')
            with urlopen(req, timeout=5) as response:
                data = json.loads(response.read().decode())
            return data.get("success", False)
        except Exception as e:
            logger.error(f"Error toggling overlay: {e}")
            return False

    def get_overlay_status(self) -> dict:
        """Get current overlay status from the video bridge."""
        if not self.is_relay_running():
            return {"enabled": False, "detection_count": 0}
        
        try:
            url = f"http://localhost:{self.relay_http_port}/overlay/status"
            with urlopen(url, timeout=5) as response:
                return json.loads(response.read().decode())
        except Exception as e:
            logger.error(f"Error getting overlay status: {e}")
            return {"enabled": False, "detection_count": 0}


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
        auto_start: Whether to auto-start the simple video bridge when container is ready
        **kwargs: Additional arguments passed to VideoStreamManager
        
    Returns:
        The initialized VideoStreamManager instance
    """
    global _video_stream_manager
    _video_stream_manager = VideoStreamManager(container_name=container_name, **kwargs)
    
    if auto_start:
        # Start in background thread to not block startup.
        # This is a SAFETY NET — the primary bridge launch is done by
        # start_isaac_ros_auto.sh after ZED topics are confirmed ready.
        # We wait long enough for that to happen first, then only start
        # a bridge if none is already running.
        def _delayed_start():
            # Wait for container to be running
            for i in range(45):
                if _video_stream_manager.is_container_running():
                    break
                time.sleep(2)
            else:
                logger.warning("Container not ready after 90s, video bridge not started")
                return

            # Wait for the primary bridge (started by start_isaac_ros_auto.sh)
            # to come online. ZED takes 15-30s to init, plus bridge has sleep 8.
            # Check periodically — if it's already running, we're done.
            logger.info("Waiting for video bridge (started by Isaac ROS startup)...")
            for i in range(30):  # Wait up to 60 seconds
                if _video_stream_manager.is_relay_running():
                    _video_stream_manager._started = True
                    logger.info("Video bridge already running (started by Isaac ROS startup)")
                    return
                time.sleep(2)

            # Safety net: primary bridge didn't start, launch one ourselves.
            # Retry a few times to survive transient startup races.
            logger.warning("Video bridge not detected after 60s, starting as safety net...")
            max_attempts = 3
            for attempt in range(1, max_attempts + 1):
                ok, msg = _video_stream_manager.start_with_reason()
                if ok:
                    logger.info(f"Video bridge safety net started on attempt {attempt}/{max_attempts}")
                    return

                logger.warning(
                    f"Video bridge safety net attempt {attempt}/{max_attempts} failed: {msg}"
                )
                if attempt < max_attempts:
                    time.sleep(10)

            logger.error("Video bridge safety net failed after all retry attempts")

        thread = threading.Thread(target=_delayed_start, daemon=True, name="video-delayed-start")
        thread.start()

    # Start persistent watchdog regardless of auto_start flag.
    # It begins polling 120 s after init so the delayed-start thread
    # has time to bring the bridge up before the watchdog first checks.
    def _start_watchdog_after_delay():
        time.sleep(120)
        _video_stream_manager._watchdog_thread = threading.Thread(
            target=_video_stream_manager._watchdog_loop,
            daemon=True,
            name="video-watchdog",
        )
        _video_stream_manager._watchdog_thread.start()

    threading.Thread(target=_start_watchdog_after_delay, daemon=True, name="video-watchdog-init").start()
    logger.info("Video bridge watchdog scheduled (starts in 120 s)")

    return _video_stream_manager
