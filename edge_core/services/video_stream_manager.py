# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
Video Stream Manager - Simple Video Bridge for NOMAD

Manages the simple video bridge running inside the Isaac ROS Docker container
for low-latency ROS-to-RTSP streaming using software H.264 encoding.

Architecture:
    ZED Camera (ROS2) -> x264enc (software, zerolatency) -> RTSP -> MediaMTX -> Viewers

The surface is what the ``/api/video/*`` routes consume: bridge start with a
failure reason, status, topic switching, and the detection-overlay toggle,
plus a crash-recovery watchdog whose thread lifecycle is owned by
``VideoStreamModule`` (see ``video_module.py``).

Runs on the Jetson Edge Core host, controls the bridge inside Docker.
"""

import json
import logging
import os
import subprocess
import threading
import time
from dataclasses import asdict, dataclass
from typing import Any
from urllib.error import URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

logger = logging.getLogger("edge_core.video_stream_manager")

_DOCKER_HOST_IP = os.environ.get("NOMAD_DOCKER_HOST_IP", "172.17.0.1")
DEFAULT_RTSP_URL = os.environ.get("NOMAD_RTSP_URL") or f"rtsp://{_DOCKER_HOST_IP}:8554/primary"
_NOMAD_ROS_ROOT = os.environ.get("NOMAD_ROS_ROOT", "/opt/ros/humble")
_NOMAD_ISAAC_WS = os.environ.get("NOMAD_ISAAC_WORKSPACE", "/workspaces/isaac_ros-dev")


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


class VideoStreamManager:
    """
    Controls the simple video bridge inside the Isaac ROS Docker container.

    The bridge subscribes to ROS2 image topics, encodes with x264enc
    (zerolatency), and streams to MediaMTX at a fixed RTSP URL. Topic
    switching goes through the bridge's HTTP API — the URL never changes,
    only the content.

    Lifecycle ownership: ``nomad-video-bridge.service`` (or the operator via
    POST /api/video/bridges/start) owns the bridge process. This class never
    auto-starts it; the watchdog only restarts a bridge that died unexpectedly.
    """

    def __init__(
        self,
        container_name: str = "nomad_isaac_ros",
        relay_http_port: int = 9200,
        rtsp_url: str = DEFAULT_RTSP_URL,
        default_topic: str = "/zed/zed_node/rgb/color/rect/image",
        width: int = 640,
        height: int = 360,
        fps: int = 15,
        bitrate: int = 800,
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
        self._edge_core_internal_token = (os.environ.get("NOMAD_INTERNAL_TOKEN") or "").strip()

        self._lock = threading.RLock()
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None

        logger.info("VideoStreamManager initialized (container=%s, rtsp=%s)", container_name, rtsp_url)

    def is_container_running(self) -> bool:
        """Check if the Isaac ROS container is running."""
        try:
            result = subprocess.run(
                ["docker", "ps", "--filter", f"name={self.container_name}", "--format", "{{.Names}}"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            return self.container_name in result.stdout
        except Exception as e:
            logger.error("Error checking container status: %s", e)
            return False

    def _get_relay_status_data(self, timeout_s: float = 2.0) -> dict[str, Any] | None:
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
            age = status.get("last_frame_age_s") if status else None
            return age is not None and age < 10.0
        except Exception:
            return False

    def start_with_reason(self) -> tuple[bool, str]:
        """Start the bridge inside the container; returns (success, message)."""
        with self._lock:
            # A bridge may already be running from any launch path (systemd
            # unit, watchdog re-launch, prior API call) — adopt it rather than
            # killing a working stream.
            if self.is_relay_running(require_recent_frames=True):
                logger.info("Simple video bridge already running, adopting existing instance")
                return (True, "Already running")
            elif self.is_relay_running(require_recent_frames=False):
                logger.warning("Simple video bridge process is alive but not receiving fresh frames; restarting bridge")

            if not self.is_container_running():
                msg = f"Docker container '{self.container_name}' is not running. Start Isaac ROS first."
                logger.warning(msg)
                return (False, msg)

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
                    check=True,
                )
            except subprocess.CalledProcessError as e:
                msg = f"Failed to copy bridge script to container: {e.stderr or e}"
                logger.error(msg)
                return (False, msg)

            # Kill any existing bridge processes (OK if nothing to kill).
            try:
                subprocess.run(
                    ["docker", "exec", self.container_name, "pkill", "-f", "simple_video_bridge"],
                    capture_output=True,
                    timeout=5,
                )
            except Exception:
                pass

            cmd = ["docker", "exec", "-d"]
            if self._edge_core_api_key:
                cmd.extend(["-e", f"NOMAD_API_KEY={self._edge_core_api_key}"])
            if self._edge_core_internal_token:
                cmd.extend(["-e", f"NOMAD_INTERNAL_TOKEN={self._edge_core_internal_token}"])
            for env_name in (
                "NOMAD_TASK2_DETECTOR_MODE",
                "NOMAD_DETECTOR_INTERVAL_S",
                "NOMAD_DETECTOR_MAX_WIDTH",
                "NOMAD_RAW_SNAPSHOT_INTERVAL",
                "NOMAD_TASK2_REQUIRE_DEPTH",
                "NOMAD_TASK2_MIN_DIAMETER_M",
                "NOMAD_TASK2_MAX_DIAMETER_M",
                "NOMAD_TASK2_DEBUG",
            ):
                env_value = os.environ.get(env_name)
                if env_value:
                    cmd.extend(["-e", f"{env_name}={env_value}"])
            cmd.extend(
                [
                    self.container_name,
                    "bash",
                    "-c",
                    f"source {_NOMAD_ROS_ROOT}/setup.bash 2>/dev/null; source {_NOMAD_ROS_ROOT}/install/setup.bash 2>/dev/null; "
                    f"source {_NOMAD_ISAAC_WS}/install/setup.bash 2>/dev/null; "
                    f"python3 /tmp/{script_name} "
                    f"--source-topic '{self.default_topic}' "
                    f"--width {self.width} "
                    f"--height {self.height} "
                    f"--fps {self.fps} "
                    f"--bitrate {self.bitrate} "
                    f"--http-port {self.relay_http_port} "
                    f"> /tmp/video_bridge.log 2>&1",
                ]
            )

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

            # Wait for the bridge HTTP API and GStreamer pipeline. Fresh frames
            # are NOT required here — the ZED topic may not be publishing yet;
            # pipeline_playing confirms the bridge is healthy and encoding.
            for _ in range(15):
                time.sleep(1)
                if self.is_relay_running(require_recent_frames=False):
                    logger.info("%s started successfully", script_name)
                    return (True, "Started successfully")

            # Bridge didn't respond — check if the process is still running.
            try:
                check = subprocess.run(
                    ["docker", "exec", self.container_name, "pgrep", "-f", "simple_video_bridge"],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                if check.returncode != 0:
                    log_result = subprocess.run(
                        ["docker", "exec", self.container_name, "tail", "-20", "/tmp/video_bridge.log"],
                        capture_output=True,
                        text=True,
                        timeout=5,
                    )
                    msg = f"Bridge process crashed. Log: {log_result.stdout.strip()[-200:]}"
                else:
                    msg = "Bridge process is running but no fresh frames were received within 25s"
            except Exception:
                msg = "Bridge did not start in time and could not check process status"

            logger.error(msg)
            return (False, msg)

    # -- watchdog (thread lifecycle owned by VideoStreamModule) ---------------

    def start_watchdog(self, initial_delay_s: float = 120.0) -> None:
        """Start the crash-recovery watchdog thread (idempotent)."""
        if self._watchdog_thread and self._watchdog_thread.is_alive():
            return
        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            args=(initial_delay_s,),
            daemon=True,
            name="video-watchdog",
        )
        self._watchdog_thread.start()

    def stop_watchdog(self) -> None:
        self._watchdog_stop.set()
        if self._watchdog_thread and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=2.0)

    def _watchdog_loop(self, initial_delay_s: float) -> None:
        """Check bridge health every 30 s; restart it if it crashed or stalled.

        The initial delay gives the systemd-owned bridge time to come up on
        boot before the watchdog starts second-guessing it.
        """
        logger.info("Video bridge watchdog started (first check in %.0fs)", initial_delay_s)
        self._watchdog_stop.wait(initial_delay_s)
        while not self._watchdog_stop.is_set():
            try:
                if not self.is_container_running():
                    pass
                elif not self.is_relay_running(require_recent_frames=False):
                    logger.warning("Watchdog: video bridge not running, restarting...")
                    ok, msg = self.start_with_reason()
                    if not ok:
                        logger.error("Watchdog: restart failed - %s", msg)
                elif not self.is_relay_running(require_recent_frames=True):
                    # Process alive but no frames — GStreamer pipeline stalled.
                    logger.warning("Watchdog: bridge running but no recent frames, triggering pipeline restart")
                    try:
                        req = Request(f"http://localhost:{self.relay_http_port}/restart", method="POST")
                        urlopen(req, timeout=5)
                    except Exception as e:
                        logger.warning("Watchdog: pipeline restart request failed (%s), relaunching bridge", e)
                        ok, msg = self.start_with_reason()
                        if not ok:
                            logger.error("Watchdog: relaunch failed - %s", msg)
            except Exception as e:
                logger.error("Watchdog: unexpected error - %s", e)
            self._watchdog_stop.wait(30)

    # -- bridge HTTP API ------------------------------------------------------

    def switch_topic(self, topic: str) -> bool:
        """Switch the video stream to a different ROS topic via the bridge API."""
        with self._lock:
            if not self.is_relay_running():
                logger.warning("Cannot switch topic: bridge not running")
                return False
            try:
                url = f"http://localhost:{self.relay_http_port}/switch?topic={quote(topic, safe='')}"
                with urlopen(Request(url, method="POST"), timeout=5) as response:
                    data = json.loads(response.read().decode())
                if data.get("success"):
                    logger.info("Switched video source to: %s", topic)
                    return True
                logger.error("Failed to switch topic: %s", data.get("message", "Unknown error"))
                return False
            except URLError as e:
                logger.error("HTTP error switching topic: %s", e)
                return False
            except Exception as e:
                logger.error("Error switching topic: %s", e)
                return False

    def set_overlay(self, enabled: bool) -> bool:
        """Enable/disable the ROS2 detection overlay drawn onto the RTSP feed."""
        if not self.is_relay_running():
            logger.warning("Cannot toggle overlay: video bridge not running")
            return False
        action = "enable" if enabled else "disable"
        try:
            url = f"http://localhost:{self.relay_http_port}/overlay/{action}"
            with urlopen(Request(url, method="POST"), timeout=5) as response:
                data = json.loads(response.read().decode())
            return data.get("success", False)
        except Exception as e:
            logger.error("Error toggling overlay: %s", e)
            return False

    def get_status(self) -> StreamStatus:
        """Get current stream status from the bridge's HTTP API."""
        default_status = StreamStatus(
            streaming=False,
            current_topic="",
            rtsp_url=self._local_rtsp_url(),
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
        data = self._get_relay_status_data(timeout_s=5.0)
        if data is None:
            return default_status
        return StreamStatus(
            streaming=data.get("streaming", False),
            current_topic=data.get("source_topic", ""),
            rtsp_url=self._local_rtsp_url(),
            fps=data.get("fps", 0.0),
            frame_count=data.get("frame_count", 0),
            error_count=data.get("error_count", 0),
            dropped_count=data.get("dropped_count", 0),
            uptime_s=0.0,  # not tracked by the bridge
            width=self.width,
            height=self.height,
            bitrate_kbps=self.bitrate,
        )

    def _local_rtsp_url(self) -> str:
        return self.rtsp_url.replace(_DOCKER_HOST_IP, "localhost")
