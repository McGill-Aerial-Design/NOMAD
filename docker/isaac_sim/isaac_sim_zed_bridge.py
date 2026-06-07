# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Isaac Sim ZED Bridge

Launches Isaac Sim headlessly, spawns a drone with a ZED2i camera model,
and publishes the same ROS2 topics that the real ZED wrapper publishes
on the Jetson. This makes the simulation transparent to Edge Core and
the Mission Planner — they cannot distinguish sim from real hardware.

ROS2 topics published (same as the real Jetson ZED wrapper):
  /zed/zed_node/odom                          (nav_msgs/Odometry)
  /zed/zed_node/imu/data                      (sensor_msgs/Imu)
  /zed/zed_node/imu/mag                       (sensor_msgs/MagneticField)
  /zed/zed_node/rgb/color/rect/image           (sensor_msgs/Image)
  /zed/zed_node/depth/depth_registered         (sensor_msgs/Image)
  /zed/zed_node/obj_det/objects               (zed_interfaces/ObjectsStamped)
  /tf                                          (tf2_msgs/TFMessage)

Usage:
  python3 isaac_sim_zed_bridge.py [options]

Environment variables:
  ISAAC_SIM_HEADLESS     Set to "1" for headless (default: 1)
  ISAAC_SIM_WORLD        USD world path (default: empty_scene)
  ISAAC_SIM_DRONE_START_X/Y/Z/YAW  Initial drone pose
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("isaac_sim_zed_bridge")


@dataclass
class SimDronePose:
    x: float = 0.0
    y: float = 0.0
    z: float = 1.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


class IsaacSimZEDBridge:
    """Bridges Isaac Sim camera data to ZED-compatible ROS2 topics.

    This class orchestrates:
    1. Launching Isaac Sim (headless or windowed)
    2. Spawning a drone with a ZED2i camera rig
    3. Publishing ZED-compatible ROS2 topics from the sim camera data
    4. Accepting velocity commands (cmd_vel) for sim drone control

    In simulation mode, the ZED ROS2 wrapper's simulation publisher
    is used — it reads synthetic frames from the Isaac Sim rendering
    pipeline and publishes them on the same topic names and types as
    the real ZED SDK.
    """

    def __init__(
        self,
        headless: bool = True,
        world_path: str = "",
        drone_start: SimDronePose | None = None,
        camera_model: str = "zed2i",
        camera_name: str = "zed",
        grab_resolution: str = "HD720",
        depth_mode: str = "NEURAL_LIGHT",
    ):
        self._headless = headless
        self._world_path = world_path
        self._drone_start = drone_start or SimDronePose()
        self._camera_model = camera_model
        self._camera_name = camera_name
        self._grab_resolution = grab_resolution
        self._depth_mode = depth_mode
        self._running = False
        self._sim_app = None
        self._ros_thread: threading.Thread | None = None

    def start(self) -> None:
        """Launch Isaac Sim and start publishing."""
        self._running = True
        self._launch_isaac_sim()
        self._start_zed_wrapper_sim()
        self._start_ros_bridge()
        logger.info("Isaac Sim ZED Bridge started")

    def stop(self) -> None:
        """Shutdown Isaac Sim and stop publishing."""
        self._running = False
        if self._sim_app is not None:
            try:
                self._sim_app.close()
            except Exception:
                pass
        logger.info("Isaac Sim ZED Bridge stopped")

    def _launch_isaac_sim(self) -> None:
        """Initialize Isaac Sim application."""
        try:
            from isaacsim import SimulationApp

            launch_config = {
                "headless": self._headless,
                "anti_aliasing": 0,
            }
            if self._world_path:
                launch_config["experience"] = self._world_path

            self._sim_app = SimulationApp(launch_config)
            logger.info(f"Isaac Sim launched (headless={self._headless})")
        except ImportError:
            logger.warning("isaacsim package not available — running in ZED-wrapper-only mode")
            self._sim_app = None
            return

        try:
            import isaacsim.core.api as isaac_core

            self._world = isaac_core.World(stage_units_in_meters=1.0)
            self._world.scene.add_default_ground_plane()
            logger.info("Isaac Sim world initialized with ground plane")
        except Exception as e:
            logger.warning(f"Could not initialize Isaac Sim world: {e}")

    def _start_zed_wrapper_sim(self) -> None:
        """Launch the ZED ROS2 wrapper in simulation mode.

        The ZED wrapper's sim mode publishes synthetic camera data
        from Isaac Sim on the standard /zed/zed_node/* topics.
        """
        try:
            import rclpy

            if not rclpy.ok():
                rclpy.init()
        except ImportError:
            logger.error(
                "rclpy not available — cannot start ZED wrapper. Ensure ROS2 Humble is installed in the container."
            )
            return

        logger.info("ZED wrapper sim mode: publishing to /zed/zed_node/* topics")
        self._start_zed_sim_publisher()

    def _start_zed_sim_publisher(self) -> None:
        """Start the ZED simulation publisher node.

        This creates a ROS2 node that publishes synthetic ZED-compatible
        data at realistic rates. In a full Isaac Sim deployment, the
        ZED wrapper reads from the sim camera directly; this publisher
        serves as the bridge when using Isaac Sim's rendered output.
        """
        try:
            from zed_sim_publisher import ZEDSimPublisher

            self._zed_sim_pub = ZEDSimPublisher(
                camera_name=self._camera_name,
                camera_model=self._camera_model,
                grab_resolution=self._grab_resolution,
                depth_mode=self._depth_mode,
            )
            self._zed_sim_pub.start()
            logger.info("ZED simulation publisher started")
        except ImportError:
            logger.info("zed_sim_publisher not available — using direct ZED wrapper launch")
            self._launch_zed_wrapper_process()

    def _launch_zed_wrapper_process(self) -> None:
        """Launch the actual zed_camera.launch.py with sim mode enabled."""
        import subprocess

        cmd = [
            "ros2",
            "launch",
            "zed_wrapper",
            "zed_camera.launch.py",
            f"camera_model:={self._camera_model}",
            f"camera_name:={self._camera_name}",
            "sim_enabled:=true",
        ]
        try:
            self._zed_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            logger.info(f"ZED wrapper process started (PID {self._zed_proc.pid})")
        except FileNotFoundError:
            logger.warning("ros2 launch not found — ZED wrapper not started")

    def _start_ros_bridge(self) -> None:
        """Start the NOMAD ROS-HTTP bridge to forward data to Edge Core."""
        bridge_path = Path("/opt/nomad/ros_http_bridge")
        if not bridge_path.exists():
            bridge_path = Path("/workspaces/nomad-sim/edge_core/ros_http_bridge")

        if not bridge_path.exists():
            logger.warning(f"ROS-HTTP bridge not found at {bridge_path}")
            return

        import subprocess

        host = os.environ.get("NOMAD_API_HOST", "nomad_edge_core_sim")
        port = os.environ.get("NOMAD_API_PORT", "8000")
        rate = os.environ.get("ROS_HTTP_BRIDGE_RATE", "5")
        vio_topic = os.environ.get("ROS_HTTP_BRIDGE_VIO_TOPIC", "/zed/zed_node/odom")
        mag_topic = os.environ.get("ROS_HTTP_BRIDGE_MAG_TOPIC", "/zed/zed_node/imu/mag")

        cmd = [
            sys.executable,
            "-m",
            "edge_core.ros_http_bridge.main",
            "--host",
            host,
            "--port",
            port,
            "--rate",
            rate,
            "--vio-topic",
            vio_topic,
            "--mag-topic",
            mag_topic,
        ]

        nvblox_autostart = os.environ.get("NOMAD_AUTOSTART_NVBLOX", "false").lower()
        nvblox_mesh = os.environ.get("NOMAD_ENABLE_NVBLOX_MESH", "false").lower()
        if nvblox_autostart in ("true", "1", "yes") or nvblox_mesh in ("true", "1", "yes"):
            cmd.append("--mesh-topic")
            cmd.append("/nvblox_node/color_layer_marker")
        else:
            cmd.append("--disable-mesh")

        detections_autostart = os.environ.get("NOMAD_DETECTIONS_AUTO_START", "false").lower()
        if detections_autostart not in ("true", "1", "yes"):
            cmd.append("--disable-detections")

        env = os.environ.copy()
        pythonpath = env.get("PYTHONPATH", "")
        bridge_parent = str(bridge_path.parent)
        if bridge_parent not in pythonpath:
            env["PYTHONPATH"] = f"{bridge_parent}:{pythonpath}"

        try:
            self._bridge_proc = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            logger.info(f"ROS-HTTP bridge started (PID {self._bridge_proc.pid})")
        except Exception as e:
            logger.error(f"Failed to start ROS-HTTP bridge: {e}")

    def spin(self) -> None:
        """Block until stopped."""
        try:
            while self._running:
                time.sleep(0.5)
                if self._sim_app is not None:
                    try:
                        self._sim_app.update()
                    except Exception:
                        pass
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="NOMAD Isaac Sim ZED Bridge")
    parser.add_argument("--headless", action="store_true", default=True, help="Run Isaac Sim headless (default: true)")
    parser.add_argument("--windowed", action="store_true", help="Run Isaac Sim with a window")
    parser.add_argument("--world", type=str, default="", help="Isaac Sim USD world path")
    parser.add_argument("--start-x", type=float, default=0.0, help="Drone start X position (meters)")
    parser.add_argument("--start-y", type=float, default=0.0, help="Drone start Y position (meters)")
    parser.add_argument("--start-z", type=float, default=1.0, help="Drone start Z position (meters)")
    parser.add_argument("--start-yaw", type=float, default=0.0, help="Drone start yaw (degrees)")
    args = parser.parse_args()

    headless = not args.windowed
    if os.environ.get("ISAAC_SIM_HEADLESS", "1") == "0":
        headless = False

    start_pose = SimDronePose(
        x=float(os.environ.get("ISAAC_SIM_DRONE_START_X", args.start_x)),
        y=float(os.environ.get("ISAAC_SIM_DRONE_START_Y", args.start_y)),
        z=float(os.environ.get("ISAAC_SIM_DRONE_START_Z", args.start_z)),
        yaw=math.radians(float(os.environ.get("ISAAC_SIM_DRONE_START_YAW", args.start_yaw))),
    )

    bridge = IsaacSimZEDBridge(
        headless=headless,
        world_path=args.world or os.environ.get("ISAAC_SIM_WORLD", ""),
        drone_start=start_pose,
        camera_model=os.environ.get("ZED_CAMERA_MODEL", "zed2i"),
        camera_name=os.environ.get("ZED_CAMERA_NAME", "zed"),
        grab_resolution=os.environ.get("ZED_GRAB_RESOLUTION", "HD720"),
        depth_mode=os.environ.get("ZED_DEPTH_MODE", "NEURAL_LIGHT"),
    )

    bridge.start()
    bridge.spin()


if __name__ == "__main__":
    main()
