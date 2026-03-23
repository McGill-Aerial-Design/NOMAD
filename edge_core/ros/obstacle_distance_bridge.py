#!/usr/bin/env python3
"""
OBSTACLE_DISTANCE Bridge for NOMAD (NV-008).

Converts nvblox 2D ESDF distance slice into MAVLink OBSTACLE_DISTANCE messages
for ArduPilot's proximity-based obstacle avoidance.

Subscribes to:
- /nvblox_node/combined_dynamic_map_slice (nav_msgs/OccupancyGrid) - combined static+dynamic
  occupancy slice at drone altitude (includes fast-decaying dynamic obstacles like people)

Sends to:
- Edge Core API POST /api/obstacle_distance - forwarded to ArduPilot via pymavlink

The OBSTACLE_DISTANCE message contains distances in 72 angular sectors (5-degree
increments, 0-360 degrees) around the drone. ArduPilot uses these distances to
avoid obstacles in the horizontal plane.

Architecture:
    nvblox (ESDF slice) -> this bridge -> Edge Core API -> pymavlink -> ArduPilot

Usage (inside Isaac ROS container):
    python3 obstacle_distance_bridge.py --host 172.17.0.1 --port 8000
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import time
from typing import Optional
from urllib.request import Request, urlopen
from urllib.error import URLError

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("obstacle_distance_bridge")


# MAVLink OBSTACLE_DISTANCE constants
NUM_SECTORS = 72           # 360 / 5 = 72 sectors
SECTOR_WIDTH_DEG = 5.0     # degrees per sector
MAX_DISTANCE_CM = 2000     # 20m max distance (in cm, MAVLink uses cm)
MIN_DISTANCE_CM = 20       # 20cm min distance


class ObstacleDistanceBridge(Node):
    """
    Converts nvblox 2D occupancy grid to MAVLink OBSTACLE_DISTANCE.

    The occupancy grid from nvblox represents occupied/free space at the
    drone's altitude. This node raycasts from the drone's position outward
    in 72 angular sectors to find the nearest obstacle in each direction.
    """

    def __init__(
        self,
        host: str = "172.17.0.1",
        port: int = 8000,
        send_rate_hz: float = 5.0,
        obstacle_buffer_m: float = 2.0,
        map_slice_topic: str = "/nvblox_node/combined_dynamic_map_slice",
    ):
        super().__init__("nomad_obstacle_distance_bridge")

        self._host = host
        self._port = port
        self._base_url = f"http://{host}:{port}"
        self._send_rate_hz = send_rate_hz
        self._obstacle_buffer_m = obstacle_buffer_m

        # Latest occupancy grid
        self._latest_grid: Optional[OccupancyGrid] = None
        self._grid_recv_count = 0
        self._send_count = 0

        # Latest obstacle distances (72 sectors, in cm)
        self._distances = [MAX_DISTANCE_CM] * NUM_SECTORS

        # Drone yaw from Edge Core (radians, NED convention: 0=North, CW positive)
        self._drone_yaw_rad = 0.0

        # Sector exclusion for spray approach (SP-005)
        # Set of sector indices to exclude (report as max distance)
        self._excluded_sectors: set[int] = set()

        # QoS for nvblox map slice (best effort, volatile)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscribe to occupancy grid
        self.create_subscription(
            OccupancyGrid,
            map_slice_topic,
            self._handle_map_slice,
            qos,
        )
        self.get_logger().info(f"Subscribed to map slice: {map_slice_topic}")

        # Timer to process and send obstacle distances
        send_period = 1.0 / send_rate_hz
        self.create_timer(send_period, self._process_and_send)

        # Timer to poll drone yaw from Edge Core (2 Hz)
        self.create_timer(0.5, self._poll_drone_yaw)

        self.get_logger().info(
            f"Obstacle distance bridge started: {send_rate_hz} Hz -> {self._base_url}"
            f" (buffer={obstacle_buffer_m}m)"
        )

    def _handle_map_slice(self, msg: OccupancyGrid) -> None:
        """Store latest occupancy grid from nvblox."""
        self._latest_grid = msg
        self._grid_recv_count += 1

    def _poll_drone_yaw(self) -> None:
        """Poll drone heading from Edge Core VIO status."""
        try:
            req = Request(f"{self._base_url}/api/vio/pose", method="GET")
            req.add_header("Connection", "keep-alive")
            with urlopen(req, timeout=0.5) as resp:
                data = json.loads(resp.read().decode("utf-8"))
            yaw = data.get("yaw", 0.0)
            if yaw is not None:
                self._drone_yaw_rad = float(yaw)
        except (URLError, Exception):
            pass

    def set_excluded_sectors(self, sectors: set[int]) -> None:
        """Set sectors to exclude from obstacle avoidance (SP-005)."""
        self._excluded_sectors = sectors

    def set_obstacle_buffer(self, buffer_m: float) -> None:
        """Update obstacle buffer distance (mode-dependent)."""
        self._obstacle_buffer_m = buffer_m
        self.get_logger().info(f"Obstacle buffer updated: {buffer_m}m")

    def _process_and_send(self) -> None:
        """Process latest grid into obstacle distances and send to Edge Core."""
        grid = self._latest_grid
        if grid is None:
            return

        # Raycast from center of grid in 72 directions
        distances = self._raycast_grid(grid)

        # Apply sector exclusion (SP-005: target sector reports max distance)
        for idx in self._excluded_sectors:
            if 0 <= idx < NUM_SECTORS:
                distances[idx] = MAX_DISTANCE_CM

        self._distances = distances

        # Send to Edge Core
        self._send_obstacle_distance(distances)

    def _raycast_grid(self, grid: OccupancyGrid) -> list[int]:
        """
        Raycast from grid center in 72 angular sectors to find nearest obstacle.

        The occupancy grid from nvblox uses:
        - 0 = free space
        - 100 = occupied
        - -1 = unknown

        We treat occupied (>= 50) as obstacle, everything else as free.

        Returns:
            List of 72 distances in centimeters.
        """
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution  # meters per cell
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        # Grid data as numpy array
        data = np.array(grid.data, dtype=np.int8).reshape(height, width)

        # Drone position is at center of the grid (nvblox centers slice on robot)
        # Convert to grid coordinates
        center_world_x = origin_x + (width * resolution) / 2.0
        center_world_y = origin_y + (height * resolution) / 2.0
        center_gx = width / 2.0
        center_gy = height / 2.0

        # Maximum raycast distance in grid cells
        max_range_cells = int(MAX_DISTANCE_CM / 100.0 / resolution)
        # Step size for raycasting (half a cell for accuracy)
        step_size = 0.5

        distances = [MAX_DISTANCE_CM] * NUM_SECTORS

        for sector in range(NUM_SECTORS):
            # Angle for this sector (degrees -> radians)
            # OBSTACLE_DISTANCE convention: 0 = forward (body frame), CW positive
            # We need to rotate by drone yaw to get world-frame angle
            sector_angle_body_deg = sector * SECTOR_WIDTH_DEG
            sector_angle_world_rad = (
                math.radians(sector_angle_body_deg) + self._drone_yaw_rad
            )

            # Direction in grid coordinates
            # Grid: X = columns (east), Y = rows (north in ROS convention)
            dx = math.cos(sector_angle_world_rad)  # east component
            dy = math.sin(sector_angle_world_rad)  # north component (grid Y)

            # Raycast along this direction
            dist_m = MAX_DISTANCE_CM / 100.0
            for step in range(1, int(max_range_cells / step_size)):
                t = step * step_size
                gx = int(center_gx + dx * t)
                gy = int(center_gy + dy * t)

                # Check bounds
                if gx < 0 or gx >= width or gy < 0 or gy >= height:
                    break

                # Check occupancy
                cell_value = data[gy, gx]
                if cell_value >= 50:  # occupied
                    dist_m = t * resolution
                    break

            distances[sector] = max(
                MIN_DISTANCE_CM,
                min(MAX_DISTANCE_CM, int(dist_m * 100))
            )

        return distances

    def _send_obstacle_distance(self, distances: list[int]) -> None:
        """Send obstacle distances to Edge Core API."""
        try:
            payload = json.dumps({
                "distances": distances,
                "increment": int(SECTOR_WIDTH_DEG),
                "min_distance": MIN_DISTANCE_CM,
                "max_distance": MAX_DISTANCE_CM,
                "angle_offset": 0,
                "frame": 0,  # MAV_FRAME_BODY_FRD
            }).encode("utf-8")

            req = Request(
                f"{self._base_url}/api/obstacle_distance",
                data=payload,
                method="POST",
            )
            req.add_header("Content-Type", "application/json")
            req.add_header("Connection", "keep-alive")

            with urlopen(req, timeout=1.0) as resp:
                if resp.status == 200:
                    self._send_count += 1

        except URLError:
            pass
        except Exception as e:
            self.get_logger().debug(f"Send error: {e}")

    def get_stats(self) -> dict:
        """Get bridge statistics."""
        return {
            "grid_received": self._grid_recv_count,
            "messages_sent": self._send_count,
            "obstacle_buffer_m": self._obstacle_buffer_m,
            "excluded_sectors": len(self._excluded_sectors),
        }


def main():
    parser = argparse.ArgumentParser(description="NOMAD Obstacle Distance Bridge")
    parser.add_argument("--host", default="172.17.0.1", help="Edge Core host")
    parser.add_argument("--port", type=int, default=8000, help="Edge Core port")
    parser.add_argument("--rate", type=float, default=5.0,
                        help="Send rate in Hz (default: 5)")
    parser.add_argument("--buffer", type=float, default=2.0,
                        help="Obstacle buffer distance in meters (default: 2.0)")
    parser.add_argument("--topic", default="/nvblox_node/combined_dynamic_map_slice",
                        help="Occupancy grid topic (combined static+dynamic)")
    args = parser.parse_args()

    rclpy.init()

    node = ObstacleDistanceBridge(
        host=args.host,
        port=args.port,
        send_rate_hz=args.rate,
        obstacle_buffer_m=args.buffer,
        map_slice_topic=args.topic,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
