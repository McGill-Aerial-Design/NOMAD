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
import queue
import threading
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

        # Background HTTP sender. The ROS timer drops payloads onto this queue
        # instead of calling urlopen() inline -- a stalled Edge Core API would
        # otherwise block the timer for up to 1s and crater the 5 Hz obstacle
        # avoidance stream. Matches the _mesh_sender_loop pattern in
        # ros_http_bridge.py.
        self._send_queue: "queue.Queue[list[int]]" = queue.Queue(maxsize=4)
        self._sender_stop = threading.Event()
        self._sender_thread = threading.Thread(
            target=self._sender_loop, name="obstacle-distance-sender", daemon=True
        )
        self._sender_thread.start()

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
        data = np.asarray(grid.data, dtype=np.int8).reshape(height, width)

        # Drone position is at center of the grid (nvblox centers slice on robot)
        center_gx = width / 2.0
        center_gy = height / 2.0

        # Maximum raycast distance in grid cells; half-cell step for accuracy
        max_range_cells = int(MAX_DISTANCE_CM / 100.0 / resolution)
        step_size = 0.5
        num_steps = max(1, int(max_range_cells / step_size) - 1)

        # Step distances along each ray (in grid cells). Shape: (num_steps,)
        t = (np.arange(1, num_steps + 1, dtype=np.float32)) * step_size

        # Per-sector direction vectors. Shape: (NUM_SECTORS, 1)
        sector_indices = np.arange(NUM_SECTORS, dtype=np.float32)
        angles_rad = np.radians(sector_indices * SECTOR_WIDTH_DEG) + self._drone_yaw_rad
        dx = np.cos(angles_rad)[:, None]
        dy = np.sin(angles_rad)[:, None]

        # Grid coordinates for every (sector, step). Shape: (NUM_SECTORS, num_steps)
        gx = (center_gx + dx * t[None, :]).astype(np.int32)
        gy = (center_gy + dy * t[None, :]).astype(np.int32)

        in_bounds = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
        gx_safe = np.clip(gx, 0, width - 1)
        gy_safe = np.clip(gy, 0, height - 1)

        cells = data[gy_safe, gx_safe]
        occupied = (cells >= 50) & in_bounds

        # First stop along each ray: either an occupied cell, or leaving bounds.
        stop = occupied | (~in_bounds)
        any_stop = stop.any(axis=1)
        first_idx = np.argmax(stop, axis=1)

        rows = np.arange(NUM_SECTORS)
        hit_occupied = any_stop & occupied[rows, first_idx]
        dist_m = np.where(
            hit_occupied,
            t[first_idx] * resolution,
            MAX_DISTANCE_CM / 100.0,
        )

        distances_cm = np.clip(
            (dist_m * 100.0).astype(np.int32), MIN_DISTANCE_CM, MAX_DISTANCE_CM
        )
        return distances_cm.tolist()

    def _send_obstacle_distance(self, distances: list[int]) -> None:
        """Hand off distances to the background sender (non-blocking)."""
        try:
            self._send_queue.put_nowait(distances)
        except queue.Full:
            # Drop the new sample rather than block the ROS timer; the next
            # tick (5 Hz) will deliver a fresh sweep.
            try:
                _ = self._send_queue.get_nowait()
                self._send_queue.put_nowait(distances)
            except (queue.Empty, queue.Full):
                pass

    def _sender_loop(self) -> None:
        """Background thread: posts queued sweeps to the Edge Core API."""
        while not self._sender_stop.is_set():
            try:
                distances = self._send_queue.get(timeout=0.5)
            except queue.Empty:
                continue
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
                try:
                    self.get_logger().debug(f"Send error: {e}")
                except Exception:
                    pass

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
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
