#!/usr/bin/env python3
"""
ROS2-HTTP Bridge for NOMAD.

This script runs INSIDE the Isaac ROS Docker container and bridges
ROS2 topics to the NOMAD Edge Core HTTP API running on the host.

It subscribes to:
- /visual_slam/tracking/odometry (VIO pose from Isaac ROS VSLAM)
- /nvblox_node/mesh (3D mesh from Nvblox) - optional
- /nvblox_node/map_slice (2D occupancy slice) - for visualization

And sends data to NOMAD Edge Core via HTTP POST requests.

Usage (inside Isaac ROS container):
    python3 ros_http_bridge.py --host 172.17.0.1 --port 8000
    
Note: 172.17.0.1 is the default Docker host IP from inside a container.
      For Jetson with network_mode=host, use localhost.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import threading
import time
from dataclasses import dataclass, asdict
from typing import Optional
from urllib.request import Request, urlopen
from urllib.error import URLError

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("ros_http_bridge")


@dataclass
class VIOData:
    """VIO pose data to send to edge_core."""
    timestamp: float
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    confidence: float = 1.0
    source: str = "isaac_ros"


class ROSHTTPBridge(Node):
    """
    ROS2 node that bridges topics to NOMAD Edge Core HTTP API.
    """
    
    def __init__(
        self,
        host: str = "172.17.0.1",
        port: int = 8000,
        vio_topic: str = "/visual_slam/tracking/odometry",
        send_rate_hz: float = 30.0,
    ):
        super().__init__("nomad_ros_http_bridge")
        
        self._host = host
        self._port = port
        self._base_url = f"http://{host}:{port}"
        self._send_interval = 1.0 / send_rate_hz
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # Latest data
        self._latest_vio: Optional[VIOData] = None
        self._lock = threading.Lock()
        
        # Stats
        self._vio_recv_count = 0
        self._vio_send_count = 0
        self._send_errors = 0
        self._last_send_time = 0.0
        
        # Subscribe to VIO odometry
        self.create_subscription(
            Odometry,
            vio_topic,
            self._handle_vio,
            sensor_qos,
        )
        self.get_logger().info(f"Subscribed to VIO: {vio_topic}")
        
        # Timer to send data to edge_core
        self.create_timer(self._send_interval, self._send_to_edge_core)
        
        self.get_logger().info(f"ROS-HTTP Bridge started -> {self._base_url}")
    
    def _handle_vio(self, msg: Odometry) -> None:
        """Handle VIO odometry from Isaac ROS VSLAM."""
        try:
            pose = msg.pose.pose
            twist = msg.twist.twist
            
            # Quaternion to Euler
            roll, pitch, yaw = self._quat_to_euler(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            
            # Convert from camera frame to NED frame
            # ZED: X-right, Y-down, Z-forward
            # NED: X-north, Y-east, Z-down
            vio = VIOData(
                timestamp=time.time(),
                x=pose.position.z,   # Forward -> North
                y=pose.position.x,   # Right -> East
                z=-pose.position.y,  # Up -> Down (negate)
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                vx=twist.linear.z,
                vy=twist.linear.x,
                vz=-twist.linear.y,
                confidence=1.0,
                source="isaac_ros",
            )
            
            with self._lock:
                self._latest_vio = vio
                self._vio_recv_count += 1
                
        except Exception as e:
            self.get_logger().error(f"VIO processing error: {e}")
    
    def _send_to_edge_core(self) -> None:
        """Send latest VIO data to edge_core via HTTP."""
        with self._lock:
            vio = self._latest_vio
        
        if vio is None:
            return
        
        # Rate limit
        now = time.time()
        if now - self._last_send_time < self._send_interval:
            return
        self._last_send_time = now
        
        try:
            url = f"{self._base_url}/api/vio/update"
            data = json.dumps(asdict(vio)).encode("utf-8")
            
            request = Request(
                url,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            
            with urlopen(request, timeout=0.5) as response:
                if response.status == 200:
                    self._vio_send_count += 1
                else:
                    self._send_errors += 1
                    
        except URLError as e:
            self._send_errors += 1
            if self._send_errors % 100 == 1:
                self.get_logger().warning(f"Failed to send VIO: {e}")
        except Exception as e:
            self._send_errors += 1
            self.get_logger().error(f"Send error: {e}")
    
    def _quat_to_euler(
        self, x: float, y: float, z: float, w: float
    ) -> tuple[float, float, float]:
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def get_stats(self) -> dict:
        """Get bridge statistics."""
        return {
            "vio_received": self._vio_recv_count,
            "vio_sent": self._vio_send_count,
            "send_errors": self._send_errors,
        }


def main():
    parser = argparse.ArgumentParser(description="ROS2-HTTP Bridge for NOMAD")
    parser.add_argument("--host", default="172.17.0.1", help="Edge Core host")
    parser.add_argument("--port", type=int, default=8000, help="Edge Core port")
    parser.add_argument("--vio-topic", default="/visual_slam/tracking/odometry",
                        help="VIO odometry topic")
    parser.add_argument("--rate", type=float, default=30.0, help="Send rate Hz")
    args = parser.parse_args()
    
    rclpy.init()
    
    bridge = ROSHTTPBridge(
        host=args.host,
        port=args.port,
        vio_topic=args.vio_topic,
        send_rate_hz=args.rate,
    )
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        stats = bridge.get_stats()
        logger.info(f"Bridge stats: {stats}")
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
