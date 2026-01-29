#!/usr/bin/env python3
"""
VIO Bridge: ZED Odometry -> MAVROS Vision Pose
Converts ZED VIO to MAVROS vision_pose format for ArduPilot EKF

This node bridges the ZED camera's visual-inertial odometry to ArduPilot's
external navigation input via MAVROS. ArduPilot uses this for indoor positioning.

Run: python3 vio_bridge.py
Or: ros2 run nomad_ros vio_bridge

Topics:
    Subscribes: /zed/zed_node/odom (nav_msgs/Odometry)
    Publishes: /mavros/vision_pose/pose (geometry_msgs/PoseStamped)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class VIOBridge(Node):
    """Bridge ZED VIO odometry to MAVROS vision pose for ArduPilot EKF."""
    
    def __init__(self):
        super().__init__('vio_bridge')
        
        # QoS for sensor data - best effort, keep last
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to ZED odometry
        self.sub = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_callback,
            sensor_qos
        )
        
        # Publish to MAVROS vision pose
        self.pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )
        
        # Stats
        self.msg_count = 0
        self.last_log_time = self.get_clock().now()
        
        self.get_logger().info(
            'VIO Bridge started: /zed/zed_node/odom -> /mavros/vision_pose/pose'
        )
    
    def odom_callback(self, msg: Odometry):
        """Convert Odometry to PoseStamped and publish."""
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = 'map'  # MAVROS expects 'map' frame
        pose_msg.pose = msg.pose.pose
        
        self.pub.publish(pose_msg)
        
        # Log stats every 5 seconds
        self.msg_count += 1
        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds / 1e9
        if elapsed >= 5.0:
            rate = self.msg_count / elapsed
            self.get_logger().info(f'VIO Bridge: {rate:.1f} Hz')
            self.msg_count = 0
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = VIOBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
