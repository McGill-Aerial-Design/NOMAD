#!/usr/bin/env python3
"""
Compare ZED odom orientation vs IMU orientation.
Run inside Isaac ROS container: python3 compare_odom_imu.py
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math

def quat_to_euler(x, y, z, w):
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

class CompareNode(Node):
    def __init__(self):
        super().__init__('compare_odom_imu')
        
        self.odom_euler = (0, 0, 0)
        self.imu_euler = (0, 0, 0)
        self.count = 0
        
        # Subscribe to odom (VIO pose)
        self.odom_sub = self.create_subscription(
            Odometry, '/zed/zed_node/odom', self.odom_callback, 10)
        
        # Subscribe to IMU (calibrated, gravity-aligned)
        self.imu_sub = self.create_subscription(
            Imu, '/zed/zed_node/imu/data', self.imu_callback, 10)
        
        # Timer to print comparison
        self.timer = self.create_timer(1.0, self.print_comparison)
        
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.odom_euler = quat_to_euler(q.x, q.y, q.z, q.w)
        
    def imu_callback(self, msg):
        q = msg.orientation
        self.imu_euler = quat_to_euler(q.x, q.y, q.z, q.w)
        
    def print_comparison(self):
        odom_r, odom_p, odom_y = [math.degrees(a) for a in self.odom_euler]
        imu_r, imu_p, imu_y = [math.degrees(a) for a in self.imu_euler]
        
        print(f"\n{'='*60}")
        print(f"ODOM (VIO, drifts):   roll={odom_r:7.1f}  pitch={odom_p:7.1f}  yaw={odom_y:7.1f}")
        print(f"IMU  (gravity-ref):   roll={imu_r:7.1f}  pitch={imu_p:7.1f}  yaw={imu_y:7.1f}")
        print(f"DIFF (odom - imu):    roll={odom_r-imu_r:7.1f}  pitch={odom_p-imu_p:7.1f}  yaw={odom_y-imu_y:7.1f}")

def main():
    rclpy.init()
    node = CompareNode()
    print("Comparing /zed/zed_node/odom vs /zed/zed_node/imu/data")
    print("Place camera flat - IMU roll/pitch should be ~0")
    print("Odom will drift over time, IMU should stay stable")
    print("Press Ctrl+C to exit")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
