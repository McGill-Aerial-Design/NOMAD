#!/usr/bin/env python3
"""Diagnostic: print quaternion and euler from ZED odom topic."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

def quat_to_euler(x, y, z, w):
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

class OdomDiagNode(Node):
    def __init__(self):
        super().__init__('odom_diag')
        self.sub = self.create_subscription(
            Odometry, '/zed/zed_node/odom', self.callback, 10)
        self.count = 0
        
    def callback(self, msg):
        q = msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        
        if self.count % 30 == 0:  # Print every ~1 second at 30Hz
            print(f"Quat: x={q.x:.4f} y={q.y:.4f} z={q.z:.4f} w={q.w:.4f}")
            print(f"Euler: roll={math.degrees(roll):.1f}° pitch={math.degrees(pitch):.1f}° yaw={math.degrees(yaw):.1f}°")
            print("---")
        self.count += 1

def main():
    rclpy.init()
    node = OdomDiagNode()
    print("Subscribing to /zed/zed_node/odom - rotate camera and watch values...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
