#!/usr/bin/env python3
"""Test receiving real ROS images."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

print(f"numpy version: {np.__version__}")

class TestNode(Node):
    def __init__(self):
        super().__init__('test_image_node')
        self.bridge = CvBridge()
        self.count = 0
        
        self.sub = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.callback,
            10
        )
        print("Subscribed to /zed/zed_node/rgb/image_rect_color")
        
    def callback(self, msg):
        self.count += 1
        print(f"Frame {self.count}: {msg.width}x{msg.height}, encoding={msg.encoding}")
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            print(f"  Converted to cv2: shape={cv_image.shape}, dtype={cv_image.dtype}")
        except Exception as e:
            print(f"  Conversion error: {e}")
            import traceback
            traceback.print_exc()
        
        if self.count >= 3:
            print("Test complete!")
            raise SystemExit(0)

def main():
    rclpy.init()
    node = TestNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
