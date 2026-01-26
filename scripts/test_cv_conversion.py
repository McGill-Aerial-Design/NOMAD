#!/usr/bin/env python3
"""Test cv_bridge image conversion."""
import numpy as np
print(f"numpy version: {np.__version__}")
print(f"numpy API version: {np.__array_api_version__}" if hasattr(np, '__array_api_version__') else "No array API version")

from cv_bridge import CvBridge
import sensor_msgs.msg

bridge = CvBridge()

# Create a fake ROS image message
img_msg = sensor_msgs.msg.Image()
img_msg.height = 480
img_msg.width = 640
img_msg.encoding = 'bgr8'
img_msg.step = 640 * 3
img_msg.data = bytes(640 * 480 * 3)

print("Created fake image message")

try:
    cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
    print(f"Conversion successful! Shape: {cv_image.shape}")
except Exception as e:
    print(f"Conversion failed: {e}")
    import traceback
    traceback.print_exc()
