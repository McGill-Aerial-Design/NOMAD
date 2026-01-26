#!/usr/bin/env python3
"""Test cv2 import with cv_bridge."""
import numpy as np
print(f"numpy version: {np.__version__}")

# Import cv2 FIRST (like ros_video_bridge does)
import cv2
print(f"cv2 version: {cv2.__version__}")
print(f"cv2 location: {cv2.__file__}")

# Now import cv_bridge
from cv_bridge import CvBridge
print("CvBridge imported after cv2")

# Create bridge
bridge = CvBridge()
print("CvBridge instantiated")

# Create a test image
import sensor_msgs.msg
img_msg = sensor_msgs.msg.Image()
img_msg.height = 480
img_msg.width = 640
img_msg.encoding = 'bgr8'
img_msg.step = 640 * 3
img_msg.data = bytes(640 * 480 * 3)

print("Testing conversion...")
try:
    cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
    print(f"Success! Shape: {cv_image.shape}")
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
