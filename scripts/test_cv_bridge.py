#!/usr/bin/env python3
"""Test cv_bridge and numpy compatibility."""
import numpy
print(f"numpy version: {numpy.__version__}")

try:
    from cv_bridge import CvBridge
    print(f"cv_bridge loaded: {CvBridge}")
    bridge = CvBridge()
    print("CvBridge instantiated successfully")
except Exception as e:
    print(f"cv_bridge error: {e}")

try:
    import cv2
    print(f"OpenCV version: {cv2.__version__}")
except Exception as e:
    print(f"OpenCV error: {e}")
