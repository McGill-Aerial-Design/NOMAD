#!/usr/bin/env python3
"""Test cv_bridge cvtColor2 directly."""
import numpy as np
print(f"numpy version: {np.__version__}")
print(f"numpy location: {np.__file__}")

# Test creating numpy array
arr = np.zeros((480, 640, 3), dtype=np.uint8)
print(f"Created numpy array: {arr.shape}, dtype={arr.dtype}")

# Now import cv_bridge boost
try:
    from cv_bridge.boost.cv_bridge_boost import cvtColor2
    print("cvtColor2 imported successfully")
    
    # Try to call it
    result = cvtColor2(arr, 'bgr8', 'bgr8')
    print(f"cvtColor2 result: {result.shape}")
except Exception as e:
    print(f"cvtColor2 failed: {e}")
    import traceback
    traceback.print_exc()
