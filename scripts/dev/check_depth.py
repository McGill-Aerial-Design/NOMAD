#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class DepthCheck(Node):
    def __init__(self):
        super().__init__('depth_check')
        self.done = False
        self.create_subscription(
            Image, '/zed/zed_node/depth/depth_registered', self.cb, 1)

    def cb(self, msg):
        if self.done:
            return
        self.done = True
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
        valid = arr[np.isfinite(arr) & (arr > 0.05) & (arr < 50.0)]
        cx, cy = arr.shape[1] // 2, arr.shape[0] // 2
        center_roi = arr[cy-10:cy+11, cx-10:cx+11]
        center_valid = center_roi[np.isfinite(center_roi) & (center_roi > 0.05)]
        print(f"encoding={msg.encoding} shape={arr.shape}")
        print(f"total={arr.size} valid={len(valid)} nan={int(np.sum(np.isnan(arr)))} inf={int(np.sum(np.isinf(arr)))}")
        if len(valid):
            print(f"depth range: {valid.min():.2f}m - {valid.max():.2f}m  median={np.median(valid):.2f}m")
        else:
            print("ALL INVALID - no valid depth pixels anywhere")
        print(f"center 20x20 roi: {len(center_valid)} valid pixels", end="")
        if len(center_valid):
            print(f"  median={np.median(center_valid):.2f}m")
        else:
            print(" -> ALL NaN")

rclpy.init()
n = DepthCheck()
for _ in range(40):
    rclpy.spin_once(n, timeout_sec=0.5)
    if n.done:
        break
if not n.done:
    print("TIMEOUT: no depth message received")
rclpy.shutdown()
