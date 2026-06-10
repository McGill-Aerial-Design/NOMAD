#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Test ZED SDK camera detection. Run inside the Isaac ROS container."""

import pyzed.sl as sl

print("ZED SDK pyzed loaded")

cam = sl.Camera()

# Try to list devices
devs = sl.Camera.get_device_list()
print("Device list length:", len(devs))
for d in devs:
    print("  Device:", d)

# Try with explicit USB init
init = sl.InitParameters()
init.camera_resolution = sl.RESOLUTION.VGA
init.camera_fps = 15
init.depth_mode = sl.DEPTH_MODE.NONE
init.sdk_verbose = 1
err = cam.open(init)
print("Open result:", err)
if err == sl.ERROR_CODE.SUCCESS:
    info = cam.get_camera_information()
    print("Camera model:", info.camera_model)
    print("Serial:", info.serial_number)
cam.close()
