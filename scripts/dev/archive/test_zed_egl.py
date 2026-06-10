#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
Test ZED SDK camera detection with various workarounds.
Run inside the Isaac ROS container.
"""

import ctypes
import ctypes.util
import os

# Set EGL platform for headless mode
os.environ["EGL_PLATFORM"] = "device"
os.environ["DISPLAY"] = ""

# Enumerate USB devices with libusb to find ZED
print("=== libusb enumeration ===")
try:
    lib = ctypes.CDLL("libusb-1.0.so.0")

    ctx = ctypes.c_void_p()
    ret = lib.libusb_init(ctypes.byref(ctx))

    if ret == 0:
        devs = ctypes.POINTER(ctypes.c_void_p)()
        cnt = lib.libusb_get_device_list(ctx, ctypes.byref(devs))
        print("Total USB devices:", cnt)
        lib.libusb_free_device_list(devs, 1)
        lib.libusb_exit(ctx)
except Exception as e:
    print("libusb error:", e)

# Try ZED SDK
print("\n=== ZED SDK detection ===")
import pyzed.sl as sl

cam = sl.Camera()
devs = sl.Camera.get_device_list()
print("ZED device list:", len(devs))

# Try with init params
init = sl.InitParameters()
init.depth_mode = sl.DEPTH_MODE.NONE
init.camera_resolution = sl.RESOLUTION.VGA
init.camera_fps = 15
init.sdk_verbose = 1
init.sdk_gpu_id = 0

# Try open
err = cam.open(init)
print("Open result:", err)
if err == sl.ERROR_CODE.SUCCESS:
    info = cam.get_camera_information()
    print("Model:", info.camera_model)
    print("Serial:", info.serial_number)
    print("Firmware:", info.camera_configuration.firmware_version)
cam.close()
