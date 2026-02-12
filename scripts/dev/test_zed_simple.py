#!/usr/bin/env python3
"""Test ZED camera detection with proper NVIDIA EGL environment."""
import os

# Force device-level EGL (no display needed)
os.environ["EGL_PLATFORM"] = "device"
os.environ["QT_QPA_PLATFORM"] = "offscreen"
os.environ["__EGL_VENDOR_LIBRARY_DIRS"] = "/usr/share/glvnd/egl_vendor.d"

# Check EGL availability first
print("=== EGL Environment ===")
print(f"EGL_PLATFORM={os.environ.get('EGL_PLATFORM')}")
print(f"DISPLAY={os.environ.get('DISPLAY', 'unset')}")
print(f"NVIDIA_DRIVER_CAPABILITIES={os.environ.get('NVIDIA_DRIVER_CAPABILITIES', 'unset')}")

# Try loading EGL
try:
    import ctypes
    egl = ctypes.CDLL("libEGL.so.1")
    display = egl.eglGetDisplay(0)
    print(f"EGL display handle: {display}")
except Exception as e:
    print(f"EGL load failed: {e}")

print()
print("=== ZED SDK Detection ===")

import pyzed.sl as sl

cam = sl.Camera()
devs = cam.get_device_list()
print(f"ZED cameras via get_device_list(): {len(devs)}")
for d in devs:
    print(f"  Model: {d.camera_model}, Serial: {d.serial_number}")

if len(devs) == 0:
    print()
    print("Trying direct open with verbose SDK...")
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.AUTO
    init_params.depth_mode = sl.DEPTH_MODE.NONE
    init_params.sdk_verbose = 1
    status = cam.open(init_params)
    print(f"cam.open() returned: {status}")
    if status == sl.ERROR_CODE.SUCCESS:
        info = cam.get_camera_information()
        print(f"Connected: {info.camera_model} SN:{info.serial_number}")
        cam.close()
    else:
        print(f"Open failed: {repr(status)}")
