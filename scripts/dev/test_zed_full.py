#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Test camera detection via v4l2, libusb, and ZED SDK. Run in container."""

import os

print("=== V4L2 Devices ===")
for dev in ["/dev/video0", "/dev/video1"]:
    try:
        with open(dev, "rb") as f:
            print(f"  {dev}: opened OK")
    except Exception as e:
        print(f"  {dev}: {e}")

# Check V4L2 info
try:
    import ctypes
    import fcntl

    class v4l2_capability(ctypes.Structure):
        _fields_ = [
            ("driver", ctypes.c_char * 16),
            ("card", ctypes.c_char * 32),
            ("bus_info", ctypes.c_char * 32),
            ("version", ctypes.c_uint32),
            ("capabilities", ctypes.c_uint32),
            ("device_caps", ctypes.c_uint32),
            ("reserved", ctypes.c_uint32 * 3),
        ]

    VIDIOC_QUERYCAP = 0x80685600

    for dev in ["/dev/video0", "/dev/video1"]:
        try:
            fd = os.open(dev, os.O_RDWR)
            cap = v4l2_capability()
            fcntl.ioctl(fd, VIDIOC_QUERYCAP, cap)
            print(f"  {dev} driver:{cap.driver.decode()} card:{cap.card.decode()} bus:{cap.bus_info.decode()}")
            os.close(fd)
        except Exception as e:
            print(f"  {dev} ioctl error: {e}")
except Exception as e:
    print(f"  V4L2 query error: {e}")

print("\n=== libusb ZED detection ===")
try:
    import ctypes as ct

    lib = ct.CDLL("libusb-1.0.so.0")

    class libusb_device_descriptor(ct.Structure):
        _fields_ = [
            ("bLength", ct.c_uint8),
            ("bDescriptorType", ct.c_uint8),
            ("bcdUSB", ct.c_uint16),
            ("bDeviceClass", ct.c_uint8),
            ("bDeviceSubClass", ct.c_uint8),
            ("bDeviceProtocol", ct.c_uint8),
            ("bMaxPacketSize0", ct.c_uint8),
            ("idVendor", ct.c_uint16),
            ("idProduct", ct.c_uint16),
            ("bcdDevice", ct.c_uint16),
            ("iManufacturer", ct.c_uint8),
            ("iProduct", ct.c_uint8),
            ("iSerialNumber", ct.c_uint8),
            ("bNumConfigurations", ct.c_uint8),
        ]

    ctx = ct.c_void_p()
    lib.libusb_init(ct.byref(ctx))

    devs = ct.POINTER(ct.c_void_p)()
    cnt = lib.libusb_get_device_list(ctx, ct.byref(devs))

    zed_found = False
    for i in range(cnt):
        desc = libusb_device_descriptor()
        lib.libusb_get_device_descriptor(devs[i], ct.byref(desc))
        vid = desc.idVendor
        pid = desc.idProduct
        if vid == 0x2B03:
            print(f"  ZED device found! VID:{vid:04x} PID:{pid:04x}")
            zed_found = True

    if not zed_found:
        print("  ZED NOT found via libusb (VID 2b03)")
        print(
            "  All USB VIDs:",
            [
                f"{desc.idVendor:04x}"
                for i in range(cnt)
                if not lib.libusb_get_device_descriptor(devs[i], ct.byref(desc))
            ],
        )

    lib.libusb_free_device_list(devs, 1)
    lib.libusb_exit(ctx)
except Exception as e:
    print(f"  libusb error: {e}")

print("\n=== ZED SDK ===")
os.environ["EGL_PLATFORM"] = "device"
import pyzed.sl as sl

cam = sl.Camera()
devs = sl.Camera.get_device_list()
print(f"  ZED SDK detected: {len(devs)} cameras")

init = sl.InitParameters()
init.depth_mode = sl.DEPTH_MODE.NONE
init.camera_resolution = sl.RESOLUTION.VGA
init.camera_fps = 15
init.sdk_verbose = 1
err = cam.open(init)
print(f"  Open result: {err}")
cam.close()
