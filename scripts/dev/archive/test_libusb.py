#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Minimal libusb test to check if ZED camera (VID 2b03) is detectable."""

import ctypes
import ctypes.util

# Load libusb
lib_name = ctypes.util.find_library("usb-1.0")
print("libusb path:", lib_name)

if lib_name:
    lib = ctypes.CDLL(lib_name)

    # Init libusb
    ctx = ctypes.c_void_p()
    ret = lib.libusb_init(ctypes.byref(ctx))
    print("libusb_init:", ret)

    if ret == 0:
        # Get device list
        devs = ctypes.POINTER(ctypes.c_void_p)()
        cnt = lib.libusb_get_device_list(ctx, ctypes.byref(devs))
        print("USB devices found:", cnt)
        lib.libusb_free_device_list(devs, 1)
        lib.libusb_exit(ctx)
else:
    print("libusb not found!")
