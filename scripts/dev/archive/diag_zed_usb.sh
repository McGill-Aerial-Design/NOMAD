#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Trace ZED SDK USB access to find why camera not detected
echo "=== Checking USB device permissions ==="
ls -la /dev/bus/usb/001/033 /dev/bus/usb/001/034 /dev/hidraw* 2>/dev/null

echo ""
echo "=== Testing direct USB access ==="
python3 -c "
import ctypes
import ctypes.util

lib = ctypes.util.find_library('usb-1.0')
print('libusb found at:', lib)

libusb = ctypes.CDLL('libusb-1.0.so.0')
ctx = ctypes.c_void_p()
rc = libusb.libusb_init(ctypes.byref(ctx))
print('libusb_init:', rc)

# Get device list
devlist = ctypes.POINTER(ctypes.c_void_p)()
cnt = libusb.libusb_get_device_list(ctx, ctypes.byref(devlist))
print('USB devices found:', cnt)

# Cleanup
libusb.libusb_free_device_list(devlist, 1)
libusb.libusb_exit(ctx)
"

echo ""
echo "=== Testing HIDAPI ==="
python3 -c "
import ctypes
try:
    hidapi = ctypes.CDLL('libhidapi-libusb.so.0')
    print('hidapi-libusb loaded')

    class hid_device_info(ctypes.Structure):
        pass
    hid_device_info._fields_ = [
        ('path', ctypes.c_char_p),
        ('vendor_id', ctypes.c_ushort),
        ('product_id', ctypes.c_ushort),
        ('serial_number', ctypes.c_wchar_p),
        ('release_number', ctypes.c_ushort),
        ('manufacturer_string', ctypes.c_wchar_p),
        ('product_string', ctypes.c_wchar_p),
        ('usage_page', ctypes.c_ushort),
        ('usage', ctypes.c_ushort),
        ('interface_number', ctypes.c_int),
        ('next', ctypes.POINTER(hid_device_info)),
    ]

    hidapi.hid_init()
    hidapi.hid_enumerate.restype = ctypes.POINTER(hid_device_info)

    # Enumerate all HID devices
    devs = hidapi.hid_enumerate(0, 0)
    if devs:
        d = devs
        while d:
            info = d.contents
            print(f'  HID: VID={info.vendor_id:04x} PID={info.product_id:04x} path={info.path}')
            if info.vendor_id == 0x2b03:
                print(f'    ^^^ STEREOLABS DEVICE ^^^')
            if info.next:
                d = info.next
            else:
                break
        hidapi.hid_free_enumeration(devs)
    else:
        print('  No HID devices found')
    hidapi.hid_exit()
except Exception as e:
    print(f'HIDAPI error: {e}')

try:
    hidapi2 = ctypes.CDLL('libhidapi-hidraw.so.0')
    print('hidapi-hidraw loaded')
    hidapi2.hid_init()
    hidapi2.hid_enumerate.restype = ctypes.POINTER(hid_device_info)
    devs = hidapi2.hid_enumerate(0x2b03, 0)
    if devs:
        d = devs
        while d:
            info = d.contents
            print(f'  HIDRAW Stereo: VID={info.vendor_id:04x} PID={info.product_id:04x}')
            if info.next:
                d = info.next
            else:
                break
        hidapi2.hid_free_enumeration(devs)
    else:
        print('  No Stereolabs HID devices via hidraw')
    hidapi2.hid_exit()
except Exception as e:
    print(f'HIDAPI-hidraw: {e}')
"

echo ""
echo "=== strace quick check ==="
apt install -y strace > /dev/null 2>&1
timeout 5 strace -f -e trace=openat,ioctl -o /tmp/zed_strace.txt python3 -c "
import pyzed.sl as sl
cam = sl.Camera()
devs = cam.get_device_list()
print('cameras:', len(devs))
" 2>/dev/null
grep -i 'usb\|hid\|video\|2b03' /tmp/zed_strace.txt | head -20
