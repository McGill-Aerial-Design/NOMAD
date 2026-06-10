#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
echo "=== Unbinding uvcvideo from ZED ==="
echo "1-2.1.1:1.0" > /sys/bus/usb/drivers/uvcvideo/unbind 2>&1 && echo "Unbound :1.0" || echo "Failed :1.0"
echo "1-2.1.1:1.1" > /sys/bus/usb/drivers/uvcvideo/unbind 2>&1 && echo "Unbound :1.1" || echo "Failed :1.1"

echo ""
echo "=== Verify no driver bound ==="
readlink /sys/bus/usb/devices/1-2.1.1:1.0/driver 2>/dev/null || echo "1.0: No driver"
readlink /sys/bus/usb/devices/1-2.1.1:1.1/driver 2>/dev/null || echo "1.1: No driver"

echo ""
echo "=== Test ZED SDK detection ==="
python3 -c "
import pyzed.sl as sl
cam = sl.Camera()
devs = cam.get_device_list()
print('ZED cameras after unbind:', len(devs))
for d in devs:
    print(f'  Model: {d.camera_model}, Serial: {d.serial_number}')
if len(devs) == 0:
    print('Still 0, trying direct open...')
    init = sl.InitParameters()
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.sdk_verbose = 1
    s = cam.open(init)
    print('cam.open():', s)
    if s == sl.ERROR_CODE.SUCCESS:
        info = cam.get_camera_information()
        print('Connected:', info.camera_model, 'SN:', info.serial_number)
        cam.close()
" 2>&1
