#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
echo "=== Test with max verbosity ==="
python3 -c "
import os
os.environ['ZED_SDK_VERBOSE'] = '100'
os.environ['SL_VERBOSE'] = '100'
import pyzed.sl as sl

cam = sl.Camera()

# Try forcing camera ID 0
init = sl.InitParameters()
init.depth_mode = sl.DEPTH_MODE.NONE
init.sdk_verbose = 100
init.sdk_verbose_log_file = '/tmp/zed_verbose.log'

# Try USB 2.0 mode explicitly
try:
    init.camera_resolution = sl.RESOLUTION.VGA
    init.camera_fps = 15
except:
    pass

s = cam.open(init)
print('open result:', s)
cam.close()
" 2>&1

echo ""
echo "=== SDK verbose log ==="
cat /tmp/zed_verbose.log 2>/dev/null | head -100 || echo "No log file"

echo ""
echo "=== Check what is on USB3 bus ==="
for d in /sys/bus/usb/devices/2-*; do
    if [ -f "$d/idVendor" ]; then
        vid=$(cat "$d/idVendor")
        pid=$(cat "$d/idProduct")
        prod=$(cat "$d/product" 2>/dev/null || echo "unknown")
        speed=$(cat "$d/speed" 2>/dev/null || echo "?")
        echo "$(basename $d): VID=$vid PID=$pid $prod @ ${speed}Mbps"
    fi
done

echo ""
echo "=== USB hub info (1-2.1) ==="
cat /sys/bus/usb/devices/1-2.1/product 2>/dev/null
cat /sys/bus/usb/devices/1-2.1/manufacturer 2>/dev/null
cat /sys/bus/usb/devices/1-2.1/idVendor 2>/dev/null
cat /sys/bus/usb/devices/1-2.1/idProduct 2>/dev/null
cat /sys/bus/usb/devices/1-2.1/version 2>/dev/null
