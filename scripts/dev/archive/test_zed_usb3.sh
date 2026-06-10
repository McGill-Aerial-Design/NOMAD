#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
echo "=== Check USB inside container ==="
lsusb 2>/dev/null | grep -i stereo || echo "No lsusb or no stereo"

echo ""
echo "=== ZED sysfs inside container ==="
for d in /sys/bus/usb/devices/*/idVendor; do
    dir=$(dirname "$d")
    vid=$(cat "$d" 2>/dev/null)
    if [ "$vid" = "2b03" ]; then
        dev=$(basename "$dir")
        speed=$(cat "$dir/speed" 2>/dev/null)
        prod=$(cat "$dir/product" 2>/dev/null)
        drv0=$(readlink "$dir/${dev}:1.0/driver" 2>/dev/null || echo "none")
        echo "$dev: $prod @ ${speed}Mbps driver=$drv0"
    fi
done

echo ""
echo "=== Unbind uvcvideo ==="
echo "2-1.4:1.0" > /sys/bus/usb/drivers/uvcvideo/unbind 2>&1 && echo "OK 1.0" || echo "FAIL 1.0"
echo "2-1.4:1.1" > /sys/bus/usb/drivers/uvcvideo/unbind 2>&1 && echo "OK 1.1" || echo "FAIL 1.1"

echo ""
echo "=== Test ZED SDK ==="
python3 << 'PYEOF'
import pyzed.sl as sl
cam = sl.Camera()
devs = cam.get_device_list()
print("ZED cameras:", len(devs))
for d in devs:
    print(f"  Model: {d.camera_model}, Serial: {d.serial_number}")

if len(devs) == 0:
    print("Trying direct open...")
    init = sl.InitParameters()
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.camera_resolution = sl.RESOLUTION.VGA
    init.camera_fps = 15
    init.sdk_verbose = 1
    status = cam.open(init)
    print("cam.open():", status)
    if status == sl.ERROR_CODE.SUCCESS:
        info = cam.get_camera_information()
        print("SUCCESS:", info.camera_model, "SN:", info.serial_number)
        cam.close()
PYEOF
