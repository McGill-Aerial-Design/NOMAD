#!/bin/bash
echo "=== Container USB device access test ==="
ls -la /dev/bus/usb/002/004
echo ""

echo "=== Try open O_RDWR as root ==="
python3 << 'PYEOF'
import os
try:
    fd = os.open("/dev/bus/usb/002/004", os.O_RDWR)
    print(f"SUCCESS: opened fd={fd}")
    os.close(fd)
except Exception as e:
    print(f"FAILED: {e}")
PYEOF

echo ""
echo "=== Fix permissions and retest ==="
chmod 666 /dev/bus/usb/002/004
ls -la /dev/bus/usb/002/004
echo ""

python3 << 'PYEOF'
import pyzed.sl as sl
cam = sl.Camera()
devs = cam.get_device_list()
print(f"After chmod 666: ZED cameras = {len(devs)}")
for d in devs:
    print(f"  Model: {d.camera_model}, Serial: {d.serial_number}")
PYEOF
