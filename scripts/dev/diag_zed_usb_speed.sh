#!/bin/bash
echo "=== USB speed chain ==="
echo -n "usb1 (root): "; cat /sys/bus/usb/devices/usb1/speed 2>/dev/null; echo " Mbps"
echo -n "1-2 (port): "; cat /sys/bus/usb/devices/1-2/speed 2>/dev/null; echo " Mbps"
echo -n "1-2.1 (hub): "; cat /sys/bus/usb/devices/1-2.1/speed 2>/dev/null; echo " Mbps"
echo -n "1-2.1.1 (ZED cam): "; cat /sys/bus/usb/devices/1-2.1.1/speed 2>/dev/null; echo " Mbps"
echo -n "1-2.1.2 (ZED HID): "; cat /sys/bus/usb/devices/1-2.1.2/speed 2>/dev/null; echo " Mbps"

echo ""
echo "=== USB version and max speed ==="
echo -n "1-2.1.1 version: "; cat /sys/bus/usb/devices/1-2.1.1/version 2>/dev/null
echo -n "1-2.1.1 bcdUSB: "; cat /sys/bus/usb/devices/1-2.1.1/bcdUSB 2>/dev/null

echo ""
echo "=== Kernel driver bound to interfaces ==="
echo -n "1-2.1.1:1.0 driver: "; readlink /sys/bus/usb/devices/1-2.1.1:1.0/driver 2>/dev/null || echo "NONE"
echo -n "1-2.1.1:1.1 driver: "; readlink /sys/bus/usb/devices/1-2.1.1:1.1/driver 2>/dev/null || echo "NONE"
echo -n "1-2.1.2:1.0 driver: "; readlink /sys/bus/usb/devices/1-2.1.2:1.0/driver 2>/dev/null || echo "NONE"

echo ""
echo "=== Interface classes ==="
echo -n "1-2.1.1:1.0 class: "; cat /sys/bus/usb/devices/1-2.1.1:1.0/bInterfaceClass 2>/dev/null
echo -n "1-2.1.1:1.1 class: "; cat /sys/bus/usb/devices/1-2.1.1:1.1/bInterfaceClass 2>/dev/null
echo -n "1-2.1.2:1.0 class: "; cat /sys/bus/usb/devices/1-2.1.2:1.0/bInterfaceClass 2>/dev/null

echo ""
echo "=== Are there USB3 XHCI ports? ==="
ls /sys/bus/usb/devices/ | sort

echo ""
echo "=== Is there a usb3 bus? ==="
for bus in /sys/bus/usb/devices/usb*; do
    b=$(basename $bus)
    speed=$(cat $bus/speed 2>/dev/null)
    echo "$b: $speed Mbps"
done

echo ""
echo "=== Try on host (not in container) ==="
echo "Install pyzed on host if available"
which python3
python3 -c "
try:
    import pyzed.sl as sl
    cam = sl.Camera()
    devs = cam.get_device_list()
    print('HOST ZED cameras:', len(devs))
    for d in devs:
        print(f'  Model: {d.camera_model}, Serial: {d.serial_number}')
except Exception as e:
    print(f'Host pyzed not available: {e}')
" 2>&1
