#!/bin/bash
echo "=== Main camera device (1-2.1.1) sysfs ==="
cat /sys/bus/usb/devices/1-2.1.1/idVendor 2>/dev/null; echo " VID"
cat /sys/bus/usb/devices/1-2.1.1/idProduct 2>/dev/null; echo " PID"
cat /sys/bus/usb/devices/1-2.1.1/product 2>/dev/null; echo " product"
cat /sys/bus/usb/devices/1-2.1.1/speed 2>/dev/null; echo " Mbps"
cat /sys/bus/usb/devices/1-2.1.1/bcdUSB 2>/dev/null; echo " bcdUSB"
cat /sys/bus/usb/devices/1-2.1.1/uevent 2>/dev/null; echo ""

echo "=== HID device (1-2.1.2) sysfs ==="
cat /sys/bus/usb/devices/1-2.1.2/uevent 2>/dev/null; echo ""

echo "=== Parent hub speed ==="
cat /sys/bus/usb/devices/1-2.1/speed 2>/dev/null; echo " Mbps (hub)"
cat /sys/bus/usb/devices/1-2/speed 2>/dev/null; echo " Mbps (root hub port)"

echo ""
echo "=== Full strace: all sysfs uevent reads ==="
strace -f -e trace=openat -o /tmp/zed3.txt python3 -c "
import pyzed.sl as sl; cam = sl.Camera(); print('cams:', len(cam.get_device_list()))
" 2>/dev/null
echo "--- All uevent reads ---"
grep 'uevent' /tmp/zed3.txt
echo ""
echo "--- All /dev/bus/usb opens ---"
grep '/dev/bus/usb/' /tmp/zed3.txt | grep -v DIRECTORY
echo ""
echo "--- Main camera device access ---"
grep '1-2\.1\.1' /tmp/zed3.txt | head -10
