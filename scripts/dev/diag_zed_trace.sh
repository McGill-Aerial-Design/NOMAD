#!/bin/bash
# Deep USB trace for ZED SDK
echo "=== Full USB trace ==="
strace -f -e trace=openat -o /tmp/zed_strace2.txt python3 -c "
import pyzed.sl as sl
cam = sl.Camera()
devs = cam.get_device_list()
print('cameras:', len(devs))
" 2>/dev/null

echo "--- USB bus opens ---"
grep '/dev/bus/usb' /tmp/zed_strace2.txt

echo ""
echo "--- sysfs USB reads ---"
grep -c 'sys/bus/usb\|sys/devices.*usb' /tmp/zed_strace2.txt
echo "(total sysfs USB accesses)"

echo ""
echo "--- Looking for ZED VID/PID reads ---"
grep '1-2\.1\.2' /tmp/zed_strace2.txt | head -20

echo ""
echo "--- idVendor/idProduct reads ---"
grep 'idVendor\|idProduct' /tmp/zed_strace2.txt | head -20

echo ""
echo "--- Checking if SDK tries to open the camera USB device ---"
grep '001/033\|001/034' /tmp/zed_strace2.txt

echo ""
echo "=== Host comparison ==="
echo "Host USB devices at /sys/bus/usb/devices/:"
ls /sys/bus/usb/devices/ 2>/dev/null

echo ""
echo "=== ZED device sysfs info ==="
cat /sys/bus/usb/devices/1-2.1.2/idVendor 2>/dev/null && echo " (ZED VID)"
cat /sys/bus/usb/devices/1-2.1.2/idProduct 2>/dev/null && echo " (ZED PID)"
cat /sys/bus/usb/devices/1-2.1.2/product 2>/dev/null && echo " (product)"
cat /sys/bus/usb/devices/1-2.1.2/manufacturer 2>/dev/null && echo " (manufacturer)"
cat /sys/bus/usb/devices/1-2.1.2/serial 2>/dev/null && echo " (serial)"
cat /sys/bus/usb/devices/1-2.1.2/speed 2>/dev/null && echo " Mbps (speed)"
cat /sys/bus/usb/devices/1-2.1.2/bNumConfigurations 2>/dev/null && echo " (configs)"
