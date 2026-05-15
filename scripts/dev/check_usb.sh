#!/bin/bash
echo "=== ZED USB status ==="
lsusb | grep -i stereo

echo ""
echo "=== ZED device speeds ==="
for d in /sys/bus/usb/devices/*/idVendor; do
    dir=$(dirname "$d")
    vid=$(cat "$d" 2>/dev/null)
    if [ "$vid" = "2b03" ]; then
        dev=$(basename "$dir")
        speed=$(cat "$dir/speed" 2>/dev/null)
        prod=$(cat "$dir/product" 2>/dev/null)
        bus=$(cat "$dir/busnum" 2>/dev/null)
        echo "$dev: $prod @ ${speed}Mbps (bus $bus)"
    fi
done

echo ""
echo "=== All USB buses ==="
for bus in /sys/bus/usb/devices/usb*; do
    b=$(basename "$bus")
    speed=$(cat "$bus/speed" 2>/dev/null)
    echo "$b: $speed Mbps"
done
