#!/bin/bash
echo "=== USB device file check ==="
ls -la /dev/bus/usb/002/ 2>/dev/null
echo ""

echo "=== Manual USB open test ==="
python3 << 'PYEOF'
import os
import struct

# Find the ZED device file by reading sysfs
for d in os.listdir("/sys/bus/usb/devices/"):
    vid_path = f"/sys/bus/usb/devices/{d}/idVendor"
    if os.path.exists(vid_path):
        vid = open(vid_path).read().strip()
        if vid == "2b03":
            pid = open(f"/sys/bus/usb/devices/{d}/idProduct").read().strip()
            busnum = open(f"/sys/bus/usb/devices/{d}/busnum").read().strip()
            devnum = open(f"/sys/bus/usb/devices/{d}/devnum").read().strip()
            speed = open(f"/sys/bus/usb/devices/{d}/speed").read().strip()
            product = open(f"/sys/bus/usb/devices/{d}/product").read().strip() if os.path.exists(f"/sys/bus/usb/devices/{d}/product") else "?"
            
            devpath = f"/dev/bus/usb/{int(busnum):03d}/{int(devnum):03d}"
            exists = os.path.exists(devpath)
            
            print(f"Device {d}: VID={vid} PID={pid} {product} @ {speed}Mbps")
            print(f"  Device file: {devpath} exists={exists}")
            
            if exists:
                try:
                    fd = os.open(devpath, os.O_RDWR)
                    print(f"  O_RDWR: SUCCESS (fd={fd})")
                    
                    # Read USB device descriptor (18 bytes)
                    desc = os.read(fd, 18)
                    if len(desc) >= 18:
                        bcdUSB, = struct.unpack_from("<H", desc, 2)
                        vid_r, = struct.unpack_from("<H", desc, 8)
                        pid_r, = struct.unpack_from("<H", desc, 10)
                        bcdDevice, = struct.unpack_from("<H", desc, 12)
                        print(f"  USB descriptor: bcdUSB={bcdUSB:04x} VID={vid_r:04x} PID={pid_r:04x} bcdDevice={bcdDevice:04x}")
                    
                    os.close(fd)
                except PermissionError as e:
                    print(f"  O_RDWR: PERMISSION DENIED: {e}")
                except Exception as e:
                    print(f"  O_RDWR: FAILED: {e}")
            else:
                print(f"  Device file MISSING!")

print()
print("=== ZED SDK internals check ===")
import pyzed.sl as sl

# Check if there are any static initialization issues
print("sl module loaded successfully")
print("sl.Camera class:", sl.Camera)

cam = sl.Camera()
print("Camera object created")

# Try with explicit input type
init = sl.InitParameters()
print("Available input types:")
try:
    print("  INPUT_TYPE:", dir(sl.INPUT_TYPE) if hasattr(sl, 'INPUT_TYPE') else "N/A")
except: pass

# Force input from USB  
try:
    init.input = sl.InputType()
    init.input.setFromCameraID(0)
    print("Set input from camera ID 0")
except Exception as e:
    print(f"setFromCameraID error: {e}")

devs = cam.get_device_list()
print(f"get_device_list: {len(devs)} cameras")
PYEOF
