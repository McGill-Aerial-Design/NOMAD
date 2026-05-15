#!/bin/bash
echo "=== ZED on USB3 - deep trace ==="

# Unbind uvcvideo first
echo "2-1.4:1.0" > /sys/bus/usb/drivers/uvcvideo/unbind 2>&1
echo "2-1.4:1.1" > /sys/bus/usb/drivers/uvcvideo/unbind 2>&1

strace -f -e trace=openat,read -o /tmp/zed_usb3_trace.txt python3 << 'PYEOF'
import pyzed.sl as sl
cam = sl.Camera()
devs = cam.get_device_list()
print("CAMERAS:", len(devs))
PYEOF

echo ""
echo "=== Bus 2 device access ==="
grep '2-1\.4' /tmp/zed_usb3_trace.txt

echo ""
echo "=== USB device node opens ==="
grep '/dev/bus/usb' /tmp/zed_usb3_trace.txt | grep -v DIRECTORY

echo ""
echo "=== idVendor/idProduct reads ==="
grep 'idVendor\|idProduct\|busnum\|devnum' /tmp/zed_usb3_trace.txt | head -20

echo ""
echo "=== libusb error calls ==="
grep -i 'error\|EACCES\|EBUSY\|EPERM' /tmp/zed_usb3_trace.txt | head -10

echo ""
echo "=== Direct libusb open test ==="
python3 << 'PYEOF'
import ctypes

libusb = ctypes.CDLL("libusb-1.0.so.0")
ctx = ctypes.c_void_p()
libusb.libusb_init(ctypes.byref(ctx))

devlist = ctypes.POINTER(ctypes.c_void_p)()
cnt = libusb.libusb_get_device_list(ctx, ctypes.byref(devlist))
print(f"Total USB devices: {cnt}")

# Try to find and open the ZED
class DevDesc(ctypes.Structure):
    _fields_ = [
        ("bLength", ctypes.c_uint8),
        ("bDescriptorType", ctypes.c_uint8),
        ("bcdUSB", ctypes.c_uint16),
        ("bDeviceClass", ctypes.c_uint8),
        ("bDeviceSubClass", ctypes.c_uint8),
        ("bDeviceProtocol", ctypes.c_uint8),
        ("bMaxPacketSize0", ctypes.c_uint8),
        ("idVendor", ctypes.c_uint16),
        ("idProduct", ctypes.c_uint16),
        ("bcdDevice", ctypes.c_uint16),
        ("iManufacturer", ctypes.c_uint8),
        ("iProduct", ctypes.c_uint8),
        ("iSerialNumber", ctypes.c_uint8),
        ("bNumConfigurations", ctypes.c_uint8),
    ]

libusb.libusb_get_device_descriptor.argtypes = [ctypes.c_void_p, ctypes.POINTER(DevDesc)]

for i in range(cnt):
    dev = devlist[i]
    desc = DevDesc()
    libusb.libusb_get_device_descriptor(dev, ctypes.byref(desc))
    if desc.idVendor == 0x2b03:
        bus = libusb.libusb_get_bus_number(dev)
        addr = libusb.libusb_get_device_address(dev)
        speed = libusb.libusb_get_device_speed(dev)
        speed_names = {0: "unknown", 1: "1.5M", 2: "12M", 3: "480M", 4: "5G", 5: "10G"}
        print(f"  ZED found: VID={desc.idVendor:04x} PID={desc.idProduct:04x} bus={bus} addr={addr} speed={speed_names.get(speed, speed)}")
        
        # Try to open
        handle = ctypes.c_void_p()
        rc = libusb.libusb_open(dev, ctypes.byref(handle))
        if rc == 0:
            print(f"    libusb_open: SUCCESS (handle={handle.value})")
            libusb.libusb_close(handle)
        else:
            print(f"    libusb_open: FAILED (rc={rc})")

libusb.libusb_free_device_list(devlist, 1)
libusb.libusb_exit(ctx)
PYEOF
