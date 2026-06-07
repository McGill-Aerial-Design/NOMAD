#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors

echo "=== Direct libusb via pyusb ==="
pip3 install pyusb > /dev/null 2>&1

python3 << 'PYEOF'
import usb.core
import usb.util

# Find all Stereolabs devices
devs = list(usb.core.find(find_all=True, idVendor=0x2b03))
print(f"pyusb found {len(devs)} Stereolabs devices:")
for dev in devs:
    print(f"  VID={dev.idVendor:04x} PID={dev.idProduct:04x} Bus={dev.bus} Addr={dev.address}")
    print(f"  Speed={dev.speed} Manufacturer={dev.manufacturer} Product={dev.product} Serial={dev.serial_number}")
    print(f"  bcdUSB={dev.bcdUSB:04x} bcdDevice={dev.bcdDevice:04x}")
    print(f"  Configs: {dev.bNumConfigurations}")

    # List interfaces
    for cfg in dev:
        print(f"  Config {cfg.bConfigurationValue}:")
        for intf in cfg:
            print(f"    Interface {intf.bInterfaceNumber} alt={intf.bAlternateSetting}")
            print(f"      Class={intf.bInterfaceClass:02x} SubClass={intf.bInterfaceSubClass:02x} Protocol={intf.bInterfaceProtocol:02x}")
            drv = None
            try:
                drv = dev.is_kernel_driver_active(intf.bInterfaceNumber)
            except:
                pass
            print(f"      Kernel driver active: {drv}")
            for ep in intf:
                print(f"      EP {ep.bEndpointAddress:02x} Type={ep.bmAttributes:02x} MaxPacket={ep.wMaxPacketSize}")

    # Try to claim
    print(f"  Trying to claim...")
    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
            print(f"  Detached kernel driver from interface 0")
        dev.set_configuration()
        print(f"  Configuration set successfully")
        usb.util.dispose_resources(dev)
    except Exception as e:
        print(f"  Claim failed: {e}")

    print()

PYEOF

echo ""
echo "=== Now test ZED SDK after pyusb claimed ==="
python3 << 'PYEOF'
import pyzed.sl as sl
cam = sl.Camera()
devs = cam.get_device_list()
print(f"ZED SDK after pyusb: {len(devs)} cameras")
PYEOF
