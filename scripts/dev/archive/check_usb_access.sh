#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Check USB access and libusb inside container
docker exec nomad_isaac_ros bash -c '
  echo "=== USB device file permissions ==="
  ls -la /dev/bus/usb/002/003 2>/dev/null  # ZED 2i main device
  ls -la /dev/bus/usb/001/005 2>/dev/null  # ZED 2i HID

  echo ""
  echo "=== Can we read USB device? ==="
  cat /dev/bus/usb/002/003 > /dev/null 2>&1 && echo "READ_OK" || echo "READ_FAIL (check permissions)"

  echo ""
  echo "=== libusb ==="
  dpkg -l 2>/dev/null | grep libusb
  find / -name "libusb*" -type f 2>/dev/null | head -5

  echo ""
  echo "=== Running as user ==="
  id

  echo ""
  echo "=== ZED SDK Python test with verbose error ==="
  python3 -c "
import pyzed.sl as sl
print(\"SDK version:\", sl.Camera.get_sdk_version())
c = sl.Camera()
p = sl.InitParameters()
p.depth_mode = sl.DEPTH_MODE.NONE
p.camera_resolution = sl.RESOLUTION.VGA
p.camera_fps = 15
p.sdk_verbose_log_file = \"/tmp/zed_verbose.log\"
p.sdk_verbose = 1
s = c.open(p)
print(\"Result:\", s)
c.close()
" 2>&1

  echo ""
  echo "=== ZED verbose log ==="
  tail -30 /tmp/zed_verbose.log 2>/dev/null || echo "No verbose log"
'
