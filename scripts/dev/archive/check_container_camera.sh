#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Check ZED camera visibility from inside the container
docker exec nomad_isaac_ros bash -c '
  echo "=== USB devices ==="
  lsusb 2>/dev/null | grep -i stereo || echo "NO_STEREOLABS_USB"

  echo ""
  echo "=== Video devices ==="
  ls -la /dev/video* 2>/dev/null || echo "NO_VIDEO_DEVICES"

  echo ""
  echo "=== ZED SDK version ==="
  cat /usr/local/zed/settings/zed_settings.conf 2>/dev/null | head -5 || echo "NO_ZED_SETTINGS"
  dpkg -l 2>/dev/null | grep -i zed | head -5 || echo "CHECK_ZED_SDK"
  ls /usr/local/zed/lib/libsl_zed.so 2>/dev/null && echo "ZED_SDK_LIB_OK" || echo "NO_ZED_SDK_LIB"

  echo ""
  echo "=== Test open camera in Python ==="
  python3 -c "
import sys
sys.path.insert(0, \"/usr/local/zed/lib\")
try:
    import pyzed.sl as sl
    c = sl.Camera()
    p = sl.InitParameters()
    p.depth_mode = sl.DEPTH_MODE.NONE
    p.camera_resolution = sl.RESOLUTION.VGA
    p.camera_fps = 15
    s = c.open(p)
    print(\"Camera open result:\", s)
    if s == sl.ERROR_CODE.SUCCESS:
        info = c.get_camera_information()
        print(\"Camera model:\", info.camera_model)
        c.close()
    else:
        print(\"Error details:\", sl.ERROR_CODE(s))
except Exception as e:
    print(\"Exception:\", e)
" 2>&1
'
