#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
echo "=== pyzed version ==="
python3 -c "import pyzed.sl as sl; print(getattr(sl, '__version__', 'NO_VERSION'))" 2>&1

echo "=== pip show ==="
pip3 show pyzed 2>/dev/null || echo "NOT_PIP_INSTALLED"

echo "=== pyzed location ==="
python3 -c "import pyzed; print(pyzed.__file__)" 2>&1

echo "=== SDK version from file ==="
cat /usr/local/zed/include/sl/Camera.hpp 2>/dev/null | grep "ZED_SDK_" | head -5 || echo "NO_HEADER"

echo "=== ZED SDK tools version ==="
/usr/local/zed/tools/ZED_Explorer --version 2>&1 | head -3 || echo "CANT_RUN"

echo "=== colord-sane check ==="
ps aux | grep colord 2>/dev/null || echo "NO_COLORD"

echo "=== USB device open count ==="
lsof /dev/bus/usb/001/033 2>/dev/null | head -10 || echo "NO_LSOF"

echo "=== try kill colord and retest ==="
killall colord-sane 2>/dev/null
sleep 1
python3 -c "
import pyzed.sl as sl
cam = sl.Camera()
devs = cam.get_device_list()
print('After killing colord: cameras =', len(devs))
"
