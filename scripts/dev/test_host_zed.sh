#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
echo "=== Installing libturbojpeg on host ==="
apt-get install -y libturbojpeg0-dev 2>/dev/null || echo "apt failed, trying alternatives"
ldconfig

echo ""
echo "=== Test ZED on host ==="
python3 << 'PYEOF'
try:
    import pyzed.sl as sl
    cam = sl.Camera()
    devs = cam.get_device_list()
    print("HOST ZED cameras:", len(devs))
    for d in devs:
        print(f"  Model: {d.camera_model}, Serial: {d.serial_number}")

    if len(devs) == 0:
        print("Trying direct open on host...")
        init = sl.InitParameters()
        init.depth_mode = sl.DEPTH_MODE.NONE
        init.sdk_verbose = 1
        s = cam.open(init)
        print("Host cam.open():", s)
        if s == sl.ERROR_CODE.SUCCESS:
            info = cam.get_camera_information()
            print("Host connected:", info.camera_model, "SN:", info.serial_number)
            cam.close()
except Exception as e:
    print(f"Host pyzed error: {e}")
PYEOF
