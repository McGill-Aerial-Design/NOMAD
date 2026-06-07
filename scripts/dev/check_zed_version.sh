#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Check ZED SDK version inside container vs host
echo "=== Container ZED SDK ==="
docker exec nomad_isaac_ros bash -c '
  ls -la /usr/local/zed/lib/libsl_zed.so 2>/dev/null
  strings /usr/local/zed/lib/libsl_zed.so 2>/dev/null | grep -oP "ZED SDK v[\d.]+" | head -1 || true
  cat /usr/local/zed/version 2>/dev/null || true
  cat /usr/local/zed/zed-sdk-version.txt 2>/dev/null || true
  ls /usr/local/zed/lib/*.so 2>/dev/null | head -10
  python3 -c "import pyzed.sl as sl; print(\"pyzed:\", sl.Camera().get_sdk_version())" 2>&1 | head -3
'

echo ""
echo "=== Host ZED SDK ==="
cat /usr/local/zed/version 2>/dev/null || true
cat /usr/local/zed/zed-sdk-version.txt 2>/dev/null || true
strings /usr/local/zed/lib/libsl_zed.so 2>/dev/null | grep -oP "ZED SDK v[\d.]+" | head -1 || true
python3 -c "import pyzed.sl as sl; print('pyzed:', sl.Camera().get_sdk_version())" 2>&1 | head -3
