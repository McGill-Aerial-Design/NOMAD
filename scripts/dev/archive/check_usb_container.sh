#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Check USB device visibility inside container
docker exec nomad_isaac_ros bash -c '
  echo "=== USB devices via sysfs ==="
  for d in /sys/bus/usb/devices/[0-9]-[0-9]*; do
    v=$(cat $d/idVendor 2>/dev/null)
    p=$(cat $d/idProduct 2>/dev/null)
    prod=$(cat $d/product 2>/dev/null)
    [ -n "$v" ] && echo "  $(basename $d): $v:$p - $prod"
  done

  echo ""
  echo "=== Check udev rules ==="
  ls /etc/udev/rules.d/ 2>/dev/null | head -10 || echo "No udev rules dir"

  echo ""
  echo "=== ZED tool ==="
  /usr/local/zed/tools/ZED_Diagnostic -da 2>&1 | head -20 || echo "ZED diagnostic not found"
'
