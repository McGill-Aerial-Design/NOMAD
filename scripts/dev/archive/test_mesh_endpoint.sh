#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Check what bridge is actually doing with messages
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null

  # Try one-shot test: manually call Edge Core mesh update endpoint
  echo "=== Testing Edge Core mesh endpoint ==="
  curl -s -X POST http://localhost:8000/api/slam/mesh/update \
    -H "Content-Type: application/json" \
    -d "{\"blocks\":[{\"index\":[1,2,3],\"color\":[255,0,0]}],\"block_size\":0.3,\"mode\":\"blocks\",\"total_blocks\":1,\"timestamp\":$(date +%s).0,\"clear\":false}" \
    2>&1 | head -c 200

  echo ""
  echo ""
  echo "=== Bridge process errors in log ==="
  grep -iE "error|warn|exception" /tmp/ros_bridge2.log 2>/dev/null | tail -10

  echo ""
  echo "=== Bridge stats ==="
  grep -E "stats|received|sent" /tmp/ros_bridge2.log 2>/dev/null | tail -5
'
