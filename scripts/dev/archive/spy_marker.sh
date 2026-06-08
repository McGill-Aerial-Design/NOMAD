#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Check why bridge isn't processing voxel markers
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null

  echo "=== Bridge process stats ==="
  ps aux | grep ros_http_bridge | grep -v grep | awk "{print \$1, \$3, \$4, \$6}"

  echo ""
  echo "=== Bridge restart log (last 15 lines) ==="
  cat /tmp/ros_bridge3.log 2>/dev/null

  echo ""
  echo "=== color_layer_marker publisher count NOW ==="
  timeout 2 ros2 topic info /nvblox_node/color_layer_marker 2>/dev/null | grep -E "Publisher|Subscription|Node name"

  echo ""
  echo "=== Run a quick spy: receive 1 marker from topic ==="
  python3 -c "
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import time

rclpy.init()
node = Node(\"spy\")
received = []
def cb(msg):
    received.append((msg.type, len(msg.points), len(msg.colors)))

sub = node.create_subscription(Marker, \"/nvblox_node/color_layer_marker\", cb, 10)
deadline = time.time() + 5
while time.time() < deadline and not received:
    rclpy.spin_once(node, timeout_sec=0.1)

if received:
    print(f\"GOT MARKER: type={received[0][0]} points={received[0][1]} colors={received[0][2]}\")
else:
    print(\"NO MARKER RECEIVED IN 5s\")
node.destroy_node()
rclpy.shutdown()
" 2>&1 | grep -v RTPS
'
