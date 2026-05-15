#!/bin/bash
# Sample multiple markers to see if any have actual content
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null

  python3 -c "
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import time

rclpy.init()
node = Node(\"spy2\")
received = []
def cb(msg):
    received.append((msg.type, len(msg.points), len(msg.colors)))

sub = node.create_subscription(Marker, \"/nvblox_node/color_layer_marker\", cb, 10)
deadline = time.time() + 8
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

print(f\"Total markers received: {len(received)}\")
non_empty = [(t,p,c) for t,p,c in received if p > 0]
print(f\"Non-empty markers: {len(non_empty)}\")
if non_empty:
    print(f\"First non-empty: {non_empty[0]}\")
else:
    print(f\"All empty. Sample: {received[:5]}\")
node.destroy_node()
rclpy.shutdown()
" 2>&1 | grep -v RTPS
'
