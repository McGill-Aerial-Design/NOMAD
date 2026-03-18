#!/bin/bash
# Restart ros_http_bridge with fresh connection to live ZED + nvblox
CONTAINER_NAME="nomad_isaac_ros"

echo "=== Killing old bridge ==="
docker exec "$CONTAINER_NAME" bash -c 'pkill -9 -f ros_http_bridge.py 2>/dev/null; sleep 1; echo done'

echo "=== Starting new bridge ==="
docker exec -i "$CONTAINER_NAME" tee /tmp/launch_bridge2.sh > /dev/null << 'BRIDGE_SCRIPT'
#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH=/usr/local/zed/lib:$LD_LIBRARY_PATH
echo "Starting ros_http_bridge..."
python3 /workspaces/isaac_ros-dev/edge_core/ros_http_bridge.py \
  --host localhost \
  --port 8000 \
  --rate 30 \
  --vio-topic /zed/zed_node/odom
BRIDGE_SCRIPT

docker exec "$CONTAINER_NAME" chmod +x /tmp/launch_bridge2.sh

docker exec -d "$CONTAINER_NAME" bash -c \
  "nohup bash /tmp/launch_bridge2.sh > /tmp/ros_bridge2.log 2>&1 & echo \$! > /tmp/ros_bridge2.pid"

echo "=== Waiting 8s for bridge to connect ==="
sleep 8

echo "=== Bridge log ==="
docker exec "$CONTAINER_NAME" tail -20 /tmp/ros_bridge2.log 2>/dev/null

echo ""
echo "=== Bridge stats from Edge Core ==="
curl -s http://localhost:8000/api/vio/status 2>/dev/null
