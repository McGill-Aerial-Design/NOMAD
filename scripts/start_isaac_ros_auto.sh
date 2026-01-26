#!/bin/bash
# =============================================================================
# NOMAD Isaac ROS Automatic Startup Script
# =============================================================================
# This script starts Isaac ROS container and launches ZED + Nvblox + ROS-HTTP bridge
# in a fully automated way without requiring interactive terminal.
#
# Usage: 
#   ./start_isaac_ros_auto.sh          - Start all Isaac ROS services
#   ./start_isaac_ros_auto.sh stop     - Stop Isaac ROS container
#   ./start_isaac_ros_auto.sh status   - Check status
#   ./start_isaac_ros_auto.sh logs     - View logs
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_WS="${HOME}/ros2/isaac_ros_ws"
CONTAINER_NAME="nomad_isaac_ros"
IMAGE_NAME="isaac_ros_dev-aarch64:latest"
EDGE_CORE_HOST="172.17.0.1"  # Docker host from inside container
EDGE_CORE_PORT="8000"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_prerequisites() {
    log_info "Checking prerequisites..."
    
    # Check Docker
    if ! command -v docker &> /dev/null; then
        log_error "Docker is not installed"
        exit 1
    fi
    
    # Check image exists
    if ! docker image inspect "$IMAGE_NAME" &> /dev/null; then
        log_error "Isaac ROS image not found: $IMAGE_NAME"
        log_error "Please run the Isaac ROS build first"
        exit 1
    fi
    
    # Check workspace exists
    if [ ! -d "$ISAAC_WS" ]; then
        log_error "Isaac ROS workspace not found: $ISAAC_WS"
        exit 1
    fi
    
    # Check ZED camera
    if [ ! -e /dev/video0 ]; then
        log_warn "ZED camera not detected at /dev/video0"
    fi
    
    log_info "Prerequisites OK"
}

start_container() {
    log_info "Starting Isaac ROS container..."
    
    # Stop existing container if running
    if docker ps -q --filter "name=$CONTAINER_NAME" | grep -q .; then
        log_warn "Container already running, stopping first..."
        docker stop "$CONTAINER_NAME" 2>/dev/null || true
        docker rm "$CONTAINER_NAME" 2>/dev/null || true
    fi
    
    # Remove old container if exists
    docker rm "$CONTAINER_NAME" 2>/dev/null || true
    
    # Start container with all necessary mounts and devices
    docker run -d \
        --name "$CONTAINER_NAME" \
        --runtime nvidia \
        --privileged \
        --network host \
        --ipc host \
        -v "$ISAAC_WS:/workspaces/isaac_ros-dev" \
        -v /dev:/dev \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /tmp/argus_socket:/tmp/argus_socket \
        -v /etc/localtime:/etc/localtime:ro \
        -v /usr/local/zed:/usr/local/zed:ro \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e ROS_DOMAIN_ID=0 \
        -w /workspaces/isaac_ros-dev \
        "$IMAGE_NAME" \
        sleep infinity
    
    log_info "Container started: $CONTAINER_NAME"
    
    # Wait for container to be ready
    sleep 2
}

install_dependencies() {
    log_info "Installing ROS2 dependencies inside container..."
    
    docker exec "$CONTAINER_NAME" bash -c "
        apt update && apt install -y \
            ros-humble-zed-msgs \
            ros-humble-robot-localization \
            ros-humble-point-cloud-transport \
            ros-humble-tf2-ros \
            ros-humble-tf2-tools \
            python3-pip
        pip3 install requests
    " 2>&1 | tail -5
    
    log_info "Dependencies installed"
}

launch_zed_nvblox() {
    log_info "Launching ZED + Nvblox..."
    
    # Create a launch script inside container
    docker exec "$CONTAINER_NAME" bash -c "
        cat > /tmp/launch_zed_nvblox.sh << 'LAUNCH_SCRIPT'
#!/bin/bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2
LAUNCH_SCRIPT
        chmod +x /tmp/launch_zed_nvblox.sh
    "
    
    # Launch in background with nohup
    docker exec -d "$CONTAINER_NAME" bash -c "
        nohup /tmp/launch_zed_nvblox.sh > /tmp/zed_nvblox.log 2>&1 &
        echo \$! > /tmp/zed_nvblox.pid
    "
    
    log_info "ZED + Nvblox launched (logs at /tmp/zed_nvblox.log inside container)"
}

launch_ros_http_bridge() {
    log_info "Launching ROS-HTTP bridge..."
    
    # Copy the bridge script into container
    docker cp "$SCRIPT_DIR/../edge_core/ros_http_bridge.py" "$CONTAINER_NAME:/tmp/ros_http_bridge.py"
    
    # Create a launch script for the bridge
    docker exec "$CONTAINER_NAME" bash -c "
        cat > /tmp/launch_bridge.sh << 'BRIDGE_SCRIPT'
#!/bin/bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
# Wait for ROS nodes to be up
sleep 10
python3 /tmp/ros_http_bridge.py --host localhost --port 8000 --rate 30
BRIDGE_SCRIPT
        chmod +x /tmp/launch_bridge.sh
    "
    
    # Launch bridge in background
    docker exec -d "$CONTAINER_NAME" bash -c "
        nohup /tmp/launch_bridge.sh > /tmp/ros_bridge.log 2>&1 &
        echo \$! > /tmp/ros_bridge.pid
    "
    
    log_info "ROS-HTTP bridge launched (logs at /tmp/ros_bridge.log inside container)"
}

stop_services() {
    log_info "Stopping Isaac ROS services..."
    
    if docker ps -q --filter "name=$CONTAINER_NAME" | grep -q .; then
        docker stop "$CONTAINER_NAME" 2>/dev/null || true
        docker rm "$CONTAINER_NAME" 2>/dev/null || true
        log_info "Container stopped and removed"
    else
        log_warn "Container not running"
    fi
}

show_status() {
    echo "=== Isaac ROS Container Status ==="
    
    if docker ps -q --filter "name=$CONTAINER_NAME" | grep -q .; then
        echo -e "${GREEN}Container: Running${NC}"
        
        # Check processes inside
        echo ""
        echo "Processes inside container:"
        docker exec "$CONTAINER_NAME" ps aux | grep -E "ros2|python3|nvblox|zed" | head -10
        
        # Check ROS topics
        echo ""
        echo "ROS2 Topics (sample):"
        docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash && ros2 topic list 2>/dev/null | head -15" || echo "(ROS not fully initialized)"
        
        # Check logs
        echo ""
        echo "Recent ZED+Nvblox logs:"
        docker exec "$CONTAINER_NAME" tail -5 /tmp/zed_nvblox.log 2>/dev/null || echo "(No logs yet)"
        
        echo ""
        echo "Recent ROS-HTTP bridge logs:"
        docker exec "$CONTAINER_NAME" tail -5 /tmp/ros_bridge.log 2>/dev/null || echo "(No logs yet)"
    else
        echo -e "${RED}Container: Not running${NC}"
    fi
}

show_logs() {
    log_type="${1:-all}"
    
    if ! docker ps -q --filter "name=$CONTAINER_NAME" | grep -q .; then
        log_error "Container not running"
        exit 1
    fi
    
    case "$log_type" in
        zed|nvblox)
            docker exec "$CONTAINER_NAME" tail -f /tmp/zed_nvblox.log
            ;;
        bridge)
            docker exec "$CONTAINER_NAME" tail -f /tmp/ros_bridge.log
            ;;
        *)
            echo "=== ZED + Nvblox Logs ==="
            docker exec "$CONTAINER_NAME" tail -20 /tmp/zed_nvblox.log 2>/dev/null || echo "(No logs)"
            echo ""
            echo "=== ROS-HTTP Bridge Logs ==="
            docker exec "$CONTAINER_NAME" tail -20 /tmp/ros_bridge.log 2>/dev/null || echo "(No logs)"
            ;;
    esac
}

# Main entry point
case "${1:-start}" in
    start)
        check_prerequisites
        start_container
        install_dependencies
        launch_zed_nvblox
        # Wait for ZED to initialize before starting bridge
        log_info "Waiting for ZED initialization (20s)..."
        sleep 20
        launch_ros_http_bridge
        log_info "Isaac ROS startup complete!"
        show_status
        ;;
    stop)
        stop_services
        ;;
    restart)
        stop_services
        sleep 2
        $0 start
        ;;
    status)
        show_status
        ;;
    logs)
        show_logs "${2:-all}"
        ;;
    shell)
        log_info "Opening shell in container..."
        docker exec -it "$CONTAINER_NAME" bash
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status|logs|shell}"
        exit 1
        ;;
esac
