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
ISAAC_WS="${HOME}/workspaces/isaac_ros-dev"
CONTAINER_NAME="nomad_isaac_ros"
IMAGE_NAME="dustynv/ros:humble-ros-base-l4t-r36.2.0"
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
    
    # Check ZED camera (informational only)
    if [ ! -e /dev/video0 ]; then
        log_warn "ZED camera not detected at /dev/video0"
    fi
    
    log_info "Prerequisites OK"
}

start_container() {
    log_info "Starting Isaac ROS container..."
    
    # Check if container exists
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        # Container exists - check if it's running
        if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            log_info "Container already running"
            return 0
        else
            # Container exists but stopped - just start it
            log_info "Container exists but stopped, starting it..."
            docker start "$CONTAINER_NAME"
            log_info "Container started: $CONTAINER_NAME"
            sleep 2
            return 0
        fi
    fi
    
    # Container doesn't exist - create it
    log_info "Creating new container..."
    
    # Start container with all necessary mounts and devices
    # Note: We do NOT mount /usr/local/zed from host - the ZED SDK
    # should be installed INSIDE the container image
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
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e ROS_DOMAIN_ID=0 \
        -e LD_LIBRARY_PATH=/usr/local/zed/lib:/opt/ros/humble/lib \
        -w /workspaces/isaac_ros-dev \
        "$IMAGE_NAME" \
        sleep infinity
    
    log_info "Container created: $CONTAINER_NAME"
    
    # Wait for container to be ready
    sleep 2
}

install_dependencies() {
    log_info "Checking dependencies inside container..."
    
    # Check if dependencies are already installed by testing for a key package
    if docker exec "$CONTAINER_NAME" bash -c "dpkg -l | grep -q ros-humble-zed-msgs" 2>/dev/null; then
        log_info "Dependencies already installed, skipping..."
        return 0
    fi
    
    log_info "Installing ROS2 dependencies inside container..."
    
    # Remove problematic yarn repo first, then install packages
    docker exec "$CONTAINER_NAME" bash -c "
        rm -f /etc/apt/sources.list.d/yarn.list 2>/dev/null
        apt-get update -qq
        apt-get install -y --no-install-recommends \
            ros-humble-zed-msgs \
            ros-humble-nmea-msgs \
            ros-humble-robot-localization \
            ros-humble-point-cloud-transport \
            ros-humble-point-cloud-transport-plugins \
            ros-humble-tf2-ros \
            ros-humble-tf2-tools \
            ros-humble-cob-srvs \
            ros-humble-cv-bridge \
            liburdfdom-dev \
            python3-pip 2>/dev/null
        # Update library cache for newly installed packages
        echo /opt/ros/humble/lib >> /etc/ld.so.conf.d/ros.conf
        ldconfig
        
        # Check if packages need to be installed/updated
        if ! python3 -c 'import requests' 2>/dev/null; then
            pip3 install requests opencv-python-headless 2>/dev/null
        fi
    " 2>&1 | tail -3
    
    # Run rosdep to install remaining dependencies
    log_info "Running rosdep for ZED wrapper..."
    docker exec "$CONTAINER_NAME" bash -c "
        source /opt/ros/humble/install/setup.bash
        cd /workspaces/isaac_ros-dev/src/zed-ros2-wrapper
        rosdep install --from-paths . --ignore-src -r -y 2>/dev/null
    " 2>&1 | tail -5
    
    # Rebuild ZED packages to pick up the new dependencies  
    log_info "Rebuilding ZED packages..."
    docker exec "$CONTAINER_NAME" bash -c "
        source /opt/ros/humble/install/setup.bash
        cd /workspaces/isaac_ros-dev
        colcon build --packages-select zed_components zed_wrapper --symlink-install 2>/dev/null
    " 2>&1 | tail -5
    
    log_info "Dependencies installed and ZED packages rebuilt"
}

check_and_build_nvblox() {
    log_info "Checking for Nvblox packages..."
    
    # Check if nvblox is already built
    if docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/install/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash && ros2 pkg list 2>/dev/null | grep -q nvblox"; then
        log_info "Nvblox packages already built"
        return 0
    fi
    
    log_warn "Nvblox packages not found - checking source..."
    
    # Check if nvblox source exists
    if ! docker exec "$CONTAINER_NAME" test -d /workspaces/isaac_ros-dev/src/isaac_ros_nvblox; then
        log_warn "Nvblox source not found. Please clone it first:"
        log_warn "  cd ~/workspaces/isaac_ros-dev/src"
        log_warn "  git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git"
        log_warn "  git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git"
        log_warn "Then rebuild: colcon build --symlink-install"
        log_warn ""
        log_warn "Falling back to ZED-only mode (no Nvblox)..."
        return 1
    fi
    
    log_info "Building Nvblox (this may take 10-30 minutes)..."
    docker exec "$CONTAINER_NAME" bash -c "
        source /opt/ros/humble/install/setup.bash
        cd /workspaces/isaac_ros-dev
        colcon build --symlink-install --packages-up-to nvblox_examples_bringup
    " 2>&1 | tail -20
    
    return 0
}

launch_zed_nvblox() {
    log_info "Launching ZED + Nvblox..."
    
    # Check if nvblox is available
    if ! docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/install/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash && ros2 pkg list 2>/dev/null | grep -q nvblox"; then
        log_warn "Nvblox not available - launching ZED wrapper only"
        launch_zed_only
        return
    fi
    
    # Note: nvblox_examples_bringup only supports zed2 and zedx cameras
    # For ZED 2i, we need to use the ZED-only launch with camera_model:=zed2i
    # Since the camera is detected as zed2i, we use ZED-only mode
    log_info "Using ZED-only launch for ZED 2i camera (nvblox requires zed2/zedx)"
    launch_zed_only
}

launch_zed_only() {
    log_info "Launching ZED wrapper only (camera_model:=zed2i)..."
    
    # Create a ZED-only launch script
    docker exec "$CONTAINER_NAME" bash -c "
        cat > /tmp/launch_zed_only.sh << 'LAUNCH_SCRIPT'
#!/bin/bash
source /opt/ros/humble/install/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/local/zed/lib:/workspaces/isaac_ros-dev/install/zed_components/lib:\$LD_LIBRARY_PATH
# Use zed2i for ZED 2i camera
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
LAUNCH_SCRIPT
        chmod +x /tmp/launch_zed_only.sh
    "
    
    docker exec -d "$CONTAINER_NAME" bash -c "
        bash /tmp/launch_zed_only.sh > /tmp/zed_nvblox.log 2>&1 &
        echo \$! > /tmp/zed_nvblox.pid
    "
    
    log_info "ZED wrapper launched (logs at /tmp/zed_nvblox.log inside container)"
    log_warn "Note: First-time launch may take several minutes for AI model optimization"
}

launch_ros_http_bridge() {
    log_info "Launching ROS-HTTP bridge..."
    
    # Copy the bridge script into container
    docker cp "$SCRIPT_DIR/../edge_core/ros_http_bridge.py" "$CONTAINER_NAME:/tmp/ros_http_bridge.py"
    
    # Create a launch script for the bridge
    docker exec "$CONTAINER_NAME" bash -c "
        cat > /tmp/launch_bridge.sh << 'BRIDGE_SCRIPT'
#!/bin/bash
source /opt/ros/humble/install/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
# Wait for ROS nodes to be up
sleep 5
# Use ZED odom topic by default
python3 /tmp/ros_http_bridge.py --host localhost --port 8000 --rate 30 --vio-topic /zed/zed_node/odom
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

# Video bridge is now managed by Edge Core's VideoStreamManager API
# The edge_core will auto-start a default stream when the container is ready
# Use the API endpoints to dynamically switch streams:
#   GET  /api/video/topics          - List available ROS image topics
#   POST /api/video/source?topic=X  - Switch to a specific topic
#   GET  /api/video/streams         - List active streams

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
        docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/install/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash && ros2 topic list 2>/dev/null | head -15" || echo "(ROS not fully initialized)"
        
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
        video)
            docker exec "$CONTAINER_NAME" tail -f /tmp/video_bridge.log
            ;;
        *)
            echo "=== ZED + Nvblox Logs ==="
            docker exec "$CONTAINER_NAME" tail -20 /tmp/zed_nvblox.log 2>/dev/null || echo "(No logs)"
            echo ""
            echo "=== ROS-HTTP Bridge Logs ==="
            docker exec "$CONTAINER_NAME" tail -20 /tmp/ros_bridge.log 2>/dev/null || echo "(No logs)"
            echo ""
            echo "=== Video Bridge Logs ==="
            docker exec "$CONTAINER_NAME" tail -20 /tmp/video_bridge.log 2>/dev/null || echo "(No logs)"
            ;;
    esac
}

# Main entry point
case "${1:-start}" in
    start)
        check_prerequisites
        start_container
        install_dependencies
        # Don't exit on nvblox check failure - we fall back to ZED-only
        check_and_build_nvblox || true
        launch_zed_nvblox
        
        # Wait for ZED to initialize - check for topics instead of fixed wait
        log_info "Waiting for ZED to publish topics..."
        WAIT_START=$(date +%s)
        MAX_WAIT=30
        ZED_READY=false
        
        while [ $(($(date +%s) - WAIT_START)) -lt $MAX_WAIT ]; do
            # Check if ZED image topics are being published
            if docker exec "$CONTAINER_NAME" bash -c "
                source /opt/ros/humble/install/setup.bash && \
                timeout 2 ros2 topic list 2>/dev/null | grep -q '/zed/zed_node/.*image'
            " 2>/dev/null; then
                ZED_READY=true
                ELAPSED=$(($(date +%s) - WAIT_START))
                log_info "ZED ready after ${ELAPSED}s"
                break
            fi
            sleep 1
        done
        
        if [ "$ZED_READY" = false ]; then
            log_warn "ZED topics not detected after ${MAX_WAIT}s, continuing anyway..."
        fi
        
        launch_ros_http_bridge
        
        # Note: Video streaming is now handled by Edge Core's VideoStreamManager
        # It will auto-start a default stream when topics become available
        # Use the API to switch streams: POST /api/video/source?topic=...
        log_info "Video streaming managed by Edge Core API (auto-starts when ready)"
        
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
