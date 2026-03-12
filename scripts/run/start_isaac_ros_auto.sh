#!/bin/bash
# =============================================================================
# NOMAD Isaac ROS Automatic Startup Script
# =============================================================================
# Starts the Isaac ROS container and launches ZED + nvblox + ROS-HTTP bridge.
#
# The container uses the official Isaac ROS dev image (isaac_ros_dev-aarch64)
# built via isaac_ros_common/scripts/build_image_layers.sh. See
# docs/ISAAC_ROS_NVBLOX_SETUP.md for the one-time build steps.
#
# Usage:
#   ./start_isaac_ros_auto.sh          - Start all Isaac ROS services
#   ./start_isaac_ros_auto.sh stop     - Stop Isaac ROS container
#   ./start_isaac_ros_auto.sh status   - Check status
#   ./start_isaac_ros_auto.sh logs     - View logs
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ISAAC_WS="${HOME}/workspaces/isaac_ros-dev"
CONTAINER_NAME="nomad_isaac_ros"
# Prefer the NOMAD-layer image (built via docker compose build) if available;
# fall back to the official Isaac ROS base image.
if docker image inspect nomad-isaac-ros:latest &> /dev/null 2>&1; then
    IMAGE_NAME="nomad-isaac-ros:latest"
else
    IMAGE_NAME="isaac_ros_dev-aarch64"
fi

# ZED ROS2 wrapper branch matching ZED SDK 4.1
ZED_WRAPPER_BRANCH="humble-v4.1.4"

# Standard ROS2 setup for the official Isaac ROS image
ROS_SETUP="source /opt/ros/humble/setup.bash 2>/dev/null"
WS_SETUP="source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null || true"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# =========================================================================
# Prerequisites
# =========================================================================
check_prerequisites() {
    log_info "Checking prerequisites..."

    if ! command -v docker &> /dev/null; then
        log_error "Docker is not installed"
        exit 1
    fi

    if ! docker image inspect "$IMAGE_NAME" &> /dev/null; then
        log_error "Isaac ROS image not found: $IMAGE_NAME"
        log_error "Build it first with:"
        log_error "  cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts"
        log_error "  ./build_image_layers.sh --image_key ros2_humble"
        log_error "See docs/ISAAC_ROS_NVBLOX_SETUP.md for full instructions."
        exit 1
    fi

    if [ ! -d "$ISAAC_WS" ]; then
        log_error "Isaac ROS workspace not found: $ISAAC_WS"
        exit 1
    fi

    if [ ! -e /dev/video0 ]; then
        log_warn "ZED camera not detected at /dev/video0"
    fi

    log_info "Prerequisites OK"
}

# =========================================================================
# Container lifecycle
# =========================================================================
start_container() {
    log_info "Starting Isaac ROS container..."

    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        log_info "Container already running"
        return 0
    fi

    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        log_info "Container exists but stopped, starting..."
        docker start "$CONTAINER_NAME"
        sleep 2
        return 0
    fi

    log_info "Creating new container from $IMAGE_NAME ..."
    docker run -d \
        --name "$CONTAINER_NAME" \
        --runtime nvidia \
        --privileged \
        --network host \
        --ipc host \
        -v "$ISAAC_WS:/workspaces/isaac_ros-dev" \
        -v /dev:/dev \
        -v /run/udev:/run/udev:ro \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /tmp/argus_socket:/tmp/argus_socket \
        -v /etc/localtime:/etc/localtime:ro \
        -v "$REPO_ROOT/config:/workspaces/isaac_ros-dev/config:ro" \
        -v "$REPO_ROOT/edge_core:/workspaces/isaac_ros-dev/edge_core:ro" \
        -e DISPLAY="${DISPLAY}" \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e EGL_PLATFORM=device \
        -e ROS_DOMAIN_ID=0 \
        -e LD_LIBRARY_PATH=/usr/local/zed/lib:/opt/ros/humble/lib \
        -e CMAKE_PREFIX_PATH=/usr/local/zed \
        -w /workspaces/isaac_ros-dev \
        "$IMAGE_NAME" \
        sleep infinity

    log_info "Container created: $CONTAINER_NAME"
    sleep 2
}

# =========================================================================
# ZED ROS2 wrapper (cloned on the host, visible via volume mount)
# =========================================================================
clone_zed_wrapper() {
    ZED_SRC="${ISAAC_WS}/src/zed-ros2-wrapper"
    if [ -d "$ZED_SRC" ] && [ -d "$ZED_SRC/zed_wrapper" ]; then
        log_info "ZED ROS2 wrapper already cloned"
        return 0
    fi

    log_info "Cloning ZED ROS2 wrapper (branch $ZED_WRAPPER_BRANCH)..."
    rm -rf "$ZED_SRC"
    git clone --branch "$ZED_WRAPPER_BRANCH" --depth 1 \
        https://github.com/stereolabs/zed-ros2-wrapper.git "$ZED_SRC"
    cd "$ZED_SRC" && git submodule update --init --recursive && cd -
    log_info "ZED ROS2 wrapper cloned"
}

# =========================================================================
# Install runtime dependencies inside the container
# Per the guide these get wiped on every container restart so we re-install.
# =========================================================================
install_dependencies() {
    log_info "Installing runtime dependencies inside container..."

    # --- ZED SDK ---
    if ! docker exec "$CONTAINER_NAME" test -f /usr/local/zed/lib/libsl_zed.so 2>/dev/null; then
        log_info "Installing ZED SDK 4.2 inside container..."
        docker exec "$CONTAINER_NAME" bash -c "
            apt-get update -qq
            apt-get install -y --no-install-recommends zstd wget
            wget -q 'https://download.stereolabs.com/zedsdk/4.2/l4t36.4/jetsons' -O /tmp/zed_installer.run
            chmod +x /tmp/zed_installer.run
            /tmp/zed_installer.run -- silent skip_od_module
            rm -f /tmp/zed_installer.run
            ldconfig
        " 2>&1 | tail -5

        if docker exec "$CONTAINER_NAME" test -f /usr/local/zed/lib/libsl_zed.so 2>/dev/null; then
            log_info "ZED SDK installed successfully"
        else
            log_error "ZED SDK installation failed"
            return 1
        fi
    else
        log_info "ZED SDK already installed"
    fi

    # --- apt deps (per the guide, needed after every container restart) ---
    # Always fix pip cmake (it overrides system cmake and breaks colcon)
    docker exec "$CONTAINER_NAME" bash -c "pip3 uninstall -y cmake 2>/dev/null || true" 2>/dev/null
    # Always fix numpy _core/include symlink (NITROS packages expect numpy 2.x paths)
    docker exec "$CONTAINER_NAME" bash -c "
        if [ ! -e /usr/local/lib/python3.10/dist-packages/numpy/_core/include ]; then
            ln -sf /usr/local/lib/python3.10/dist-packages/numpy/core/include \
                   /usr/local/lib/python3.10/dist-packages/numpy/_core/include 2>/dev/null || true
        fi
    " 2>/dev/null

    # Check ROS deps and GStreamer deps separately so GStreamer is never skipped
    local ros_deps_ok=false
    local gst_deps_ok=false
    if docker exec "$CONTAINER_NAME" dpkg -l ros-humble-zed-msgs 2>/dev/null | grep -q '^ii'; then
        ros_deps_ok=true
        log_info "ROS dependencies already installed"
    fi
    if docker exec "$CONTAINER_NAME" test -f /usr/lib/aarch64-linux-gnu/girepository-1.0/Gst-1.0.typelib 2>/dev/null; then
        gst_deps_ok=true
        log_info "GStreamer dependencies already installed"
    fi

    if [ "$ros_deps_ok" = true ] && [ "$gst_deps_ok" = true ]; then
        return 0
    fi

    log_info "Installing missing dependencies..."
    docker exec "$CONTAINER_NAME" bash -c "
        apt-get update -qq
        apt-get install -y --no-install-recommends \
            ros-humble-zed-msgs \
            ros-humble-nmea-msgs \
            ros-humble-robot-localization \
            ros-humble-point-cloud-transport \
            ros-humble-tf2-ros \
            ros-humble-tf2-tools \
            ros-humble-cv-bridge \
            ros-humble-isaac-ros-managed-nitros \
            ros-humble-isaac-ros-nitros \
            ros-humble-isaac-ros-nitros-image-type \
            ros-humble-isaac-ros-nitros-camera-info-type \
            ros-humble-isaac-ros-nitros-point-cloud-type \
            python3-pip \
            gir1.2-gstreamer-1.0 \
            gir1.2-gst-plugins-base-1.0 \
            gstreamer1.0-plugins-good \
            gstreamer1.0-plugins-bad \
            gstreamer1.0-rtsp \
            gstreamer1.0-x
        pip3 install --no-cache-dir requests transforms3d 'numpy<2' 2>/dev/null || true
        ldconfig
    " 2>&1 | tail -5

    log_info "Dependencies installed"
}

# =========================================================================
# Build ZED + nvblox packages (colcon build artifacts persist on host)
# =========================================================================
build_packages() {
    log_info "Building ROS2 packages..."

    # --- ZED wrapper ---
    if docker exec "$CONTAINER_NAME" bash -c "$ROS_SETUP; $WS_SETUP; ros2 pkg list 2>/dev/null | grep -q zed_wrapper"; then
        log_info "ZED wrapper already built"
    else
        log_info "Building ZED wrapper (first time may take several minutes)..."
        docker exec "$CONTAINER_NAME" bash -c "
            $ROS_SETUP
            cd /workspaces/isaac_ros-dev
            colcon build --packages-up-to zed_wrapper --symlink-install --cmake-args -Wno-dev 2>&1
        " 2>&1 | tail -10
    fi

    # --- nvblox ---
    if docker exec "$CONTAINER_NAME" bash -c "$ROS_SETUP; $WS_SETUP; ros2 pkg list 2>/dev/null | grep -q nvblox_ros"; then
        log_info "nvblox already built"
    else
        if docker exec "$CONTAINER_NAME" test -d /workspaces/isaac_ros-dev/src/isaac_ros_nvblox; then
            log_info "Building nvblox (first time may take 10-30 minutes)..."
            docker exec "$CONTAINER_NAME" bash -c "
                $ROS_SETUP
                cd /workspaces/isaac_ros-dev
                colcon build --symlink-install --packages-up-to nvblox_examples_bringup --cmake-args -Wno-dev 2>&1
            " 2>&1 | tail -15
        else
            log_warn "nvblox source not found at src/isaac_ros_nvblox -- skipping"
            log_warn "Clone it with: cd ~/workspaces/isaac_ros-dev/src && git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git"
        fi
    fi
}

# =========================================================================
# Launch ZED + nvblox
# =========================================================================
launch_zed_nvblox() {
    # Check if nvblox is available
    if docker exec "$CONTAINER_NAME" bash -c "$ROS_SETUP; $WS_SETUP; ros2 pkg list 2>/dev/null | grep -q nvblox_examples_bringup"; then
        log_info "Launching ZED + nvblox (camera:=zed2)..."

        # Write launch script via stdin to avoid quoting issues with $() and nested "
        docker exec -i "$CONTAINER_NAME" tee /tmp/launch_zed_nvblox.sh > /dev/null << 'LAUNCH_SCRIPT'
#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH=/usr/local/zed/lib:$LD_LIBRARY_PATH
# Patch ZED publish resolution to native 720p
sed -i 's/pub_downscale_factor: 2\.0/pub_downscale_factor: 1.0/' \
    /workspaces/isaac_ros-dev/install/zed_wrapper/share/zed_wrapper/config/common.yaml 2>/dev/null
# Overlay NOMAD nvblox config onto installed base config
# voxel_size=0.15, ESDF 3D, 8m clearing radius
NOMAD_CFG=/workspaces/isaac_ros-dev/config/nvblox_performance.yaml
NVBLOX_BASE=$(python3 -c "from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory('nvblox_examples_bringup'))" 2>/dev/null)/config/nvblox/nvblox_base.yaml
if [ -f "$NOMAD_CFG" ] && [ -f "$NVBLOX_BASE" ]; then
    echo "Applying NOMAD nvblox config (voxel_size=0.15, esdf=3d, 8m radius)"
    cp "$NOMAD_CFG" "$NVBLOX_BASE"
else
    echo "NOMAD config or nvblox base not found, using defaults"
fi
# Use custom NOMAD launch file with YOLO object detection enabled
NOMAD_LAUNCH=/workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py
if [ -f "$NOMAD_LAUNCH" ]; then
    echo "Launching with NOMAD custom OD launch file"
    ros2 launch "$NOMAD_LAUNCH"
else
    echo "NOMAD launch file not found, falling back to stock launch (no OD)"
    ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2
fi
LAUNCH_SCRIPT
        docker exec "$CONTAINER_NAME" chmod +x /tmp/launch_zed_nvblox.sh

        docker exec -d "$CONTAINER_NAME" bash -c \
            "bash /tmp/launch_zed_nvblox.sh > /tmp/zed_nvblox.log 2>&1 & echo \$! > /tmp/zed_nvblox.pid"

        log_info "ZED + nvblox + OD launched (logs: /tmp/zed_nvblox.log inside container)"
    else
        log_warn "nvblox not available -- launching ZED wrapper only"
        launch_zed_only
    fi
}

launch_zed_only() {
    log_info "Launching ZED wrapper only (camera_model:=zed2i)..."

    docker exec -i "$CONTAINER_NAME" tee /tmp/launch_zed_only.sh > /dev/null << 'LAUNCH_SCRIPT'
#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH=/usr/local/zed/lib:$LD_LIBRARY_PATH
# Patch ZED publish resolution to native 720p
sed -i 's/pub_downscale_factor: 2\.0/pub_downscale_factor: 1.0/' \
    /workspaces/isaac_ros-dev/install/zed_wrapper/share/zed_wrapper/config/common.yaml 2>/dev/null
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
LAUNCH_SCRIPT
    docker exec "$CONTAINER_NAME" chmod +x /tmp/launch_zed_only.sh

    docker exec -d "$CONTAINER_NAME" bash -c \
        "bash /tmp/launch_zed_only.sh > /tmp/zed_nvblox.log 2>&1 & echo \$! > /tmp/zed_nvblox.pid"

    log_info "ZED wrapper launched (logs: /tmp/zed_nvblox.log inside container)"
}

# =========================================================================
# Video Bridge (subscribes to ROS image topics, pushes H264 to MediaMTX)
# =========================================================================
launch_video_bridge() {
    log_info "Launching video bridge..."

    # Kill any existing video bridge to prevent duplicates
    docker exec "$CONTAINER_NAME" bash -c 'pkill -f simple_video_bridge.py 2>/dev/null || true; sleep 1'

    docker exec -i "$CONTAINER_NAME" tee /tmp/launch_video_bridge.sh > /dev/null << 'VIDEO_SCRIPT'
#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH=/usr/local/zed/lib:$LD_LIBRARY_PATH
sleep 8  # Wait for ZED image topics to be publishing
python3 /workspaces/isaac_ros-dev/edge_core/ros/simple_video_bridge.py \
    --source-topic /zed/zed_node/rgb/image_rect_color \
    --width 1280 --height 720 --fps 30 --bitrate 2000 \
    --http-port 9200
VIDEO_SCRIPT
    docker exec "$CONTAINER_NAME" chmod +x /tmp/launch_video_bridge.sh

    docker exec -d "$CONTAINER_NAME" bash -c \
        "nohup /tmp/launch_video_bridge.sh > /tmp/video_bridge.log 2>&1 & echo \$! > /tmp/video_bridge.pid"

    log_info "Video bridge launched (logs: /tmp/video_bridge.log inside container)"
}

# =========================================================================
# ROS-HTTP Bridge (relays ROS2 data to Edge Core API)
# =========================================================================
launch_ros_http_bridge() {
    log_info "Launching ROS-HTTP bridge..."

    # Bridge script is available via volume mount at /workspaces/isaac_ros-dev/edge_core/
    # Kill any existing bridge processes to prevent duplicates
    docker exec "$CONTAINER_NAME" bash -c 'pkill -f ros_http_bridge.py 2>/dev/null || true; sleep 1'
    
    docker exec -i "$CONTAINER_NAME" tee /tmp/launch_bridge.sh > /dev/null << 'BRIDGE_SCRIPT'
#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
sleep 5
python3 /workspaces/isaac_ros-dev/edge_core/ros_http_bridge.py --host localhost --port 8000 --rate 10 --vio-topic /zed/zed_node/odom
BRIDGE_SCRIPT
    docker exec "$CONTAINER_NAME" chmod +x /tmp/launch_bridge.sh

    docker exec -d "$CONTAINER_NAME" bash -c \
        "nohup /tmp/launch_bridge.sh > /tmp/ros_bridge.log 2>&1 & echo \$! > /tmp/ros_bridge.pid"

    log_info "ROS-HTTP bridge launched (logs: /tmp/ros_bridge.log inside container)"
}

# =========================================================================
# Stop / Status / Logs
# =========================================================================
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
        echo ""
        echo "Processes inside container:"
        docker exec "$CONTAINER_NAME" ps aux 2>/dev/null | grep -E "ros2|python3|nvblox|zed" | head -10
        echo ""
        echo "ROS2 Topics (sample):"
        docker exec "$CONTAINER_NAME" bash -c "$ROS_SETUP; $WS_SETUP; ros2 topic list 2>/dev/null | head -15" || echo "(ROS not ready)"
        echo ""
        echo "Recent logs:"
        docker exec "$CONTAINER_NAME" tail -5 /tmp/zed_nvblox.log 2>/dev/null || echo "(No ZED logs)"
    else
        echo -e "${RED}Container: Not running${NC}"
    fi
}

show_logs() {
    if ! docker ps -q --filter "name=$CONTAINER_NAME" | grep -q .; then
        log_error "Container not running"
        exit 1
    fi
    case "${1:-all}" in
        zed|nvblox) docker exec "$CONTAINER_NAME" tail -f /tmp/zed_nvblox.log ;;
        bridge)     docker exec "$CONTAINER_NAME" tail -f /tmp/ros_bridge.log ;;
        video)      docker exec "$CONTAINER_NAME" tail -f /tmp/video_bridge.log ;;
        *)
            echo "=== ZED + nvblox ==="
            docker exec "$CONTAINER_NAME" tail -20 /tmp/zed_nvblox.log 2>/dev/null || echo "(No logs)"
            echo ""
            echo "=== ROS-HTTP Bridge ==="
            docker exec "$CONTAINER_NAME" tail -20 /tmp/ros_bridge.log 2>/dev/null || echo "(No logs)"
            ;;
    esac
}

# =========================================================================
# Main
# =========================================================================
case "${1:-start}" in
    start)
        check_prerequisites
        clone_zed_wrapper
        start_container
        install_dependencies
        build_packages
        launch_zed_nvblox

        # Wait for ZED topics
        log_info "Waiting for ZED topics..."
        WAIT_START=$(date +%s)
        ZED_READY=false
        while [ $(($(date +%s) - WAIT_START)) -lt 30 ]; do
            if docker exec "$CONTAINER_NAME" bash -c \
                "$ROS_SETUP; timeout 2 ros2 topic list 2>/dev/null | grep -q '/zed/zed_node/.*image'" 2>/dev/null; then
                ZED_READY=true
                log_info "ZED ready after $(($(date +%s) - WAIT_START))s"
                break
            fi
            sleep 1
        done
        [ "$ZED_READY" = false ] && log_warn "ZED topics not detected after 30s, continuing..."

        launch_ros_http_bridge
        launch_video_bridge
        log_info "Isaac ROS startup complete!"
        show_status
        ;;
    stop)    stop_services ;;
    restart) stop_services; sleep 2; exec "$0" start ;;
    status)  show_status ;;
    logs)    show_logs "${2:-all}" ;;
    shell)   docker exec -it "$CONTAINER_NAME" bash ;;
    *)       echo "Usage: $0 {start|stop|restart|status|logs|shell}"; exit 1 ;;
esac
