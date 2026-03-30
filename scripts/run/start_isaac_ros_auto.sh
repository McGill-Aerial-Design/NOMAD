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
ROS_SETUP="source /opt/ros/humble/install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null"
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

    # =========================================================================
    # Jetson Power Mode Check (NV-011)
    # =========================================================================
    log_info "Checking Jetson power mode..."
    if [ -e /sys/devices/virtual/thermal/cooling_device0/cur_state ]; then
        POWER_MODE=$(cat /sys/devices/virtual/thermal/cooling_device0/cur_state 2>/dev/null || echo "unknown")
        case "$POWER_MODE" in
            15|16|17|18|19|20)  # Assuming states above 15 are MAXN or high performance
                log_info "Power mode: High performance (state $POWER_MODE) - OK"
                ;;
            0|1|2|3|4|5)
                log_warn "Power mode: Very low (state $POWER_MODE). nvblox may throttle."
                log_warn "Run 'sudo nvpmodel -m 2' for 25W mode or 'sudo jetson_clocks' for max performance"
                ;;
            *)
                log_warn "Power mode: Unknown state $POWER_MODE"
                ;;
        esac
    else
        log_warn "Cannot determine power mode (nvpmodel/jetson_clocks not available)"
        log_warn "For best results, run: sudo jetson_clocks"
    fi

    # =========================================================================
    # Shared Memory Availability Check (NV-012)
    # =========================================================================
    log_info "Checking shared memory availability..."
    SHM_AVAIL=$(df /dev/shm 2>/dev/null | tail -1 | awk '{print $4}')
    if [ -n "$SHM_AVAIL" ] && [ "$SHM_AVAIL" -gt 0 ]; then
        SHM_AVAIL_MiB=$((SHM_AVAIL / 1024))
        if [ "$SHM_AVAIL_MiB" -lt 512 ]; then
            log_warn "Shared memory low: ${SHM_AVAIL_MiB}MiB available (recommend 1GB)"
            log_warn "Clean stale FastRTPS lockfiles: rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*"
        else
            log_info "Shared memory OK: ${SHM_AVAIL_MiB}MiB available"
        fi
    else
        log_warn "Cannot determine shared memory availability"
    fi

    # =========================================================================
    # Available Memory Check (NV-013)
    # =========================================================================
    log_info "Checking available system memory..."
    MEM_AVAIL=$(free -m 2>/dev/null | grep '^Mem:' | awk '{print $7}')
    if [ -n "$MEM_AVAIL" ]; then
        if [ "$MEM_AVAIL" -lt 2048 ]; then
            log_warn "Available memory: ${MEM_AVAIL}MiB (recommend +2GB for host)"
            log_warn "Background processes may be consuming memory. Run: free -h && ps aux --sort=-%mem | head -10"
        else
            log_info "Available memory: ${MEM_AVAIL}MiB - OK"
        fi
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
        --memory 4g \
        --cpus 6 \
        --shm-size 1g \
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
            ros-humble-navigation2 \
            ros-humble-nav2-bringup \
            ros-humble-nav2-msgs \
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
# Also explicitly build isaac_ros_nvblox_utils as a mandatory target.
# =========================================================================
build_packages() {
    log_info "Building ROS2 packages..."

    # --- nvblox_ros (check it's available) ---
    if docker exec "$CONTAINER_NAME" bash -c "$ROS_SETUP; $WS_SETUP; ros2 pkg list 2>/dev/null | grep -q nvblox_ros"; then
        log_info "nvblox_ros already built"
    else
        if docker exec "$CONTAINER_NAME" test -d /workspaces/isaac_ros-dev/src/isaac_ros_nvblox; then
            log_info "Building nvblox packages (first time may take 10-30 minutes)..."
            docker exec "$CONTAINER_NAME" bash -c "
                $ROS_SETUP
                cd /workspaces/isaac_ros-dev
                colcon build --packages-up-to nvblox_examples_bringup --symlink-install --cmake-args -Wno-dev 2>&1
            " 2>&1 | tail -10
        else
            log_warn "nvblox source not found -- skipping build"
        fi
    fi

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
# Navigation 2 (Nav2) Pre-launch Validation
# =========================================================================
validate_nav2_config() {
    local nav2_params_file="/workspaces/isaac_ros-dev/config/nav2_drone.yaml"
    local goal_bridge="/workspaces/isaac_ros-dev/edge_core/ros/nav2_goal_bridge.py"
    local bt_xml="/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
    
    log_info "Validating Nav2 configuration before launch..."
    
    # Check 1: nav2_bringup package exists
    if ! docker exec "$CONTAINER_NAME" bash -c "$ROS_SETUP; ros2 pkg list 2>/dev/null | grep -q nav2_bringup"; then
        log_error "FATAL: nav2_bringup package not found in ROS2 install"
        log_error "Nav2 was not built or installed. Check container dependencies."
        return 1
    fi
    log_info "  [OK] nav2_bringup package found"
    
    # Check 2: BT XML file exists
    if ! docker exec "$CONTAINER_NAME" test -f "$bt_xml"; then
        log_error "FATAL: Default BT XML file not found: $bt_xml"
        log_error "This file is required by nav2_bringup. Verify nav2_bringup installation."
        return 1
    fi
    log_info "  [OK] BT XML file exists: $bt_xml"
    
    # Check 3: Nav2 config file exists
    if ! docker exec "$CONTAINER_NAME" test -f "$nav2_params_file"; then
        log_error "FATAL: Nav2 config file not found: $nav2_params_file"
        log_error "Verify NOMAD config volume mount or nav2_drone.yaml availability."
        return 1
    fi
    log_info "  [OK] Nav2 config file exists: $nav2_params_file"
    
    # Check 4: Nav2 goal bridge script exists
    if ! docker exec "$CONTAINER_NAME" test -f "$goal_bridge"; then
        log_error "FATAL: Nav2 goal bridge script not found: $goal_bridge"
        log_error "Verify edge_core volume mount or nav2_goal_bridge.py availability."
        return 1
    fi
    log_info "  [OK] Nav2 goal bridge script exists: $goal_bridge"
    
    # Check 5: Validate nav2_drone.yaml YAML syntax
    if ! docker exec "$CONTAINER_NAME" python3 -c "import yaml; yaml.safe_load(open('$nav2_params_file')); print('YAML OK')" 2>/dev/null | grep -q "YAML OK"; then
        log_error "FATAL: Nav2 config file has invalid YAML syntax: $nav2_params_file"
        log_error "Fix YAML errors before relaunching Nav2."
        return 1
    fi
    log_info "  [OK] Nav2 config file YAML syntax valid"
    
    log_info "Nav2 validation PASSED - safe to launch"
    return 0
}

# =========================================================================
# Launch ZED + nvblox
# =========================================================================
launch_zed_nvblox() {
    # PREFLIGHT: nvblox_ros MUST be available (hard fail if missing)
    if ! docker exec "$CONTAINER_NAME" bash -c "$ROS_SETUP; $WS_SETUP; ros2 pkg list 2>/dev/null | grep -q nvblox_ros"; then
        log_error "FATAL: nvblox_ros package not found in container"
        log_error "Build it with: colcon build --packages-up-to nvblox_examples_bringup"
        exit 1
    fi

    # Check if nvblox is available
    if docker exec "$CONTAINER_NAME" bash -c "$ROS_SETUP; $WS_SETUP; ros2 pkg list 2>/dev/null | grep -q nvblox_examples_bringup"; then
        log_info "Launching ZED + nvblox (camera:=zed2)..."

        # Kill any existing launch/container processes first to avoid duplicate
        # camera initialization and Argus resource contention.
        # Kill processes from ALL launch paths (startup script AND API-triggered launches)
        # Then clean stale FastRTPS SHM locks so new ROS2 nodes can acquire ports.
        # Kill existing ROS processes. Use [r]os trick to avoid pkill matching itself.
        docker exec "$CONTAINER_NAME" bash -c '
            pkill -f "[l]aunch_nvblox_bridge" 2>/dev/null || true
            pkill -f "[l]aunch_zed_nvblox" 2>/dev/null || true
            pkill -f "[n]omad_zed_nvblox" 2>/dev/null || true
            pkill -f "[z]ed_example.launch" 2>/dev/null || true
            pkill -f "[c]omponent_container" 2>/dev/null || true
            pkill -f "[r]os_http_bridge" 2>/dev/null || true
            sleep 2
            rm -f /dev/shm/fastrtps_* 2>/dev/null || true
        '

        # Write launch script to host temp file, then copy into container.
        # (heredoc + docker exec -i fails when run under nohup/background)
        local _launch_tmp
        _launch_tmp=$(mktemp /tmp/launch_zed_nvblox.XXXXXX.sh)
        cat > "$_launch_tmp" << 'LAUNCH_SCRIPT'
#!/bin/bash
# Bind uvcvideo driver to ZED camera USB interfaces
echo "[init] Binding uvcvideo driver to ZED camera..."
for dev in /sys/bus/usb/devices/*/idVendor; do
  dir=$(dirname $dev)
  vid=$(cat $dev 2>/dev/null)
  if [ "$vid" = "2b03" ]; then
    for iface in $dir/*:*/bInterfaceClass; do
      idir=$(dirname $iface)
      cls=$(cat $iface 2>/dev/null)
      iname=$(basename $idir)
      if [ "$cls" = "0e" ] && [ ! -e $idir/driver ]; then
        echo "[init] Binding uvcvideo to $iname (Video class)"
        echo $iname > /sys/bus/usb/drivers/uvcvideo/bind 2>/dev/null || true
      fi
    done
  fi
done
sleep 1
ls /dev/video* 2>/dev/null && echo "[init] Video devices ready." || echo "[init] Warning: No /dev/video devices found."

# Set LD_LIBRARY_PATH BEFORE sourcing setup.bash so ROS libraries are found
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/local/zed/lib:${LD_LIBRARY_PATH:-}
source /opt/ros/humble/install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
export EGL_PLATFORM=device
# Patch ZED publish resolution to native 720p
sed -i 's/pub_downscale_factor: 2\.0/pub_downscale_factor: 1.0/' \
    /workspaces/isaac_ros-dev/install/zed_wrapper/share/zed_wrapper/config/common.yaml 2>/dev/null
# Overlay NOMAD nvblox config onto installed base config
# Performance profile: 0.10m voxels, 10-15Hz rates, 2D ESDF, 8m radius
# Tuned for Orin Nano 8GB — see config/nvblox_performance.yaml
NOMAD_CFG=/workspaces/isaac_ros-dev/config/nvblox_performance.yaml
NVBLOX_BASE=$(python3 -c "from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory('nvblox_examples_bringup'))" 2>/dev/null)/config/nvblox/nvblox_base.yaml
if [ -f "$NOMAD_CFG" ] && [ -f "$NVBLOX_BASE" ]; then
    echo "Applying NOMAD nvblox config (performance profile: 0.10m voxels, 10-15Hz, 2D ESDF, 8m radius)"
    cp "$NOMAD_CFG" "$NVBLOX_BASE"
else
    echo "NOMAD config or nvblox base not found, using defaults"
fi
# Preflight check: ensure nav2 dependencies are available
NAV2_PREFLIGHT_OK=true
if ! dpkg -l ros-humble-nav2-bringup 2>/dev/null | grep -q '^ii'; then
    echo "[WARN] ros-humble-nav2-bringup not installed - nav2 will be disabled"
    NAV2_PREFLIGHT_OK=false
fi
if ! ros2 pkg list 2>/dev/null | grep -q nvblox_nav2; then
    echo "[WARN] nvblox nav2 plugins not found - nav2 costmap will be unavailable"
    NAV2_PREFLIGHT_OK=false
fi

# Use custom NOMAD launch file with YOLO object detection and nav2 enabled
NOMAD_LAUNCH=/workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py
if [ -f "$NOMAD_LAUNCH" ]; then
    echo "Launching with NOMAD custom OD launch file (nav2 enabled)"
    if [ "$NAV2_PREFLIGHT_OK" = true ]; then
        ros2 launch "$NOMAD_LAUNCH" enable_nav2:=true
    else
        echo "[WARN] Nav2 preflight check failed - falling back to nav2 disabled"
        ros2 launch "$NOMAD_LAUNCH" enable_nav2:=false
    fi
else
    echo "NOMAD launch file not found, falling back to stock launch (no OD, nav2 disabled)"
    ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2
fi

# Post-launch validation: confirm nav2 lifecycle nodes are running (give 5s for startup)
if [ "$NAV2_PREFLIGHT_OK" = true ]; then
    sleep 5
    NAV2_NODES=$(ros2 node list 2>/dev/null | grep -E 'nav2_.*lifecycle' | wc -l)
    if [ "$NAV2_NODES" -gt 0 ]; then
        echo "[INFO] Nav2 lifecycle nodes detected ($NAV2_NODES running) - nav2 active"
    else
        echo "[WARN] No Nav2 lifecycle nodes detected - nav2 may have failed to start"
    fi
fi
LAUNCH_SCRIPT
        docker cp "$_launch_tmp" "$CONTAINER_NAME:/tmp/launch_zed_nvblox.sh"
        rm -f "$_launch_tmp"
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

    local _zed_tmp
    _zed_tmp=$(mktemp /tmp/launch_zed_only.XXXXXX.sh)
    cat > "$_zed_tmp" << 'LAUNCH_SCRIPT'
#!/bin/bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/local/zed/lib:${LD_LIBRARY_PATH:-}
source /opt/ros/humble/install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
export EGL_PLATFORM=device
# Patch ZED publish resolution to native 720p
sed -i 's/pub_downscale_factor: 2\.0/pub_downscale_factor: 1.0/' \
    /workspaces/isaac_ros-dev/install/zed_wrapper/share/zed_wrapper/config/common.yaml 2>/dev/null
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
LAUNCH_SCRIPT
    docker cp "$_zed_tmp" "$CONTAINER_NAME:/tmp/launch_zed_only.sh"
    rm -f "$_zed_tmp"
    docker exec "$CONTAINER_NAME" chmod +x /tmp/launch_zed_only.sh

    docker exec -d "$CONTAINER_NAME" bash -c \
        "bash /tmp/launch_zed_only.sh > /tmp/zed_nvblox.log 2>&1 & echo \$! > /tmp/zed_nvblox.pid"

    log_info "ZED wrapper launched (logs: /tmp/zed_nvblox.log inside container)"
}

# =========================================================================
# Video Bridge (REMOVED - now launched by Edge Core via /api/video/start)
# (Previously subscribed to ROS image topics and pushed H264 to MediaMTX)
# Single owner: Edge Core's VideoStreamManager prevents flapping/duplicates
# =========================================================================

# =========================================================================
# ROS-HTTP Bridge (relays ROS2 data to Edge Core API)
# =========================================================================
launch_ros_http_bridge() {
    log_info "Launching ROS-HTTP bridge..."

    # Bridge script is available via volume mount at /workspaces/isaac_ros-dev/edge_core/
    # Kill any existing bridge processes to prevent duplicates
    docker exec "$CONTAINER_NAME" bash -c 'pkill -f ros_http_bridge.py 2>/dev/null || true; sleep 1'
    
    local _bridge_tmp
    _bridge_tmp=$(mktemp /tmp/launch_bridge.XXXXXX.sh)
    cat > "$_bridge_tmp" << 'BRIDGE_SCRIPT'
#!/bin/bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/local/zed/lib:${LD_LIBRARY_PATH:-}
source /opt/ros/humble/install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
# Wait for ZED node to fully start and DDS discovery to complete
# (ZED + nvblox take ~20-30s to init)
sleep 30
python3 /workspaces/isaac_ros-dev/edge_core/ros_http_bridge.py --host localhost --port 8000 --rate 30 --vio-topic /zed/zed_node/odom
BRIDGE_SCRIPT
    docker cp "$_bridge_tmp" "$CONTAINER_NAME:/tmp/launch_bridge.sh"
    rm -f "$_bridge_tmp"
    docker exec "$CONTAINER_NAME" chmod +x /tmp/launch_bridge.sh

    docker exec -d "$CONTAINER_NAME" bash -c \
        "nohup /tmp/launch_bridge.sh > /tmp/ros_bridge.log 2>&1 & echo \$! > /tmp/ros_bridge.pid"

    log_info "ROS-HTTP bridge launched (logs: /tmp/ros_bridge.log inside container)"
    
    # Validate process startup: wait briefly and check PID
    sleep 2
    BRIDGE_PID=$(docker exec "$CONTAINER_NAME" cat /tmp/ros_bridge.pid 2>/dev/null)
    if [ -z "$BRIDGE_PID" ] || ! docker exec "$CONTAINER_NAME" kill -0 "$BRIDGE_PID" 2>/dev/null; then
        log_error "Failed: ros_http_bridge did not start. Check logs:"
        docker exec "$CONTAINER_NAME" tail -20 /tmp/ros_bridge.log
        return 1
    fi
    log_info "ros_http_bridge PID $BRIDGE_PID confirmed running"
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

        # Pre-validate Nav2 configuration (runs early so issues are caught before launch)
        if ! validate_nav2_config; then
            log_error "Nav2 validation FAILED. Run with 'bash $0 logs' to diagnose."
            log_error "CONTINUING WITH LAUNCH (Nav2 disabled, but config must be resolvable for robustness)."
        fi

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
        log_info "Isaac ROS startup complete! (Video bridge will be started by Edge Core /api/video/start)"
        show_status
        ;;
    stop)    stop_services ;;
    restart) stop_services; sleep 2; exec "$0" start ;;
    status)  show_status ;;
    logs)    show_logs "${2:-all}" ;;
    shell)   docker exec -it "$CONTAINER_NAME" bash ;;
    *)       echo "Usage: $0 {start|stop|restart|status|logs|shell}"; exit 1 ;;
esac
