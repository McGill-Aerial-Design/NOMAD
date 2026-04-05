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
set -o pipefail

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

# ZED ROS2 wrapper branch matching ZED SDK 5.2
ZED_WRAPPER_BRANCH="v5.2.0"
ZED_SDK_VERSION="5.2.3"
ZED_SDK_L4T_TARGET="l4t36.4"

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
# Bridge Authentication Environment
# =========================================================================
load_bridge_auth_env() {
    local env_file="${REPO_ROOT}/config/env/jetson.env"

    if [ -f "$env_file" ]; then
        if [ -z "${NOMAD_API_KEY:-}" ]; then
            local api_key
            api_key=$(grep -E '^NOMAD_API_KEY=' "$env_file" | tail -n1 | cut -d= -f2- | tr -d '\r')
            if [ -n "$api_key" ]; then
                export NOMAD_API_KEY="$api_key"
                log_info "Loaded NOMAD_API_KEY from config/env/jetson.env for bridge auth"
            fi
        fi

        if [ -z "${NOMAD_INTERNAL_TOKEN:-}" ]; then
            local internal_token
            internal_token=$(grep -E '^NOMAD_INTERNAL_TOKEN=' "$env_file" | tail -n1 | cut -d= -f2- | tr -d '\r')
            if [ -n "$internal_token" ]; then
                export NOMAD_INTERNAL_TOKEN="$internal_token"
                log_info "Loaded NOMAD_INTERNAL_TOKEN from config/env/jetson.env for bridge auth"
            fi
        fi
    fi

    if [ -z "${NOMAD_API_KEY:-}" ]; then
        log_warn "NOMAD_API_KEY is not set; ros_http_bridge may receive 401 from Edge Core"
    fi
}

# =========================================================================
# Zombie Process Cleanup
# =========================================================================
cleanup_zombies() {
    # Reap zombie processes by waiting for any terminated children
    # This is called periodically to prevent zombie accumulation
    docker exec "$CONTAINER_NAME" bash -c '
        # Find all zombie processes
        zombies=$(ps aux | grep defunct | grep -v grep | awk "{print \$2}" || true)
        if [ -n "$zombies" ]; then
            echo "[CLEANUP] Found zombie processes, attempting cleanup..."
            # Wait for init (PID 1) to reap orphaned zombies
            # In containers, this happens automatically but may be delayed
            sleep 1
        fi
    ' 2>/dev/null || true
}

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
    if command -v nvpmodel &> /dev/null; then
        NVP_OUT="$(nvpmodel -q 2>/dev/null || true)"
        POWER_MODE_LINE="$(printf '%s\n' "$NVP_OUT" | grep -m1 -E 'NV Power Mode|Power Mode' | sed 's/^[[:space:]]*//' || true)"

        if [ -n "$POWER_MODE_LINE" ]; then
            if printf '%s\n' "$POWER_MODE_LINE" | grep -qiE 'MAXN|25W'; then
                log_info "Power mode: High performance ($POWER_MODE_LINE) - OK"
            else
                log_warn "Power mode may be limited ($POWER_MODE_LINE). nvblox may throttle."
                log_warn "Run 'sudo nvpmodel -m 2' for 25W mode or 'sudo jetson_clocks' for max performance"
            fi
        else
            log_warn "nvpmodel is available but power mode could not be parsed"
            log_warn "For best results, run: sudo nvpmodel -m 2 && sudo jetson_clocks"
        fi
    elif [ -e /sys/devices/virtual/thermal/cooling_device0/cur_state ]; then
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
container_uses_init() {
    local init_enabled
    init_enabled=$(docker inspect -f '{{.HostConfig.Init}}' "$CONTAINER_NAME" 2>/dev/null || echo "false")
    [ "$init_enabled" = "true" ]
}

container_has_usb_sys_mount() {
    docker inspect -f '{{range .Mounts}}{{.Destination}} {{end}}' "$CONTAINER_NAME" 2>/dev/null | grep -q '/sys/bus/usb'
}

start_container() {
    log_info "Starting Isaac ROS container..."

    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        if ! container_uses_init || ! container_has_usb_sys_mount; then
            log_warn "Container missing required runtime settings; recreating"
            if ! container_uses_init; then
                log_warn "  Reason: init reaper not enabled"
            fi
            if ! container_has_usb_sys_mount; then
                log_warn "  Reason: /sys/bus/usb not mounted"
            fi
            if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
                docker stop "$CONTAINER_NAME" 2>/dev/null || true
            fi
            docker rm "$CONTAINER_NAME" 2>/dev/null || true
        else
            if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
                log_info "Container already running"
                return 0
            fi

            log_info "Container exists but stopped, starting..."
            docker start "$CONTAINER_NAME"
            sleep 2
            return 0
        fi
    fi

    log_info "Creating new container from $IMAGE_NAME ..."
    docker run -d \
        --name "$CONTAINER_NAME" \
        --init \
        --runtime nvidia \
        --privileged \
        --network host \
        --ipc host \
        --shm-size 1g \
        -v "$ISAAC_WS:/workspaces/isaac_ros-dev" \
        -v /dev:/dev \
        -v /sys/bus/usb:/sys/bus/usb \
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
        -e LD_LIBRARY_PATH=/usr/local/zed/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu \
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

get_installed_zed_sdk_version() {
    docker exec "$CONTAINER_NAME" bash -c '
        if [ ! -f /usr/local/zed/include/sl/Camera.hpp ]; then
            exit 1
        fi

        major=$(grep -m1 "^#define ZED_SDK_MAJOR_VERSION " /usr/local/zed/include/sl/Camera.hpp | awk "{print \$3}")
        minor=$(grep -m1 "^#define ZED_SDK_MINOR_VERSION " /usr/local/zed/include/sl/Camera.hpp | awk "{print \$3}")
        patch=$(grep -m1 "^#define ZED_SDK_PATCH_VERSION " /usr/local/zed/include/sl/Camera.hpp | awk "{print \$3}")

        if [ -n "$major" ] && [ -n "$minor" ] && [ -n "$patch" ]; then
            echo "$major.$minor.$patch"
            exit 0
        fi

        exit 1
    ' 2>/dev/null
}

zed_component_sdk_abi_ok() {
    docker exec "$CONTAINER_NAME" bash -c '
        comp="/workspaces/isaac_ros-dev/install/zed_components/lib/libzed_camera_component.so"
        if [ ! -f "$comp" ]; then
            exit 1
        fi

        GXF_LIB_DIRS=$(find /opt/ros/humble/share -path "*/gxf/lib" -type d 2>/dev/null | tr "\n" ":")
        export LD_LIBRARY_PATH=/usr/local/zed/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:${GXF_LIB_DIRS}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

        # Detect ZED SDK ABI mismatches (common cause: wrapper built for SDK 5.x
        # but runtime still has SDK 4.x, or vice versa).
        if ldd -r "$comp" 2>&1 | grep -q "undefined symbol: _ZN2sl"; then
            exit 1
        fi

        exit 0
    '
}

# =========================================================================
# Install runtime dependencies inside the container
# Per the guide these get wiped on every container restart so we re-install.
# =========================================================================
install_dependencies() {
    log_info "Installing runtime dependencies inside container..."

    # --- ZED SDK ---
    local reinstall_zed_sdk=true
    local detected_zed_version=""

    if docker exec "$CONTAINER_NAME" test -f /usr/local/zed/lib/libsl_zed.so 2>/dev/null; then
        detected_zed_version="$(get_installed_zed_sdk_version || true)"
        detected_zed_version="${detected_zed_version//$'\r'/}"

        if [ "$detected_zed_version" = "$ZED_SDK_VERSION" ]; then
            reinstall_zed_sdk=false
            log_info "ZED SDK already installed ($detected_zed_version)"
        elif [ -n "$detected_zed_version" ]; then
            log_warn "Detected ZED SDK $detected_zed_version, expected $ZED_SDK_VERSION"
            log_warn "Reinstalling ZED SDK to fix wrapper/runtime ABI mismatch"
        else
            log_warn "Detected ZED SDK but could not parse version from headers"
            log_warn "Reinstalling ZED SDK to ensure compatibility"
        fi
    fi

    if [ "$reinstall_zed_sdk" = true ]; then
        log_info "Installing ZED SDK ${ZED_SDK_VERSION} inside container..."
        docker exec "$CONTAINER_NAME" bash -c "
            apt-get update -qq
            apt-get install -y --no-install-recommends zstd wget
            wget -q 'https://download.stereolabs.com/zedsdk/${ZED_SDK_VERSION}/${ZED_SDK_L4T_TARGET}/jetsons' -O /tmp/zed_installer.run
            chmod +x /tmp/zed_installer.run
            /tmp/zed_installer.run -- silent skip_od_module
            rm -f /tmp/zed_installer.run
            ldconfig
        " 2>&1 | tail -5

        detected_zed_version="$(get_installed_zed_sdk_version || true)"
        detected_zed_version="${detected_zed_version//$'\r'/}"

        if [ "$detected_zed_version" = "$ZED_SDK_VERSION" ]; then
            log_info "ZED SDK installed successfully ($detected_zed_version)"
        else
            if [ -z "$detected_zed_version" ]; then
                log_error "ZED SDK installation failed: unable to detect installed version"
            else
                log_error "ZED SDK installation mismatch: expected $ZED_SDK_VERSION, got $detected_zed_version"
            fi
            return 1
        fi
    fi

    # --- apt deps (per the guide, needed after every container restart) ---
    # Always fix pip cmake (it overrides system cmake and breaks colcon)
    docker exec "$CONTAINER_NAME" bash -c "pip3 uninstall -y cmake 2>/dev/null || true" 2>/dev/null
    # Keep numpy include layout compatible across numpy variants where either
    # core/include or _core/include may be present.
    docker exec "$CONTAINER_NAME" bash -c "
        NUMPY_BASE=/usr/local/lib/python3.10/dist-packages/numpy
        if [ -d \"\$NUMPY_BASE/_core/include\" ] && [ ! -e \"\$NUMPY_BASE/core/include\" ]; then
            mkdir -p \"\$NUMPY_BASE/core\" 2>/dev/null || true
            ln -sf \"\$NUMPY_BASE/_core/include\" \"\$NUMPY_BASE/core/include\" 2>/dev/null || true
        fi
        if [ -d \"\$NUMPY_BASE/core/include\" ] && [ ! -e \"\$NUMPY_BASE/_core/include\" ]; then
            mkdir -p \"\$NUMPY_BASE/_core\" 2>/dev/null || true
            ln -sf \"\$NUMPY_BASE/core/include\" \"\$NUMPY_BASE/_core/include\" 2>/dev/null || true
        fi
    " 2>/dev/null

    # Check ROS deps and GStreamer deps separately so GStreamer is never skipped
    local ros_deps_ok=false
    local gst_deps_ok=false
    if docker exec "$CONTAINER_NAME" bash -c "
        dpkg -s \
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
            >/dev/null 2>&1
    "; then
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
ensure_nvblox_built() {
    local pkg_check_cmd="$ROS_SETUP; $WS_SETUP; ros2 pkg list 2>/dev/null | grep -q nvblox_ros"

    if docker exec "$CONTAINER_NAME" bash -c "$pkg_check_cmd"; then
        log_info "nvblox_ros already built"
        return 0
    fi

    if ! docker exec "$CONTAINER_NAME" test -d /workspaces/isaac_ros-dev/src/isaac_ros_nvblox; then
        log_warn "nvblox source not found at src/isaac_ros_nvblox -- skipping"
        log_warn "Clone it with: cd ~/workspaces/isaac_ros-dev/src && git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git"
        return 0
    fi

    log_info "Building nvblox packages (first time may take 10-30 minutes)..."
    docker exec "$CONTAINER_NAME" bash -c "
        $ROS_SETUP
        cd /workspaces/isaac_ros-dev
        colcon build --packages-up-to nvblox_examples_bringup --symlink-install --cmake-args -Wno-dev 2>&1
    " 2>&1 | tail -15
}

build_packages() {
    log_info "Building ROS2 packages..."

    ensure_nvblox_built

    # --- ZED wrapper ---
    local rebuild_zed=false
    if docker exec "$CONTAINER_NAME" bash -c "$ROS_SETUP; $WS_SETUP; ros2 pkg list 2>/dev/null | grep -q zed_wrapper"; then
        if zed_component_sdk_abi_ok; then
            log_info "ZED wrapper already built and ABI-compatible"
        else
            log_warn "ZED wrapper detected but ABI check failed; rebuilding zed interfaces/components"
            rebuild_zed=true
        fi
    else
        rebuild_zed=true
    fi

    if [ "$rebuild_zed" = true ]; then
        log_info "Building ZED interfaces/components (may take several minutes)..."
        docker exec "$CONTAINER_NAME" bash -c "
            $ROS_SETUP
            cd /workspaces/isaac_ros-dev
            colcon build --packages-select zed_interfaces zed_components zed_wrapper --symlink-install --cmake-args -Wno-dev 2>&1
        " 2>&1 | tail -20

        if ! zed_component_sdk_abi_ok; then
            log_error "ZED component ABI check failed after rebuild"
            log_error "Inspect with: docker exec $CONTAINER_NAME bash -c 'ldd -r /workspaces/isaac_ros-dev/install/zed_components/lib/libzed_camera_component.so | grep -E \"undefined symbol|not found\"'"
            return 1
        fi

        log_info "ZED wrapper/components rebuilt and ABI check passed"
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
        log_error "Nav2 validation error: nav2_bringup package not found in ROS2 install"
        log_error "Nav2 was not built or installed. Check container dependencies."
        return 1
    fi
    log_info "  [OK] nav2_bringup package found"
    
    # Check 2: BT XML file exists
    if ! docker exec "$CONTAINER_NAME" test -f "$bt_xml"; then
        log_error "Nav2 validation error: default BT XML file not found: $bt_xml"
        log_error "This file is required by nav2_bringup. Verify nav2_bringup installation."
        return 1
    fi
    log_info "  [OK] BT XML file exists: $bt_xml"
    
    # Check 3: Nav2 config file exists
    if ! docker exec "$CONTAINER_NAME" test -f "$nav2_params_file"; then
        log_error "Nav2 validation error: config file not found: $nav2_params_file"
        log_error "Verify NOMAD config volume mount or nav2_drone.yaml availability."
        return 1
    fi
    log_info "  [OK] Nav2 config file exists: $nav2_params_file"
    
    # Check 4: Nav2 goal bridge script exists
    if ! docker exec "$CONTAINER_NAME" test -f "$goal_bridge"; then
        log_error "Nav2 validation error: goal bridge script not found: $goal_bridge"
        log_error "Verify edge_core volume mount or nav2_goal_bridge.py availability."
        return 1
    fi
    log_info "  [OK] Nav2 goal bridge script exists: $goal_bridge"
    
    # Check 5: Validate nav2_drone.yaml YAML syntax
    if ! docker exec "$CONTAINER_NAME" python3 -c "import yaml; yaml.safe_load(open('$nav2_params_file')); print('YAML OK')" 2>/dev/null | grep -q "YAML OK"; then
        log_error "Nav2 validation error: config file has invalid YAML syntax: $nav2_params_file"
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
        # Use killall with proper waiting to avoid zombie processes.
        docker exec "$CONTAINER_NAME" bash -c '
            # Kill processes and wait for them to terminate
            for pattern in "[l]aunch_nvblox_bridge" "[l]aunch_zed_nvblox" "[n]omad_zed_nvblox" "[z]ed_example.launch" "[c]omponent_container" "[r]os_http_bridge"; do
                pkill -f "$pattern" 2>/dev/null || true
            done
            
            # Wait for processes to fully terminate (max 5 seconds)
            timeout=10
            while [ $timeout -gt 0 ]; do
                if ! pgrep -f "[l]aunch_nvblox_bridge|[l]aunch_zed_nvblox|[n]omad_zed_nvblox|[z]ed_example\.launch|[c]omponent_container|[r]os_http_bridge" >/dev/null 2>&1; then
                    break
                fi
                sleep 0.5
                timeout=$((timeout - 1))
            done
            
            # Force kill any remaining processes
            for pattern in "[l]aunch_nvblox_bridge" "[l]aunch_zed_nvblox" "[n]omad_zed_nvblox" "[z]ed_example.launch" "[c]omponent_container" "[r]os_http_bridge"; do
                pkill -9 -f "$pattern" 2>/dev/null || true
            done
            
            # Final wait to allow kernel to reap zombies
            sleep 1
            
            # Clean FastRTPS shared memory
            rm -f /dev/shm/fastrtps_* 2>/dev/null || true
        '

        # Write launch script to host temp file, then copy into container.
        # (heredoc + docker exec -i fails when run under nohup/background)
        local _launch_tmp
        _launch_tmp=$(mktemp /tmp/launch_zed_nvblox.XXXXXX.sh)
        cat > "$_launch_tmp" << 'LAUNCH_SCRIPT'
#!/bin/bash
# Clean up old ROS/nav2 processes before launching new ones
echo "[init] Cleaning up old ROS/nav2 processes..."
pkill -f '[c]ontroller_server|[p]lanner_server|[n]av2_.*lifecycle' 2>/dev/null || true
pkill -f '[r]os2 launch' | grep -v $$ 2>/dev/null || true
sleep 1

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

# Camera preflight with retry/rebind attempts.
cam_ready=false
for attempt in 1 2 3; do
    if grep -q '^2b03$' /sys/bus/usb/devices/*/idVendor 2>/dev/null && ls /dev/video* >/dev/null 2>&1; then
        cam_ready=true
        echo "[init] ZED camera detected (attempt $attempt)."
        break
    fi

    echo "[init] ZED camera not detected (attempt $attempt/3). Rebinding USB video interfaces..."
    for dev in /sys/bus/usb/devices/*/idVendor; do
        dir=$(dirname $dev)
        vid=$(cat $dev 2>/dev/null)
        if [ "$vid" = "2b03" ]; then
            for iface in $dir/*:*/bInterfaceClass; do
                idir=$(dirname $iface)
                cls=$(cat $iface 2>/dev/null)
                iname=$(basename $idir)
                if [ "$cls" = "0e" ]; then
                    echo $iname > /sys/bus/usb/drivers/uvcvideo/unbind 2>/dev/null || true
                    sleep 0.1
                    echo $iname > /sys/bus/usb/drivers/uvcvideo/bind 2>/dev/null || true
                fi
            done
        fi
    done
    sleep 2
done

if [ "$cam_ready" != "true" ]; then
    echo "[fatal] ZED camera not detected after retries; aborting nvblox launch"
    exit 3
fi

# Disable ZED object detection to avoid known instability with composable nodes in this pipeline
sed -i 's/od_enabled: true/od_enabled: false/' \
    /workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/sensors/zed_common.yaml 2>/dev/null || true

# Set LD_LIBRARY_PATH BEFORE sourcing setup.bash so ROS libraries are found
GXF_LIB_DIRS=$(find /opt/ros/humble/share -path '*/gxf/lib' -type d 2>/dev/null | tr '\n' ':')
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/usr/local/zed/lib:${GXF_LIB_DIRS}${LD_LIBRARY_PATH:-}
source /opt/ros/humble/install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
export EGL_PLATFORM=device
# Keep ZED at 360p (default downscale 2.0) to save GPU memory.
# 720p causes cudaErrorIllegalAddress when nvblox allocates GPU memory.
# sed -i 's/pub_downscale_factor: 2\.0/pub_downscale_factor: 1.0/' \
#     /workspaces/isaac_ros-dev/install/zed_wrapper/share/zed_wrapper/config/common.yaml 2>/dev/null
# Overlay NOMAD nvblox config onto installed base config
# Performance profile: 0.10m voxels, 5.0Hz rates, 2D ESDF, 5.0m radius
# Tuned for Orin Nano 8GB — see config/nvblox_performance.yaml
NOMAD_CFG=/workspaces/isaac_ros-dev/config/nvblox_performance.yaml
NVBLOX_BASE_A=$(python3 -c "from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory(\"nvblox_examples_bringup\"))" 2>/dev/null)/config/nvblox/nvblox_base.yaml
NVBLOX_BASE_B=/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml
NVBLOX_BASE=""
for cand in "$NVBLOX_BASE_A" "$NVBLOX_BASE_B"; do
    if [ -f "$cand" ]; then
        NVBLOX_BASE="$cand"
        break
    fi
done
if [ -f "$NOMAD_CFG" ] && [ -n "$NVBLOX_BASE" ]; then
    echo "Applying NOMAD nvblox config (performance profile: 0.10m voxels, 5.0Hz, 2D ESDF, 5.0m radius)"
        cp "$NOMAD_CFG" "$NVBLOX_BASE"
        echo "Overlay checksums:"
        sha256sum "$NOMAD_CFG" "$NVBLOX_BASE" 2>/dev/null || true
else
        echo "NOMAD config or nvblox base not found, using defaults"
        echo "  NOMAD_CFG=$NOMAD_CFG"
        echo "  NVBLOX_BASE_A=$NVBLOX_BASE_A"
        echo "  NVBLOX_BASE_B=$NVBLOX_BASE_B"
fi

# ZED SDK 5.2 publishes RGB on /zed/zed_node/rgb/color/rect/*, while
# older nvblox launch files still remap to /zed/zed_node/rgb/image_rect_color
# and /zed/zed_node/rgb/camera_info. Patch remaps in-place so color
# integration and downstream consumers receive RGB frames.
python3 << 'PYEOF_RGB_REMAP'
from pathlib import Path

launch_path = Path(
    "/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/"
    "nvblox_examples_bringup/launch/zed_nvblox_split.launch.py"
)

if not launch_path.exists():
    print("WARNING: nvblox split launch not found: " + str(launch_path))
else:
    text = launch_path.read_text()
    text_new = text.replace(
        "'/zed/zed_node/rgb/image_rect_color'",
        "'/zed/zed_node/rgb/color/rect/image'",
    )
    text_new = text_new.replace(
        "'/zed/zed_node/rgb/camera_info'",
        "'/zed/zed_node/rgb/color/rect/camera_info'",
    )
    if text_new != text:
        launch_path.write_text(text_new)
        print("Patched nvblox RGB remaps to ZED SDK 5.2 topics")
    else:
        print("nvblox RGB remaps already set")
PYEOF_RGB_REMAP

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

# Ensure default area-map path exists inside container so FilePath services
# (save_map/load_map) succeed with the Mission Planner default path.
mkdir -p /home/mad/NOMAD/data/area_maps 2>/dev/null || true

# Prevent duplicate helper/launch processes when restarting in an existing
# container (without full container recreation).
pkill -f 'nomad_zed_nvblox\.launch\.py|zed_camera\.launch\.py' 2>/dev/null || true
pkill -f 'component_container' 2>/dev/null || true
pkill -f 'controller_server|planner_server|smoother_server|behavior_server|bt_navigator|lifecycle_manager_navigation|waypoint_follower|velocity_smoother' 2>/dev/null || true
pkill -f 'obstacle_distance_bridge\.py|nav2_goal_bridge\.py|servo_tf_publisher\.py' 2>/dev/null || true
sleep 1

# Launch ZED + nvblox using NOMAD custom launch file.
# Includes: optical frame alias TF, servo TF publisher, obstacle distance bridge.
# Uses blocking CUDA stream (type 0) to prevent cudaErrorIllegalAddress on 8GB Jetson.
# Nav2 is disabled by default; enable with: enable_nav2:=true
echo "Launching ZED + nvblox (nomad_zed_nvblox.launch.py) with nav2 disabled..."
ros2 launch /workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py enable_nav2:=false &
LAUNCH_PID=$!

# Post-launch validation: nav2 is disabled by default and will not start
# To enable nav2, restart with: ./start_nomad_full.sh task2 ENABLE_NAV2=true
echo "[INFO] Nav2 is disabled by default (prevents duplicate instances)"
echo "[INFO] To enable nav2, use: ros2 launch ... enable_nav2:=true"

# Keep script long-running while launch remains active.
wait "$LAUNCH_PID"
LAUNCH_SCRIPT
        docker cp "$_launch_tmp" "$CONTAINER_NAME:/tmp/launch_zed_nvblox.sh"
        rm -f "$_launch_tmp"
        docker exec "$CONTAINER_NAME" chmod +x /tmp/launch_zed_nvblox.sh

        docker exec -d "$CONTAINER_NAME" bash -c \
            "bash /tmp/launch_zed_nvblox.sh > /tmp/zed_nvblox.log 2>&1 & echo \$! > /tmp/zed_nvblox.pid"

        log_info "ZED + nvblox launched (OD disabled by default in this path; logs: /tmp/zed_nvblox.log inside container)"
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
GXF_LIB_DIRS=$(find /opt/ros/humble/share -path '*/gxf/lib' -type d 2>/dev/null | tr '\n' ':')
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/usr/local/zed/lib:${GXF_LIB_DIRS}${LD_LIBRARY_PATH:-}
source /opt/ros/humble/install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
export EGL_PLATFORM=device
# Keep ZED at 360p (default downscale 2.0) to save GPU memory.
# 720p causes cudaErrorIllegalAddress when nvblox allocates GPU memory.
# sed -i 's/pub_downscale_factor: 2\.0/pub_downscale_factor: 1.0/' \
#     /workspaces/isaac_ros-dev/install/zed_wrapper/share/zed_wrapper/config/common.yaml 2>/dev/null
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
    load_bridge_auth_env

    # Bridge script is available via volume mount at /workspaces/isaac_ros-dev/edge_core/
    # Kill any existing bridge processes to prevent duplicates with proper reaping
    docker exec "$CONTAINER_NAME" bash -c '
        pkill -f "[r]os_http_bridge\.py" 2>/dev/null || true
        # Wait for process to terminate (max 3 seconds)
        timeout=6
        while [ $timeout -gt 0 ]; do
            if ! pgrep -f "[r]os_http_bridge\.py" >/dev/null 2>&1; then
                break
            fi
            sleep 0.5
            timeout=$((timeout - 1))
        done
        # Force kill if still running
        pkill -9 -f "[r]os_http_bridge\.py" 2>/dev/null || true
        sleep 0.5
    '
    
    local _bridge_tmp
    _bridge_tmp=$(mktemp /tmp/launch_bridge.XXXXXX.sh)
    cat > "$_bridge_tmp" << 'BRIDGE_SCRIPT'
#!/bin/bash
GXF_LIB_DIRS=$(find /opt/ros/humble/share -path '*/gxf/lib' -type d 2>/dev/null | tr '\n' ':')
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/usr/local/zed/lib:${GXF_LIB_DIRS}${LD_LIBRARY_PATH:-}
source /opt/ros/humble/install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
# Ensure package imports (for example, edge_core.*) resolve when launching by script path.
export PYTHONPATH=/workspaces/isaac_ros-dev:${PYTHONPATH:-}
# Wait for ZED node to fully start and DDS discovery to complete
# (ZED + nvblox take ~20-30s to init)
sleep 30
python3 /workspaces/isaac_ros-dev/edge_core/ros_http_bridge.py --host localhost --port 8000 --rate 30 --vio-topic /zed/zed_node/odom --mesh-topic /nvblox_node/color_layer_marker --high-rate-transport both
BRIDGE_SCRIPT
    docker cp "$_bridge_tmp" "$CONTAINER_NAME:/tmp/launch_bridge.sh"
    rm -f "$_bridge_tmp"
    docker exec "$CONTAINER_NAME" chmod +x /tmp/launch_bridge.sh

    local bridge_env_args=()
    if [ -n "${NOMAD_API_KEY:-}" ]; then
        bridge_env_args+=("-e" "NOMAD_API_KEY=$NOMAD_API_KEY")
    fi
    if [ -n "${NOMAD_INTERNAL_TOKEN:-}" ]; then
        bridge_env_args+=("-e" "NOMAD_INTERNAL_TOKEN=$NOMAD_INTERNAL_TOKEN")
    fi

    if [ ${#bridge_env_args[@]} -gt 0 ]; then
        docker exec "${bridge_env_args[@]}" -d "$CONTAINER_NAME" bash -c \
            "nohup /tmp/launch_bridge.sh > /tmp/ros_bridge.log 2>&1 & echo \$! > /tmp/ros_bridge.pid"
    else
        docker exec -d "$CONTAINER_NAME" bash -c \
            "nohup /tmp/launch_bridge.sh > /tmp/ros_bridge.log 2>&1 & echo \$! > /tmp/ros_bridge.pid"
    fi

    log_info "ROS-HTTP bridge launcher started (logs: /tmp/ros_bridge.log inside container)"

    # Validate launcher startup quickly. The real bridge process starts after
    # the warmup sleep inside /tmp/launch_bridge.sh.
    sleep 2
    BRIDGE_LAUNCHER_PID=$(docker exec "$CONTAINER_NAME" cat /tmp/ros_bridge.pid 2>/dev/null)
    if [ -z "$BRIDGE_LAUNCHER_PID" ] || ! docker exec "$CONTAINER_NAME" kill -0 "$BRIDGE_LAUNCHER_PID" 2>/dev/null; then
        log_error "Failed: ROS-HTTP bridge launcher did not start. Check logs:"
        docker exec "$CONTAINER_NAME" tail -20 /tmp/ros_bridge.log
        return 1
    fi
    if docker exec "$CONTAINER_NAME" pgrep -f "[r]os_http_bridge\.py" >/dev/null 2>&1; then
        log_info "ros_http_bridge.py process detected"
    else
        log_info "ROS-HTTP bridge launcher PID $BRIDGE_LAUNCHER_PID is running; ros_http_bridge.py will start after warmup (~30s)"
    fi
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
        docker exec "$CONTAINER_NAME" ps aux 2>/dev/null | grep -E "ros2|python3|nvblox|zed" | head -10 || true
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
            log_error "CONTINUING WITH LAUNCH (validation warning only; this check does not force-disable Nav2)."
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
