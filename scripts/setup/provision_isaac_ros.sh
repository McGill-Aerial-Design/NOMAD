#!/bin/bash
# =============================================================================
# One-time provisioning of the Isaac ROS container for NOMAD.
#
# Run this ONCE on a fresh Jetson (or after a clean container removal). It is
# idempotent — re-running just verifies/repairs the parts that are missing.
#
# What it does (extracted from the old start_isaac_ros_auto.sh):
#   1. Verifies the Isaac ROS image exists.
#   2. Clones the ZED ROS2 wrapper into ~/workspaces/isaac_ros-dev/src.
#   3. Starts the container (`nomad start isaac_ros_container`).
#   4. Installs the ZED SDK inside the container.
#   5. Installs ROS2 / GStreamer apt deps inside the container.
#   6. Builds and installs CUDA-enabled OpenCV for Python image processing.
#   7. Builds zed_wrapper / zed_components / nvblox packages with colcon.
#   8. Persists the resulting state into the image with `docker commit`.
#
# After this, runtime service scripts assume provisioning is done and will
# fail loudly with an actionable message if it isn't.
# =============================================================================
set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$THIS_DIR/../.." && pwd)"
# shellcheck source=../lib/common.sh
. "$REPO_ROOT/scripts/lib/common.sh"
SERVICE="provision"
load_nomad_env || exit 1

ROS_SETUP='source /opt/ros/humble/install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null'
WS_SETUP='source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null || true'

check_host_prereqs() {
    log_info "checking host prerequisites"
    command -v docker >/dev/null || { log_fail "docker not installed"; exit 1; }
    command -v git    >/dev/null || { log_fail "git not installed"; exit 1; }
    if ! docker image inspect "$ISAAC_IMAGE_NAME" >/dev/null 2>&1 \
       && ! docker image inspect "$ISAAC_IMAGE_FALLBACK" >/dev/null 2>&1; then
        log_fail "Isaac ROS image not found. Build it first:"
        log_fail "  cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts"
        log_fail "  ./build_image_layers.sh --image_key ros2_humble"
        exit 1
    fi
    [ -d "$ISAAC_WORKSPACE" ] || { log_fail "ISAAC_WORKSPACE not found: $ISAAC_WORKSPACE"; exit 1; }
    log_ok "host prerequisites satisfied"
}

clone_zed_wrapper() {
    local src="$ISAAC_WORKSPACE/src/zed-ros2-wrapper"
    if [ -d "$src/zed_wrapper" ]; then
        log_ok "ZED ROS2 wrapper already cloned"
        return
    fi
    log_info "cloning ZED ROS2 wrapper (branch $ZED_WRAPPER_BRANCH)"
    rm -rf "$src"
    git clone --branch "$ZED_WRAPPER_BRANCH" --depth 1 \
        https://github.com/stereolabs/zed-ros2-wrapper.git "$src"
    (cd "$src" && git submodule update --init --recursive)
}

ensure_container() {
    log_info "ensuring isaac_ros_container is running"
    "$REPO_ROOT/scripts/services/isaac_ros_container.sh" start
}

install_zed_sdk() {
    local installed
    installed=$(in_container 'if [ -f /usr/local/zed/include/sl/Camera.hpp ]; then
        m=$(grep -m1 "^#define ZED_SDK_MAJOR_VERSION " /usr/local/zed/include/sl/Camera.hpp | awk "{print \$3}")
        n=$(grep -m1 "^#define ZED_SDK_MINOR_VERSION " /usr/local/zed/include/sl/Camera.hpp | awk "{print \$3}")
        p=$(grep -m1 "^#define ZED_SDK_PATCH_VERSION " /usr/local/zed/include/sl/Camera.hpp | awk "{print \$3}")
        printf "%s.%s.%s" "$m" "$n" "$p"
    fi' 2>/dev/null | tr -d '\r' || true)

    if [ "$installed" = "$ZED_SDK_VERSION" ]; then
        log_ok "ZED SDK $installed already installed"
        return
    fi
    log_info "installing ZED SDK ${ZED_SDK_VERSION} (detected: ${installed:-none})"
    in_container "
        apt-get update -qq
        apt-get install -y --no-install-recommends zstd wget
        wget -q 'https://download.stereolabs.com/zedsdk/${ZED_SDK_VERSION}/${ZED_SDK_L4T_TARGET}/jetsons' -O /tmp/zed_installer.run
        chmod +x /tmp/zed_installer.run
        /tmp/zed_installer.run -- silent skip_od_module
        rm -f /tmp/zed_installer.run
        ldconfig
    " 2>&1 | tail -5
}

install_apt_deps() {
    log_info "installing ROS / GStreamer apt deps inside container"
    in_container '
        # pip-installed cmake breaks colcon; ensure system cmake wins.
        pip3 uninstall -y cmake 2>/dev/null || true

        # numpy core/_core layout shim for the cross-version build.
        NUMPY_BASE=/usr/local/lib/python3.10/dist-packages/numpy
        if [ -d "$NUMPY_BASE/_core/include" ] && [ ! -e "$NUMPY_BASE/core/include" ]; then
            mkdir -p "$NUMPY_BASE/core" && ln -sf "$NUMPY_BASE/_core/include" "$NUMPY_BASE/core/include" || true
        fi
        if [ -d "$NUMPY_BASE/core/include" ] && [ ! -e "$NUMPY_BASE/_core/include" ]; then
            mkdir -p "$NUMPY_BASE/_core" && ln -sf "$NUMPY_BASE/core/include" "$NUMPY_BASE/_core/include" || true
        fi

        apt-get update -qq
        apt-get install -y --no-install-recommends \
            ros-humble-zed-msgs ros-humble-nmea-msgs ros-humble-robot-localization \
            ros-humble-point-cloud-transport ros-humble-tf2-ros ros-humble-tf2-tools \
            ros-humble-cv-bridge \
            ros-humble-isaac-ros-managed-nitros ros-humble-isaac-ros-nitros \
            ros-humble-isaac-ros-nitros-image-type ros-humble-isaac-ros-nitros-camera-info-type \
            ros-humble-isaac-ros-nitros-point-cloud-type \
            ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-msgs \
            python3-pip \
            gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 \
            gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
            gstreamer1.0-rtsp gstreamer1.0-x
        pip3 install --no-cache-dir requests transforms3d "numpy<2" || true
        ldconfig
    ' 2>&1 | tail -5
}

install_opencv_cuda() {
    "$REPO_ROOT/scripts/setup/install_opencv_cuda_container.sh"
}

build_packages() {
    log_info "building ROS packages (zed + nvblox). First run takes 15-30 minutes."
    in_container "
        $ROS_SETUP
        cd /workspaces/isaac_ros-dev
        if [ -d src/isaac_ros_nvblox ]; then
            colcon build --packages-up-to nvblox_examples_bringup --symlink-install --cmake-args -Wno-dev 2>&1 | tail -20
        else
            echo '[provision] nvblox source not present at src/isaac_ros_nvblox -- skipping nvblox build'
        fi
        colcon build --packages-select zed_interfaces zed_components zed_wrapper --symlink-install --cmake-args -Wno-dev 2>&1 | tail -20
    "
}

persist_image() {
    log_info "committing container state to $ISAAC_IMAGE_NAME (so deps survive recreate)"
    docker commit "$ISAAC_CONTAINER_NAME" "$ISAAC_IMAGE_NAME" >/dev/null
    log_ok "image $ISAAC_IMAGE_NAME updated"
}

main() {
    check_host_prereqs
    clone_zed_wrapper
    ensure_container
    install_zed_sdk
    install_apt_deps
    install_opencv_cuda
    build_packages
    persist_image
    log_ok "provisioning complete. You can now run: nomad start all"
}

main "$@"
