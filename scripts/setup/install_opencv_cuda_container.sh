#!/bin/bash
# =============================================================================
# Build and install CUDA-enabled OpenCV inside the NOMAD Isaac ROS container.
#
# This script runs on the Jetson host. It installs into the running
# nomad_isaac_ros container and verifies that Python imports the CUDA build.
# The caller should commit the container image after this succeeds.
# =============================================================================
set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$THIS_DIR/../.." && pwd)"
# shellcheck source=../lib/common.sh
. "$REPO_ROOT/scripts/lib/common.sh"
SERVICE="opencv-cuda"
load_nomad_env || exit 1

OPENCV_CUDA_VERSION="${OPENCV_CUDA_VERSION:-4.10.0}"
OPENCV_CUDA_JOBS="${OPENCV_CUDA_JOBS:-2}"
OPENCV_CUDA_BUILD_ROOT="${OPENCV_CUDA_BUILD_ROOT:-/tmp/nomad-opencv-cuda}"
OPENCV_CUDA_BUILD_LIST="${OPENCV_CUDA_BUILD_LIST:-core,imgproc,imgcodecs,videoio,python3,cudaarithm,cudaimgproc,cudafilters,cudev}"
CUDA_ARCH_BIN="${OPENCV_CUDA_ARCH_BIN:-8.7}"

require_container

already_installed() {
    in_container 'python3 - <<'"'"'PY'"'"'
import sys
try:
    import cv2
    count = cv2.cuda.getCudaEnabledDeviceCount()
    build = cv2.getBuildInformation()
except Exception as exc:
    print(f"not-ready: {exc}")
    sys.exit(1)
print(cv2.__file__)
print(cv2.__version__)
print(f"cuda_devices={count}")
if count > 0 and "NVIDIA CUDA:                   YES" in build:
    sys.exit(0)
sys.exit(1)
PY'
}

install_build_deps() {
    log_info "installing OpenCV CUDA build dependencies in container"
    in_container '
        set -e
        apt-get update -qq
        apt-get install -y --no-install-recommends \
            build-essential cmake git pkg-config ninja-build \
            python3-dev python3-numpy python3-pip \
            libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev \
            libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
            libjpeg-dev libpng-dev libtiff-dev libopenexr-dev \
            libtbb-dev libv4l-dev libdc1394-dev \
            libeigen3-dev liblapack-dev libatlas-base-dev \
            gfortran ca-certificates curl
    ' 2>&1 | tail -20
}

remove_pip_wheels() {
    log_info "removing CPU-only pip OpenCV wheels that shadow CUDA OpenCV"
    in_container '
        set -e
        python3 -m pip uninstall -y \
            opencv-python opencv-python-headless \
            opencv-contrib-python opencv-contrib-python-headless || true
    ' 2>&1 | tail -20
}

build_and_install() {
    local cudnn_flags
    cudnn_flags='-D WITH_CUDNN=OFF -D OPENCV_DNN_CUDA=OFF'
    if in_container 'test -f /usr/include/cudnn_version.h || test -f /usr/local/cuda/include/cudnn_version.h'; then
        cudnn_flags='-D WITH_CUDNN=ON -D OPENCV_DNN_CUDA=ON'
    fi

    log_info "building OpenCV ${OPENCV_CUDA_VERSION} with CUDA arch ${CUDA_ARCH_BIN}; modules: ${OPENCV_CUDA_BUILD_LIST}"
    in_container "
        set -e
        export DEBIAN_FRONTEND=noninteractive
        export CUDA_HOME=/usr/local/cuda
        export PATH=\$CUDA_HOME/bin:\$PATH
        export LD_LIBRARY_PATH=\$CUDA_HOME/lib64:\${LD_LIBRARY_PATH:-}

        rm -rf '$OPENCV_CUDA_BUILD_ROOT'
        mkdir -p '$OPENCV_CUDA_BUILD_ROOT'
        cd '$OPENCV_CUDA_BUILD_ROOT'
        git clone --branch '$OPENCV_CUDA_VERSION' --depth 1 https://github.com/opencv/opencv.git
        git clone --branch '$OPENCV_CUDA_VERSION' --depth 1 https://github.com/opencv/opencv_contrib.git
        mkdir -p opencv/build
        cd opencv/build

        PYTHON_PACKAGES=\$(python3 - <<'PY'
import sysconfig
print(sysconfig.get_paths()[\"platlib\"])
PY
)
        PYTHON_INCLUDE=\$(python3 - <<'PY'
import sysconfig
print(sysconfig.get_paths()[\"include\"])
PY
)
        NUMPY_INCLUDE=\$(python3 - <<'PY'
import numpy
print(numpy.get_include())
PY
)

        cmake -G Ninja \
            -D CMAKE_BUILD_TYPE=Release \
            -D CMAKE_INSTALL_PREFIX=/usr/local \
            -D OPENCV_EXTRA_MODULES_PATH='$OPENCV_CUDA_BUILD_ROOT/opencv_contrib/modules' \
            -D BUILD_LIST='$OPENCV_CUDA_BUILD_LIST' \
            -D WITH_CUDA=ON \
            -D CUDA_ARCH_BIN='$CUDA_ARCH_BIN' \
            -D CUDA_ARCH_PTX= \
            -D ENABLE_FAST_MATH=ON \
            -D CUDA_FAST_MATH=ON \
            -D WITH_CUBLAS=ON \
            $cudnn_flags \
            -D WITH_GSTREAMER=ON \
            -D WITH_V4L=ON \
            -D WITH_OPENGL=OFF \
            -D BUILD_opencv_python3=ON \
            -D BUILD_opencv_python2=OFF \
            -D PYTHON3_EXECUTABLE=/usr/bin/python3 \
            -D PYTHON3_INCLUDE_DIR=\$PYTHON_INCLUDE \
            -D PYTHON3_NUMPY_INCLUDE_DIRS=\$NUMPY_INCLUDE \
            -D PYTHON3_PACKAGES_PATH=\$PYTHON_PACKAGES \
            -D OPENCV_GENERATE_PKGCONFIG=ON \
            -D BUILD_TESTS=OFF \
            -D BUILD_PERF_TESTS=OFF \
            -D BUILD_EXAMPLES=OFF \
            -D BUILD_DOCS=OFF \
            -D BUILD_JAVA=OFF \
            -D BUILD_opencv_world=OFF \
            ..

        ninja -j '$OPENCV_CUDA_JOBS'
        ninja install
        echo /usr/local/lib > /etc/ld.so.conf.d/opencv-cuda.conf
        ldconfig
    "
}

verify_install() {
    log_info "verifying Python OpenCV CUDA binding"
    in_container 'python3 - <<'"'"'PY'"'"'
import cv2

print(f"cv2_file={cv2.__file__}")
print(f"cv2_version={cv2.__version__}")
print(f"cuda_devices={cv2.cuda.getCudaEnabledDeviceCount()}")
cuda_lines = [
    line for line in cv2.getBuildInformation().splitlines()
    if "CUDA" in line or "cuDNN" in line
]
print("\n".join(cuda_lines))
if cv2.cuda.getCudaEnabledDeviceCount() < 1:
    raise SystemExit("OpenCV imported, but no CUDA device is visible")
if "NVIDIA CUDA:                   YES" not in cv2.getBuildInformation():
    raise SystemExit("OpenCV build information does not report CUDA support")
PY'
}

main() {
    if already_installed; then
        log_ok "CUDA-enabled OpenCV is already installed"
        return 0
    fi
    install_build_deps
    remove_pip_wheels
    build_and_install
    remove_pip_wheels
    verify_install
    log_ok "OpenCV CUDA install complete"
}

main "$@"
