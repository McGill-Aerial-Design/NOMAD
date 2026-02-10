#!/bin/bash
# =============================================================================
# Setup Isaac ROS with ZED SDK
# =============================================================================
# This script configures Isaac ROS to build with ZED SDK support
# Run this once, then rebuild the Docker image with run_dev.sh
# =============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

ISAAC_WS="${HOME}/workspaces/isaac_ros-dev"
NOMAD_DIR="${HOME}/NOMAD"
ZED_DOCKER_DIR="${ISAAC_WS}/src/zed-ros2-wrapper/docker"

log_info "Setting up Isaac ROS with ZED SDK support..."

# Check if workspace exists
if [ ! -d "$ISAAC_WS" ]; then
    log_error "Isaac ROS workspace not found: $ISAAC_WS"
    exit 1
fi

# Check if ZED wrapper exists
if [ ! -d "${ISAAC_WS}/src/zed-ros2-wrapper" ]; then
    log_error "ZED ROS2 wrapper not found. Clone it first:"
    log_error "  cd ${ISAAC_WS}/src"
    log_error "  git clone https://github.com/stereolabs/zed-ros2-wrapper.git"
    exit 1
fi

# Create docker directory if it doesn't exist
mkdir -p "$ZED_DOCKER_DIR"

# Copy the Dockerfile
log_info "Copying ZED Dockerfile..."
cp "${NOMAD_DIR}/infra/docker/Dockerfile.isaac_ros_zed" "${ZED_DOCKER_DIR}/Dockerfile.user"
log_info "Created: ${ZED_DOCKER_DIR}/Dockerfile.user"

# Create Isaac ROS config
log_info "Creating Isaac ROS config..."
cat > ~/.isaac_ros_common-config << 'EOF'
# Isaac ROS configuration for ZED SDK support
CONFIG_IMAGE_KEY="ros2_humble.user"
CONFIG_DOCKER_SEARCH_DIRS=("${HOME}/workspaces/isaac_ros-dev/src/zed-ros2-wrapper/docker")
EOF
log_info "Created: ~/.isaac_ros_common-config"

# Show next steps
echo ""
echo "=============================================="
echo " Setup Complete!"
echo "=============================================="
echo ""
echo "Next steps:"
echo ""
echo "1. Rebuild the Isaac ROS Docker image (this takes 30-60 minutes):"
echo "   cd ${ISAAC_WS}"
echo "   ./src/isaac_ros_common/scripts/run_dev.sh"
echo ""
echo "2. Once rebuild completes, you can use the automatic startup:"
echo "   ${NOMAD_DIR}/scripts/start_isaac_ros_auto.sh start"
echo ""
echo "3. Or manually launch inside the container:"
echo "   ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2"
echo ""
echo "=============================================="
