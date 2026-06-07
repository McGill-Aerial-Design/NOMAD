#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# NOMAD Development Runner (Linux/macOS)
# =============================================================================
# Convenience script to run NOMAD Edge Core in simulation mode for development.
#
# This enables testing the full system (Web UI, Telemetry, Vision) on a laptop
# without requiring actual hardware (ZED camera, Jetson, ArduPilot).
#
# Usage:
#   ./scripts/dev/run_dev.sh           # Run with default settings
#   ./scripts/dev/run_dev.sh --port 8080  # Custom port
#   ./scripts/dev/run_dev.sh --no-vision  # Disable vision process
# =============================================================================

set -e

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}  NOMAD Edge Core - Development Mode${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""

# Check Python version
PYTHON_CMD=""
if command -v python3.13 &> /dev/null; then
    PYTHON_CMD="python3.13"
elif command -v python3 &> /dev/null; then
    PYTHON_CMD="python3"
elif command -v python &> /dev/null; then
    PYTHON_CMD="python"
else
    echo -e "${RED}Error: Python not found. Please install Python 3.13+${NC}"
    exit 1
fi

# Verify Python version
PY_VERSION=$($PYTHON_CMD -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo -e "${GREEN}Using Python ${PY_VERSION}${NC}"

# Change to project directory
cd "$PROJECT_ROOT"

# Set PYTHONPATH
export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH:-}"
echo -e "${GREEN}PYTHONPATH set to: ${PROJECT_ROOT}${NC}"

# Enable simulation mode
export NOMAD_SIM_MODE="true"
echo -e "${YELLOW}NOMAD_SIM_MODE=true (mock hardware enabled)${NC}"

# Additional environment variables for development
export VISION_VIDEO_SOURCE="zed"
export ENABLE_VIO="true"
export VISION_CONFIDENCE="0.5"

echo ""
echo -e "${BLUE}Starting Edge Core server...${NC}"
echo -e "${GREEN}API will be available at: http://localhost:8000${NC}"
echo -e "${GREEN}API docs at: http://localhost:8000/docs${NC}"
echo ""

# Run the Edge Core with simulation mode
# Pass through any additional arguments
$PYTHON_CMD -m edge_core.main --sim "$@"
