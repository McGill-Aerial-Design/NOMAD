#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# ============================================================
# NOMAD Jetson Setup Script
# ============================================================
# Run this script on the Jetson Orin Nano to configure NOMAD
#
# Usage:
#   chmod +x setup_jetson.sh
#   ./setup_jetson.sh
# ============================================================

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  NOMAD Jetson Orin Nano Setup${NC}"
echo -e "${GREEN}========================================${NC}"

# Configuration
# Use HOME environment variable for a user-agnostic NOMAD path.
NOMAD_HOME="${HOME}/NOMAD"
NOMAD_USER="${NOMAD_USER:-$(id -un)}"
# Override this for an optional connectivity check. The router discovers a
# peer when GCS_IP is left blank in the canonical configuration.
GCS_TAILSCALE_IP="${GCS_TAILSCALE_IP:-}"

# ============================================================
# 1. System Dependencies
# ============================================================
echo -e "\n${YELLOW}[1/7] Installing system dependencies...${NC}"

sudo apt-get update
sudo apt-get install -y \
    python3 \
    build-essential cmake ninja-build \
    git curl wget \
    mavlink-router \
    ffmpeg \
    ufw

# ============================================================
# 2. Verify Tailscale
# ============================================================
echo -e "\n${YELLOW}[2/7] Verifying Tailscale connection...${NC}"

if ! command -v tailscale &> /dev/null; then
    echo -e "${RED}Tailscale not installed! Installing...${NC}"
    curl -fsSL https://tailscale.com/install.sh | sh
    echo -e "${YELLOW}Please run: sudo tailscale up --hostname=nomad-jetson${NC}"
    echo -e "${YELLOW}Then re-run this script.${NC}"
    exit 1
fi

CURRENT_IP=$(tailscale ip -4 2>/dev/null || echo "")
if [ -z "$CURRENT_IP" ]; then
    echo -e "${RED}Tailscale not connected! Please authenticate:${NC}"
    echo -e "${YELLOW}  sudo tailscale up --hostname=nomad-jetson${NC}"
    exit 1
fi

echo -e "${GREEN}Tailscale IP: $CURRENT_IP${NC}"

# Test connectivity to Ground Station
if [ -n "$GCS_TAILSCALE_IP" ]; then
    echo "Testing connection to Ground Station ($GCS_TAILSCALE_IP)..."
    if ping -c 2 "$GCS_TAILSCALE_IP" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ Ground Station reachable${NC}"
    else
        echo -e "${YELLOW}⚠ Cannot reach Ground Station (may be offline)${NC}"
    fi
else
    echo -e "${YELLOW}No GCS_TAILSCALE_IP supplied; router will discover a peer from config.${NC}"
fi

# ============================================================
# 3. Clone/Update NOMAD Repository
# ============================================================
echo -e "\n${YELLOW}[3/7] Setting up NOMAD repository...${NC}"

if [ ! -d "$NOMAD_HOME" ]; then
    echo "Cloning NOMAD repository..."
    sudo mkdir -p "$(dirname "$NOMAD_HOME")"
    sudo chown "$NOMAD_USER:$NOMAD_USER" "$(dirname "$NOMAD_HOME")"
    git clone https://github.com/McGill-Aerial-Design/NOMAD.git "$NOMAD_HOME"
else
    echo "Updating existing repository..."
    cd "$NOMAD_HOME"
    git pull --ff-only
fi
cd "$NOMAD_HOME"
git submodule update --init --recursive

# ============================================================
# 4. Build C++ Core and Validate Product Profile
# ============================================================
echo -e "\n${YELLOW}[4/7] Building C++ core and validating product profile...${NC}"

NOMAD_ENV="$NOMAD_HOME/config/nomad.env"
if [ ! -f "$NOMAD_ENV" ]; then
    echo -e "${RED}Missing $NOMAD_ENV. Copy the example and load a product profile first.${NC}" >&2
    exit 1
fi

if ! python3 "$NOMAD_HOME/scripts/profile.py" validate; then
    echo -e "${RED}Active product profile validation failed.${NC}" >&2
    exit 1
fi

# shellcheck disable=SC1090
. "$NOMAD_ENV"
case "${NOMAD_PROFILE:-}" in
    onboard_companion|groundstation_gpu|groundstation_minimal) ;;
    "")
        echo -e "${RED}NOMAD_PROFILE is missing; load a supported product profile first.${NC}" >&2
        exit 1
        ;;
    *)
        echo -e "${RED}Unsupported NOMAD_PROFILE: $NOMAD_PROFILE${NC}" >&2
        exit 1
        ;;
esac

OPTIONAL_FLAGS=(
    NOMAD_AUTOSTART_MEDIAMTX
    NOMAD_AUTOSTART_ISAAC_ROS_CONTAINER
    NOMAD_AUTOSTART_ROS_VEHICLE
    NOMAD_AUTOSTART_VIDEO_BRIDGE
)
for flag in "${OPTIONAL_FLAGS[@]}"; do
    case "${!flag:-}" in
        false|0) ;;
        true|1)
            echo -e "${RED}Unavailable: $flag is enabled before optional-provider qualification.${NC}" >&2
            echo -e "${RED}Set $flag=false or load a supported profile with it disabled.${NC}" >&2
            exit 1
            ;;
        *)
            echo -e "${RED}$flag must be explicitly false until its provider is qualified.${NC}" >&2
            exit 1
            ;;
    esac
done

cmake -S "$NOMAD_HOME" -B "$NOMAD_HOME/build/core" -DCMAKE_BUILD_TYPE=Release
cmake --build "$NOMAD_HOME/build/core" --parallel
echo -e "${GREEN}C++ core build complete${NC}"

# ============================================================
# 5. Install and Start Retained NOMAD Services
# ============================================================
echo -e "\n${YELLOW}[5/7] Installing and starting retained NOMAD services...${NC}"

sudo systemctl disable --now mavlink-router.service 2>/dev/null || true
sudo bash "$NOMAD_HOME/infra/systemd/install.sh"
sudo systemctl start nomad.target
echo -e "${GREEN}NOMAD service target started; optional workloads remain profile-controlled${NC}"

# ============================================================
# 6. Configure Firewall
# ============================================================
echo -e "\n${YELLOW}[6/7] Configuring firewall...${NC}"

echo -e "${GREEN}Environment configured with:${NC}"
echo "  Jetson IP: $CURRENT_IP"
echo "  Active profile: $NOMAD_PROFILE"
echo "  GCS check target: ${GCS_TAILSCALE_IP:-discovered by MAVLink Router}"

# Allow Tailscale subnet
sudo ufw allow from 100.0.0.0/8 to any port 22 proto tcp comment "SSH via Tailscale"
sudo ufw allow from 100.0.0.0/8 to any port 8554 proto tcp comment "RTSP via Tailscale"
sudo ufw allow from 100.0.0.0/8 to any port 14560 proto udp comment "MAVLink LTE via Tailscale"

# Also allow local network
sudo ufw allow from 192.168.0.0/16 to any port 22 proto tcp comment "SSH Local"
sudo ufw allow from 192.168.0.0/16 to any port 8554 proto tcp comment "RTSP Local"
sudo ufw allow from 192.168.0.0/16 to any port 14550 proto udp comment "MAVLink RadioMaster Local"

sudo ufw --force enable

echo -e "${GREEN}Firewall configured${NC}"

# ============================================================
# Setup Complete
# ============================================================
echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}  Setup Complete!${NC}"
echo -e "${GREEN}========================================${NC}"

echo -e "\n${YELLOW}Network Configuration:${NC}"
echo "  Jetson Tailscale IP: $CURRENT_IP"
echo "  Ground Station IP:   $GCS_TAILSCALE_IP"

echo -e "\n${YELLOW}One-time provisioning of the Isaac ROS container:${NC}"
echo "  bash $NOMAD_HOME/scripts/setup/provision_isaac_ros.sh"

echo -e "\n${YELLOW}Install systemd units (per-service) and reconcile autostart:${NC}"
echo "  sudo bash $NOMAD_HOME/infra/systemd/install.sh"

echo -e "\n${YELLOW}Start everything in the autostart set:${NC}"
echo "  $NOMAD_HOME/scripts/nomad start all"
echo "  # or, under systemd:"
echo "  sudo systemctl start nomad.target"

echo -e "\n${YELLOW}Check from Ground Station:${NC}"
echo "  ping $CURRENT_IP"
echo "  nomad status"

echo -e "\n${GREEN}Done!${NC}"
