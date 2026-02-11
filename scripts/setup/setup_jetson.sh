#!/bin/bash
# ============================================================
# NOMAD Jetson Setup Script
# ============================================================
# Run this script on the Jetson Orin Nano to configure NOMAD
#
# Usage:
#   chmod +x setup_jetson.sh
#   ./setup_jetson.sh
# ============================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  NOMAD Jetson Orin Nano Setup${NC}"
echo -e "${GREEN}========================================${NC}"

# Configuration
# Use HOME environment variable for user-agnostic NOMAD path (default: /home/mad/NOMAD)
NOMAD_HOME="${HOME}/NOMAD"
NOMAD_USER="mad"
JETSON_TAILSCALE_IP="100.85.121.98"
GCS_TAILSCALE_IP="100.76.127.17"

# ============================================================
# 1. System Dependencies
# ============================================================
echo -e "\n${YELLOW}[1/7] Installing system dependencies...${NC}"

sudo apt-get update
sudo apt-get install -y \
    python3 python3-pip python3-venv \
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
echo "Testing connection to Ground Station ($GCS_TAILSCALE_IP)..."
if ping -c 2 $GCS_TAILSCALE_IP > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Ground Station reachable${NC}"
else
    echo -e "${YELLOW}⚠ Cannot reach Ground Station (may be offline)${NC}"
fi

# ============================================================
# 3. Clone/Update NOMAD Repository
# ============================================================
echo -e "\n${YELLOW}[3/7] Setting up NOMAD repository...${NC}"

if [ ! -d "$NOMAD_HOME" ]; then
    echo "Cloning NOMAD repository..."
    sudo mkdir -p "$(dirname $NOMAD_HOME)"
    sudo chown $USER:$USER "$(dirname $NOMAD_HOME)"
    git clone https://github.com/YoussGm3o8/NOMAD.git "$NOMAD_HOME"
else
    echo "Updating existing repository..."
    cd "$NOMAD_HOME"
    git pull
fi

# ============================================================
# 4. Setup Python Environment
# ============================================================
echo -e "\n${YELLOW}[4/7] Setting up Python environment...${NC}"

cd "$NOMAD_HOME"

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    python3 -m venv venv
fi

source venv/bin/activate

# Install dependencies
pip install --upgrade pip
pip install -r edge_core/requirements-jetson.txt 2>/dev/null || \
pip install -r edge_core/requirements.txt

# ============================================================
# 5. Configure MAVLink Router
# ============================================================
echo -e "\n${YELLOW}[5/7] Configuring MAVLink Router...${NC}"

sudo mkdir -p /etc/mavlink-router
sudo cp "$NOMAD_HOME/transport/mavlink_router/main.conf" /etc/mavlink-router/main.conf

# Update Ground Station IP in config
sudo sed -i "s/Address=.*/Address=$GCS_TAILSCALE_IP/" /etc/mavlink-router/main.conf

# Enable and start service
sudo systemctl enable mavlink-router
sudo systemctl restart mavlink-router

echo -e "${GREEN}MAVLink Router configured to send to $GCS_TAILSCALE_IP:14550${NC}"

# ============================================================
# 6. Configure Environment
# ============================================================
echo -e "\n${YELLOW}[6/7] Setting up environment configuration...${NC}"

# Copy environment file
cp "$NOMAD_HOME/config/env/jetson.env" "$NOMAD_HOME/.env"

# Update Tailscale IPs if they've changed
sed -i "s/TAILSCALE_IP=.*/TAILSCALE_IP=$CURRENT_IP/" "$NOMAD_HOME/.env"
sed -i "s/GCS_IP=.*/GCS_IP=$GCS_TAILSCALE_IP/" "$NOMAD_HOME/.env"

echo -e "${GREEN}Environment configured with:${NC}"
echo "  Jetson IP: $CURRENT_IP"
echo "  GCS IP: $GCS_TAILSCALE_IP"

# ============================================================
# 7. Configure Firewall
# ============================================================
echo -e "\n${YELLOW}[7/7] Configuring firewall...${NC}"

# Allow Tailscale subnet
sudo ufw allow from 100.0.0.0/8 to any port 22 proto tcp comment "SSH via Tailscale"
sudo ufw allow from 100.0.0.0/8 to any port 8000 proto tcp comment "NOMAD API via Tailscale"
sudo ufw allow from 100.0.0.0/8 to any port 8554 proto tcp comment "RTSP via Tailscale"
sudo ufw allow from 100.0.0.0/8 to any port 14550 proto udp comment "MAVLink via Tailscale"

# Also allow local network
sudo ufw allow from 192.168.0.0/16 to any port 22 proto tcp comment "SSH Local"
sudo ufw allow from 192.168.0.0/16 to any port 8000 proto tcp comment "NOMAD API Local"
sudo ufw allow from 192.168.0.0/16 to any port 8554 proto tcp comment "RTSP Local"
sudo ufw allow from 192.168.0.0/16 to any port 14550 proto udp comment "MAVLink Local"

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

echo -e "\n${YELLOW}To start NOMAD Edge Core:${NC}"
echo "  cd $NOMAD_HOME"
echo "  source venv/bin/activate"
echo "  python -m edge_core.main"

echo -e "\n${YELLOW}Or use systemd service:${NC}"
echo "  sudo cp $NOMAD_HOME/infra/nomad.service /etc/systemd/system/"
echo "  sudo systemctl daemon-reload"
echo "  sudo systemctl enable nomad"
echo "  sudo systemctl start nomad"

echo -e "\n${YELLOW}Test from Ground Station:${NC}"
echo "  curl http://$CURRENT_IP:8000/health"
echo "  ping $CURRENT_IP"

echo -e "\n${GREEN}Done!${NC}"
