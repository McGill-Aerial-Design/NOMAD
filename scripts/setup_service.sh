#!/bin/bash
# =============================================================================
# NOMAD Edge Core Service Setup
# Must be run with sudo: sudo bash scripts/setup_service.sh
# =============================================================================

if [ "$EUID" -ne 0 ]; then
    echo "Please run with sudo: sudo bash $0"
    exit 1
fi

cd /home/mad/NOMAD || exit 1

# Setup Python environment  
if [ ! -d "venv" ]; then
    python3 -m venv venv
fi

source venv/bin/activate
pip install --upgrade pip -q
pip install -q fastapi uvicorn pydantic pydantic-settings pymavlink pyzmq psutil python-dotenv httpx

# Setup config
cp -f config/env/jetson.env .env || touch .env
CURRENT_IP=$(tailscale ip -4 2>/dev/null || echo "100.85.121.98")
ENV_FILE=".env"
if grep -q "^TAILSCALE_IP=" "$ENV_FILE"; then
    sed -i "s|^TAILSCALE_IP=.*|TAILSCALE_IP=$CURRENT_IP|" "$ENV_FILE"
else
    echo "TAILSCALE_IP=$CURRENT_IP" >> "$ENV_FILE"
fi

# Install canonical systemd service from infra/nomad.service
NOMAD_DIR="/home/mad/NOMAD"
SERVICE_SRC="${NOMAD_DIR}/infra/nomad.service"

if [ ! -f "$SERVICE_SRC" ]; then
    echo "Error: Service unit file not found at $SERVICE_SRC"
    exit 1
fi

echo "Installing systemd unit from $SERVICE_SRC ..."
sudo cp "$SERVICE_SRC" /etc/systemd/system/nomad.service

# Enable and start
systemctl daemon-reload
systemctl enable nomad
systemctl restart nomad
sleep 3
systemctl status nomad --no-pager

