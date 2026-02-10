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

# Create systemd service
tee /etc/systemd/system/nomad.service > /dev/null << 'EOF'
[Unit]
Description=NOMAD Edge Core
After=network-online.target

[Service]
Type=simple
User=mad
WorkingDirectory=/home/mad/NOMAD
Environment="PYTHONUNBUFFERED=1"
ExecStart=/home/mad/NOMAD/venv/bin/python -m uvicorn edge_core.main:app --host 0.0.0.0 --port 8000
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# Enable and start
systemctl daemon-reload
systemctl enable nomad
systemctl restart nomad
sleep 3
systemctl status nomad --no-pager

