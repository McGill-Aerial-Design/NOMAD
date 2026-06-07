#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# NOMAD systemd setup — installs the per-service units.
# Must be run with sudo: sudo bash scripts/setup/setup_service.sh
#
# This is a thin wrapper around infra/systemd/install.sh, kept for backward
# compatibility with existing setup runbooks. New runbooks should call the
# install script directly.
# =============================================================================
set -e

if [ "$EUID" -ne 0 ]; then
    echo "Please run with sudo: sudo bash $0"
    exit 1
fi

# Resolve the repo root from this script's location (scripts/setup/ -> repo root).
NOMAD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$NOMAD_DIR"

# Ensure the venv exists and has the runtime deps (the edge_core systemd unit
# uses venv/bin/python3 directly).
if [ ! -d venv ]; then
    python3 -m venv venv
fi
# shellcheck disable=SC1091
. venv/bin/activate
pip install --upgrade pip -q
pip install -q fastapi uvicorn pydantic pydantic-settings pymavlink pyzmq psutil python-dotenv httpx

# Make sure GCS_IP / TAILSCALE_IP reflect this Jetson's current Tailscale IP.
NOMAD_ENV="$NOMAD_DIR/config/nomad.env"
if [ -f "$NOMAD_ENV" ]; then
    CURRENT_IP=$(tailscale ip -4 2>/dev/null || true)
    if [ -n "$CURRENT_IP" ]; then
        sed -i "s|^TAILSCALE_IP=.*|TAILSCALE_IP=$CURRENT_IP|" "$NOMAD_ENV" 2>/dev/null || true
    fi
else
    echo "WARNING: $NOMAD_ENV not found"
fi

# Install / reconcile per-service systemd units.
bash "$NOMAD_DIR/infra/systemd/install.sh"

echo
echo "Done. Bring everything up with:"
echo "  sudo systemctl start nomad.target"
echo "  systemctl status nomad-edge-core --no-pager"
