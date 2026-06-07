#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# ============================================================
# NOMAD Tailscale Setup Script
# ============================================================
# Automated Tailscale installation and configuration for Jetson
#
# Usage:
#   sudo ./setup.sh                      # Interactive auth
#   sudo ./setup.sh --authkey <KEY>      # Auth key
#   sudo ./setup.sh --help               # Show help
#
# Requirements:
#   - Ubuntu 20.04+ (JetPack SDK)
#   - Internet connectivity
#   - Root privileges
# ============================================================

set -e

# Configuration
HOSTNAME="nomad-jetson"
TAILSCALE_INSTALL_URL="https://tailscale.com/install.sh"
LOG_FILE="/var/log/nomad-tailscale-setup.log"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ============================================================
# Helper Functions
# ============================================================

log() {
    echo -e "${GREEN}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} $1" | tee -a "$LOG_FILE"
}

warn() {
    echo -e "${YELLOW}[WARNING]${NC} $1" | tee -a "$LOG_FILE"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1" | tee -a "$LOG_FILE"
    exit 1
}

check_root() {
    if [[ $EUID -ne 0 ]]; then
        error "This script must be run as root (use sudo)"
    fi
}

check_internet() {
    log "Checking internet connectivity..."
    if ! ping -c 1 8.8.8.8 &> /dev/null; then
        error "No internet connectivity. Please connect to the internet first."
    fi
    log "Internet connectivity OK"
}

show_help() {
    echo "NOMAD Tailscale Setup Script"
    echo ""
    echo "Usage: sudo $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --authkey <KEY>    Use auth key for non-interactive authentication"
    echo "  --hostname <NAME>  Set custom hostname (default: nomad-jetson)"
    echo "  --help             Show this help message"
    echo ""
    echo "Examples:"
    echo "  sudo $0                           # Interactive login"
    echo "  sudo $0 --authkey tskey-auth-xxx  # Auth key login"
    echo ""
    echo "Get auth key from: https://login.tailscale.com/admin/settings/keys"
}

# ============================================================
# Installation Functions
# ============================================================

install_tailscale() {
    log "Installing Tailscale..."

    if command -v tailscale &> /dev/null; then
        warn "Tailscale is already installed"
        tailscale version
        return 0
    fi

    # Download and run install script
    curl -fsSL "$TAILSCALE_INSTALL_URL" | sh

    if ! command -v tailscale &> /dev/null; then
        error "Tailscale installation failed"
    fi

    log "Tailscale installed successfully"
    tailscale version
}

configure_service() {
    log "Configuring Tailscale service..."

    # Enable and start tailscaled
    systemctl enable tailscaled
    systemctl start tailscaled

    # Wait for service to be ready
    sleep 2

    if ! systemctl is-active --quiet tailscaled; then
        error "Failed to start tailscaled service"
    fi

    log "Tailscale service configured and running"
}

authenticate() {
    local authkey="$1"

    log "Authenticating with Tailscale..."

    if [[ -n "$authkey" ]]; then
        # Non-interactive auth with key
        log "Using auth key for authentication"
        tailscale up --authkey="$authkey" --hostname="$HOSTNAME"
    else
        # Interactive auth (generates URL)
        log "Starting interactive authentication..."
        echo ""
        echo "Please visit the URL below to authenticate:"
        echo ""
        tailscale up --hostname="$HOSTNAME"
    fi

    # Verify authentication
    sleep 3
    if tailscale status &> /dev/null; then
        log "Authentication successful"
    else
        error "Authentication failed"
    fi
}

configure_firewall() {
    log "Configuring firewall..."

    if ! command -v ufw &> /dev/null; then
        warn "UFW not installed, skipping firewall configuration"
        return 0
    fi

    # Allow Tailscale interface
    ufw allow in on tailscale0 2>/dev/null || true

    # Allow necessary ports from Tailscale network (100.0.0.0/8)
    ufw allow from 100.0.0.0/8 to any port 22 proto tcp 2>/dev/null || true    # SSH
    ufw allow from 100.0.0.0/8 to any port 8000 proto tcp 2>/dev/null || true  # API
    ufw allow from 100.0.0.0/8 to any port 8554 proto tcp 2>/dev/null || true  # RTSP
    ufw allow from 100.0.0.0/8 to any port 14560 proto udp 2>/dev/null || true # MAVLink LTE

    log "Firewall configured for Tailscale"
}

install_watchdog() {
    log "Installing Tailscale watchdog service..."

    local script_dir
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local watchdog_script="$script_dir/watchdog.sh"
    local service_file="$script_dir/../config/tailscale-watchdog.service"

    # Check if watchdog script exists
    if [[ ! -f "$watchdog_script" ]]; then
        warn "Watchdog script not found at $watchdog_script, skipping"
        return 0
    fi

    # Make watchdog executable
    chmod +x "$watchdog_script"

    # Determine NOMAD repo root from config or script path
    local repo_root="${NOMAD_REPO_ROOT:-$(cd "$script_dir/../.." && pwd)}"

    # Copy service file if exists
    if [[ -f "$service_file" ]]; then
        sed -e "s|__NOMAD_ROOT__|$repo_root|g" "$service_file" > /etc/systemd/system/tailscale-watchdog.service
        systemctl daemon-reload
        systemctl enable tailscale-watchdog
        systemctl start tailscale-watchdog
        log "Watchdog service installed and started"
    else
        warn "Watchdog service file not found, skipping"
    fi
}

verify_connection() {
    log "Verifying Tailscale connection..."

    echo ""
    echo "============================================================"
    echo "Tailscale Status:"
    echo "============================================================"
    tailscale status
    echo ""

    local ip
    ip=$(tailscale ip -4 2>/dev/null)
    if [[ -n "$ip" ]]; then
        echo "============================================================"
        echo "Your Tailscale IP: $ip"
        echo "Hostname: $HOSTNAME"
        echo "============================================================"
    else
        error "Could not get Tailscale IP address"
    fi
}

print_next_steps() {
    local ip
    ip=$(tailscale ip -4 2>/dev/null)

    echo ""
    echo "============================================================"
    echo "                     SETUP COMPLETE"
    echo "============================================================"
    echo ""
    echo "Tailscale is now configured on this Jetson."
    echo ""
    echo "Tailscale IP: $ip"
    echo "Hostname: $HOSTNAME"
    echo ""
    echo "Next steps:"
    echo ""
    echo "1. Install Tailscale on your Ground Station:"
    echo "   Windows: https://tailscale.com/download/windows"
    echo "   Linux: curl -fsSL https://tailscale.com/install.sh | sh"
    echo ""
    echo "2. Update MAVLink router config with Ground Station IP:"
    echo "   sudo nano /etc/mavlink-router/main.conf"
    echo "   # Change Address= in [UdpEndpoint groundstation]"
    echo ""
    echo "3. Test connectivity from Ground Station:"
    echo "   ping $ip"
    echo "   curl http://$ip:8000/health"
    echo ""
    echo "4. Configure Mission Planner:"
    echo "   Jetson IP: $ip"
    echo "   Main MAVLink connection: UDP port 14600"
    echo "   NOMAD plugin LTE input: UDP port 14560"
    echo "   NOMAD plugin RadioMaster input: UDP port 14550"
    echo "   API: http://$ip:8000"
    echo ""
    echo "============================================================"
}

# ============================================================
# Main
# ============================================================

main() {
    local authkey=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --authkey)
                authkey="$2"
                shift 2
                ;;
            --hostname)
                HOSTNAME="$2"
                shift 2
                ;;
            --help)
                show_help
                exit 0
                ;;
            *)
                error "Unknown option: $1. Use --help for usage."
                ;;
        esac
    done

    echo ""
    echo "============================================================"
    echo "           NOMAD Tailscale Setup"
    echo "============================================================"
    echo ""

    # Run setup steps
    check_root
    check_internet
    install_tailscale
    configure_service
    authenticate "$authkey"
    configure_firewall
    install_watchdog
    verify_connection
    print_next_steps
}

main "$@"
