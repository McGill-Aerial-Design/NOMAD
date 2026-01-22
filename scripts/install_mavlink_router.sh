#!/bin/bash
# ============================================================
# Install mavlink-router on Jetson
# ============================================================

set -e

echo "Installing mavlink-router build dependencies..."
sudo apt-get update
sudo apt-get install -y git meson ninja-build pkg-config gcc g++ systemd

echo "Cloning mavlink-router source..."
cd /tmp
if [ -d "mavlink-router" ]; then
    rm -rf mavlink-router
fi
git clone https://github.com/mavlink-router/mavlink-router.git
cd mavlink-router

echo "Building mavlink-router..."
meson setup build .
ninja -C build

echo "Installing mavlink-router..."
sudo ninja -C build install

# Create config directory if it doesn't exist
sudo mkdir -p /etc/mavlink-router

echo "Verifying installation..."
which mavlink-routerd
mavlink-routerd --version

echo "Done! You can now start the NOMAD system."
