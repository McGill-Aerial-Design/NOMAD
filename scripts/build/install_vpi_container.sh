#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Install VPI and nvblox dependencies inside the Isaac ROS container
set -e

echo "=== Adding NVIDIA L4T apt repo ==="
echo "deb https://repo.download.nvidia.com/jetson/common r36.4 main" > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
echo "deb https://repo.download.nvidia.com/jetson/t234 r36.4 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
apt-key adv --fetch-keys https://repo.download.nvidia.com/jetson/jetson-ota-public.asc 2>&1 | tail -3

echo "=== Updating apt ==="
apt-get update 2>&1 | tail -10

echo "=== Installing VPI ==="
apt-get install -y --no-install-recommends vpi3-dev libnvvpi3 2>&1 | tail -10

echo "=== Verifying VPI ==="
find / -name 'vpi-config.cmake' 2>/dev/null
echo "=== Done ==="
