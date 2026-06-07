#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# ==============================================================================
# Add 25W MAXN Power Mode to Jetson Orin Nano (JetPack 6.2 Compatible)
# ==============================================================================
# Fixed version for JetPack 6.2 (L4T 36.4.7) with correct GPU paths
# ==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}  Adding 25W MAXN Power Mode (JetPack 6.2)${NC}"
echo -e "${GREEN}================================================${NC}"

NVPMODEL_CONF="/etc/nvpmodel.conf"
BACKUP_FILE="/etc/nvpmodel.conf.backup.$(date +%Y%m%d_%H%M%S)"

if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Please run as root: sudo $0${NC}"
    exit 1
fi

echo -e "\n${YELLOW}Creating backup: $BACKUP_FILE${NC}"
cp "$NVPMODEL_CONF" "$BACKUP_FILE"

# Remove any existing 25W mode and PM_CONFIG DEFAULT
sed -i '/< POWER_MODEL ID=2 NAME=25W >/,/^$/d' "$NVPMODEL_CONF"
sed -i '/< PM_CONFIG DEFAULT/d' "$NVPMODEL_CONF"

echo -e "\n${YELLOW}Adding 25W MAXN power mode (corrected paths)...${NC}"

# Add 25W power mode - using minimal power gating settings for compatibility
cat >> "$NVPMODEL_CONF" << 'EOF'

< POWER_MODEL ID=2 NAME=25W >
CPU_ONLINE CORE_0 1
CPU_ONLINE CORE_1 1
CPU_ONLINE CORE_2 1
CPU_ONLINE CORE_3 1
CPU_ONLINE CORE_4 1
CPU_ONLINE CORE_5 1
CPU_A78_0 MIN_FREQ 729600
CPU_A78_0 MAX_FREQ -1
CPU_A78_1 MIN_FREQ 729600
CPU_A78_1 MAX_FREQ -1
GPU MIN_FREQ 0
GPU MAX_FREQ -1
EMC MAX_FREQ -1

# mandatory section to configure the default power mode
< PM_CONFIG DEFAULT=2 >
EOF

echo -e "${GREEN}25W mode added successfully!${NC}"

# Set to 25W mode
echo -e "\n${YELLOW}Setting power mode to 25W...${NC}"
/usr/sbin/nvpmodel -m 2

# Verify
echo -e "\n${GREEN}Current power mode:${NC}"
/usr/sbin/nvpmodel -q

# Show CPU frequencies
echo -e "\n${GREEN}CPU Frequency info:${NC}"
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null && echo " Hz (Core 0)" || true
cat /sys/devices/system/cpu/cpu4/cpufreq/scaling_cur_freq 2>/dev/null && echo " Hz (Core 4)" || true

echo -e "\n${GREEN}================================================${NC}"
echo -e "${GREEN}  Setup Complete!${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""
echo "Available power modes:"
echo "  Mode 0: 15W"
echo "  Mode 1: 7W"
echo "  Mode 2: 25W (MAXN) - Currently Active"
echo ""
echo "To switch modes:"
echo "  sudo nvpmodel -m 0  # 15W mode"
echo "  sudo nvpmodel -m 1  # 7W mode"
echo "  sudo nvpmodel -m 2  # 25W mode"
echo ""
echo "Then enable max CPU frequency:"
echo "  sudo jetson_clocks"
echo ""
echo "Backup saved at: $BACKUP_FILE"
echo ""
