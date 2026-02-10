#!/bin/bash
# ==============================================================================
# DEPRECATED: Use fix_power_mode_25w_v2.sh instead.
# That script is the JetPack 6.2 (L4T 36.4.7) compatible version with
# corrected GPU power gating paths.
# ==============================================================================
#
# Add 25W MAXN Power Mode to Jetson Orin Nano
# ==============================================================================
# This script adds the missing 25W power mode to nvpmodel on Jetson Orin Nano.
# The default JetPack 6.2 config only includes 15W and 7W modes.
#
# Run this on the Jetson:
#   chmod +x fix_power_mode_25w.sh
#   sudo ./fix_power_mode_25w.sh
# ==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}  Adding 25W MAXN Power Mode${NC}"
echo -e "${GREEN}================================================${NC}"

NVPMODEL_CONF="/etc/nvpmodel.conf"
BACKUP_FILE="/etc/nvpmodel.conf.backup.$(date +%Y%m%d_%H%M%S)"

# Check if we're root
if [ "$EUID" -ne 0 ]; then 
    echo -e "${RED}Please run as root: sudo $0${NC}"
    exit 1
fi

# Backup original config
echo -e "\n${YELLOW}Creating backup: $BACKUP_FILE${NC}"
cp "$NVPMODEL_CONF" "$BACKUP_FILE"

# Check if 25W mode already exists
if grep -q "NAME=25W" "$NVPMODEL_CONF"; then
    echo -e "${GREEN}25W mode already exists!${NC}"
    nvpmodel -q
    exit 0
fi

echo -e "\n${YELLOW}Adding 25W MAXN power mode...${NC}"

# Remove the last line (PM_CONFIG DEFAULT)
sed -i '/< PM_CONFIG DEFAULT/d' "$NVPMODEL_CONF"

# Add 25W power mode before PM_CONFIG
cat >> "$NVPMODEL_CONF" << 'EOF'

< POWER_MODEL ID=2 NAME=25W >
CPU_ONLINE CORE_0 1
CPU_ONLINE CORE_1 1
CPU_ONLINE CORE_2 1
CPU_ONLINE CORE_3 1
CPU_ONLINE CORE_4 1
CPU_ONLINE CORE_5 1
FBP_POWER_GATING FBP_PG_MASK 0
TPC_POWER_GATING TPC_PG_MASK 0
GPU_POWER_CONTROL_ENABLE GPU_PWR_CNTL_EN on
CPU_A78_0 MIN_FREQ 729600
CPU_A78_0 MAX_FREQ -1
CPU_A78_1 MIN_FREQ 729600
CPU_A78_1 MAX_FREQ -1
GPU MIN_FREQ 0
GPU MAX_FREQ -1
GPU_POWER_CONTROL_DISABLE GPU_PWR_CNTL_DIS auto
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

echo -e "\n${GREEN}================================================${NC}"
echo -e "${GREEN}  Setup Complete!${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""
echo "Available power modes:"
echo "  Mode 0: 15W"
echo "  Mode 1: 7W"
echo "  Mode 2: 25W (MAXN) - Currently Active"
echo ""
echo "To switch modes manually:"
echo "  sudo nvpmodel -m 0  # 15W mode"
echo "  sudo nvpmodel -m 1  # 7W mode"
echo "  sudo nvpmodel -m 2  # 25W mode"
echo ""
echo "Backup saved at: $BACKUP_FILE"
echo ""

