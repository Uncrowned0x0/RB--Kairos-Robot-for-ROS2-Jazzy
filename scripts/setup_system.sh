#!/bin/bash
# ==============================================================================
# RB-KAIROS — Automated System Setup & Hardware Permissions Configuration
# Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
# GitHub: https://github.com/Uncrowned0x0
# ==============================================================================
# This script performs all required low-level system configuration for RB-KAIROS:
#  1. Installs udev rules for persistent IMU & LED USB symlinks
#  2. Reloads and triggers the udev subsystem
#  3. Adds the target user to dialout and tty groups (non-root serial access)
#  4. Tests and configures the SocketCAN interface (can0 @ 1,000,000 baud)
#  5. Configures static IP routing for dual SICK TiM5xx LiDARs on enp3s0
# ==============================================================================

set -e

# ANSI Color Codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

echo -e "${CYAN}${BOLD}======================================================================${NC}"
echo -e "${CYAN}${BOLD}     RB-KAIROS — SYSTEM SETUP & HARDWARE CONFIGURATION                ${NC}"
echo -e "${CYAN}${BOLD}======================================================================${NC}"

# Determine sudo usage and target user
if [ "$EUID" -ne 0 ]; then
    SUDO="sudo"
else
    SUDO=""
fi
TARGET_USER="${SUDO_USER:-$USER}"

# Resolve script directory and config source
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/../config/51-kairos-devices.rules" ]; then
    RULES_SRC="$SCRIPT_DIR/../config/51-kairos-devices.rules"
elif [ -f "/home/kairos/kairos_ws/src/kairos_real_bringup/config/51-kairos-devices.rules" ]; then
    RULES_SRC="/home/kairos/kairos_ws/src/kairos_real_bringup/config/51-kairos-devices.rules"
else
    echo -e "${RED}[ERROR] Source udev rules file 51-kairos-devices.rules not found!${NC}"
    exit 1
fi

# ------------------------------------------------------------------------------
# 1. Install Udev Rules
# ------------------------------------------------------------------------------
echo -e "\n${BOLD}${GREEN}[1/5]${NC} ${BOLD}Installing udev rules (IMU & LED USB symlinks)...${NC}"
$SUDO cp "$RULES_SRC" /etc/udev/rules.d/51-kairos-devices.rules
$SUDO chmod 644 /etc/udev/rules.d/51-kairos-devices.rules
echo -e "      ${GREEN}[OK] Installed /etc/udev/rules.d/51-kairos-devices.rules${NC}"

# ------------------------------------------------------------------------------
# 2. Reload and Trigger Udev Rules
# ------------------------------------------------------------------------------
echo -e "\n${BOLD}${GREEN}[2/5]${NC} ${BOLD}Reloading and triggering udev subsystem...${NC}"
$SUDO udevadm control --reload-rules
$SUDO udevadm trigger
echo -e "      ${GREEN}[OK] Udev rules reloaded and triggered.${NC}"

# ------------------------------------------------------------------------------
# 3. Add User to Required Groups
# ------------------------------------------------------------------------------
echo -e "\n${BOLD}${GREEN}[3/5]${NC} ${BOLD}Granting hardware group permissions to user: ${CYAN}${TARGET_USER}${NC}..."
$SUDO usermod -a -G dialout,tty "$TARGET_USER"
echo -e "      ${GREEN}[OK] Added '${TARGET_USER}' to groups 'dialout' and 'tty'.${NC}"

# ------------------------------------------------------------------------------
# 4. Configure SocketCAN can0 Interface
# ------------------------------------------------------------------------------
echo -e "\n${BOLD}${GREEN}[4/5]${NC} ${BOLD}Configuring SocketCAN interface (can0 @ 1,000,000 baud)...${NC}"
# Load kernel modules if available
$SUDO modprobe can 2>/dev/null || true
$SUDO modprobe can_raw 2>/dev/null || true
$SUDO modprobe can_dev 2>/dev/null || true

if ip link show can0 >/dev/null 2>&1; then
    $SUDO ip link set can0 down 2>/dev/null || true
    $SUDO ip link set can0 type can bitrate 1000000
    $SUDO ip link set can0 up
    if ip link show can0 2>/dev/null | grep -q "state UP"; then
        echo -e "      ${GREEN}[OK] can0 is UP and active at 1,000,000 baud (1 Mbps).${NC}"
    else
        echo -e "      ${YELLOW}[INFO] can0 configured at 1 Mbps. Link will turn UP when bus transceivers are powered.${NC}"
    fi
else
    echo -e "      ${YELLOW}[WARNING] Interface can0 not found. Connect the CAN-to-USB/PCIe interface and rerun.${NC}"
fi

# ------------------------------------------------------------------------------
# 5. Static Network Routing for SICK LiDARs
# ------------------------------------------------------------------------------
echo -e "\n${BOLD}${GREEN}[5/5]${NC} ${BOLD}Configuring static Ethernet routing for SICK LiDARs (enp3s0)...${NC}"
if ip link show enp3s0 >/dev/null 2>&1; then
    $SUDO ip link set enp3s0 up 2>/dev/null || true
    $SUDO ip route replace 192.168.0.10 dev enp3s0 2>/dev/null || $SUDO ip route add 192.168.0.10 dev enp3s0 2>/dev/null
    $SUDO ip route replace 192.168.0.11 dev enp3s0 2>/dev/null || $SUDO ip route add 192.168.0.11 dev enp3s0 2>/dev/null
    echo -e "      ${GREEN}[OK] Route 192.168.0.10 dev enp3s0 established.${NC}"
    echo -e "      ${GREEN}[OK] Route 192.168.0.11 dev enp3s0 established.${NC}"
else
    echo -e "      ${YELLOW}[WARNING] Ethernet interface enp3s0 not found.${NC}"
fi

# ------------------------------------------------------------------------------
# Summary & Next Steps
# ------------------------------------------------------------------------------
echo -e "\n${CYAN}${BOLD}======================================================================${NC}"
echo -e "${GREEN}${BOLD}             SYSTEM CONFIGURATION COMPLETED SUCCESSFULLY              ${NC}"
echo -e "${CYAN}${BOLD}======================================================================${NC}"
echo -e " ${BOLD}Summary:${NC}"
echo -e "  • Udev rules:     /etc/udev/rules.d/51-kairos-devices.rules (Active)"
echo -e "  • User groups:    ${TARGET_USER} -> dialout, tty"
echo -e "  • SocketCAN:      can0 @ 1,000,000 baud"
echo -e "  • SICK LiDARs:    192.168.0.10 & 192.168.0.11 via enp3s0"
echo ""
echo -e " ${YELLOW}${BOLD}Note on Group Permissions:${NC}"
echo -e "  If you just added your user to the dialout/tty groups, log out and"
echo -e "  log back in, or run '${BOLD}newgrp dialout${NC}' in your current terminal"
echo -e "  session for permissions to apply immediately."
echo -e "${CYAN}======================================================================${NC}\n"
