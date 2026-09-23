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
# 4. Configure SocketCAN can0 Interface (Persistent via systemd)
# ------------------------------------------------------------------------------
echo -e "\n${BOLD}${GREEN}[4/5]${NC} ${BOLD}Configuring SocketCAN interface (can0 @ 1,000,000 baud)...${NC}"
if [ -f "$SCRIPT_DIR/kairos-can.service" ]; then
    $SUDO cp "$SCRIPT_DIR/kairos-can.service" /etc/systemd/system/
    $SUDO systemctl daemon-reload
    $SUDO systemctl enable kairos-can.service
    $SUDO systemctl start kairos-can.service || true
    echo -e "      ${GREEN}[OK] Systemd service kairos-can.service installed and enabled.${NC}"
else
    echo -e "      ${RED}[ERROR] kairos-can.service not found in ${SCRIPT_DIR}!${NC}"
fi

# ------------------------------------------------------------------------------
# 5. Static Network Routing for SICK LiDARs (Persistent via systemd)
# ------------------------------------------------------------------------------
echo -e "\n${BOLD}${GREEN}[5/5]${NC} ${BOLD}Configuring static Ethernet routing for SICK LiDARs (enp3s0)...${NC}"
if [ -f "$SCRIPT_DIR/kairos-network.service" ]; then
    $SUDO cp "$SCRIPT_DIR/kairos-network.service" /etc/systemd/system/
    $SUDO systemctl daemon-reload
    $SUDO systemctl enable kairos-network.service
    $SUDO systemctl start kairos-network.service || true
    echo -e "      ${GREEN}[OK] Systemd service kairos-network.service installed and enabled.${NC}"
else
    echo -e "      ${RED}[ERROR] kairos-network.service not found in ${SCRIPT_DIR}!${NC}"
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
