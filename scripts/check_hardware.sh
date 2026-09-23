#!/bin/bash
# ==============================================================================
# RB-KAIROS — Hardware Diagnostic Script
# ==============================================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

echo -e "Starting hardware diagnostic..."

# 1. SocketCAN
if ip link show can0 2>/dev/null | grep -q "state UP"; then
    if ip -details link show can0 | grep -q "bitrate 1000000"; then
        echo -e "SocketCAN can0: ${GREEN}PASS${NC}"
    else
        echo -e "SocketCAN can0: ${RED}FAIL (Bitrate is not 1000000)${NC}"
    fi
else
    echo -e "SocketCAN can0: ${RED}FAIL (Interface not UP or not found)${NC}"
fi

# 2. IMU VectorNav
if [ -e "/dev/ttyUSB_IMU" ]; then
    echo -e "IMU VectorNav (/dev/ttyUSB_IMU): ${GREEN}PASS${NC}"
else
    echo -e "IMU VectorNav (/dev/ttyUSB_IMU): ${RED}FAIL${NC}"
fi

# 3. LEDs Teensy
if [ -e "/dev/ttyUSB_LEDS" ]; then
    echo -e "LEDs Teensy (/dev/ttyUSB_LEDS): ${GREEN}PASS${NC}"
else
    echo -e "LEDs Teensy (/dev/ttyUSB_LEDS): ${RED}FAIL${NC}"
fi

# 4. LiDAR Front (192.168.0.10:2111)
if timeout 1 bash -c '</dev/tcp/192.168.0.10/2111' 2>/dev/null; then
    echo -e "LiDAR Front (192.168.0.10:2111): ${GREEN}PASS${NC}"
else
    echo -e "LiDAR Front (192.168.0.10:2111): ${RED}FAIL${NC}"
fi

# 5. LiDAR Rear (192.168.0.11:2111)
if timeout 1 bash -c '</dev/tcp/192.168.0.11/2111' 2>/dev/null; then
    echo -e "LiDAR Rear (192.168.0.11:2111): ${GREEN}PASS${NC}"
else
    echo -e "LiDAR Rear (192.168.0.11:2111): ${RED}FAIL${NC}"
fi

echo -e "Diagnostic complete."
