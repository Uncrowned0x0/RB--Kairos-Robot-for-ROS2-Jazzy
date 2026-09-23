#!/bin/bash
# ==============================================================================
# RB-KAIROS — Intel RealSense D435 Camera View
# Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
# GitHub: https://github.com/Uncrowned0x0
# ==============================================================================

# Resolve workspace directory portably
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/../package.xml" ]; then
    WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
else
    WS_DIR="${KAIROS_WS:-/home/kairos/kairos_ws}"
fi

source /opt/ros/jazzy/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
elif [ -f "/home/kairos/kairos_ws/install/setup.bash" ]; then
    source "/home/kairos/kairos_ws/install/setup.bash"
fi

# Check if the RealSense driver node is already running
if ! ros2 node list 2>/dev/null | grep -q "camera"; then
    echo -e "\033[1;36m[INFO] Starting RealSense D435 driver (RGB + Depth)...\033[0m"
    ros2 launch kairos_real_bringup camera.launch.py &
    PID_CAM=$!
    sleep 3
else
    echo -e "\033[1;32m[INFO] RealSense driver is already active.\033[0m"
    PID_CAM=""
fi

echo -e "\033[1;36m[INFO] Opening rqt_image_view on /camera/camera/color/image_raw...\033[0m"
ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw

if [ -n "$PID_CAM" ]; then
    echo -e "\033[1;33m[INFO] Stopping camera driver...\033[0m"
    kill -SIGINT $PID_CAM 2>/dev/null
    pkill -f "realsense2_camera" 2>/dev/null
fi
