#!/bin/bash
# ==============================================================================
# RB-KAIROS — SICK TiM5xx LiDAR Visualization (RViz2)
# Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
# GitHub: https://github.com/Uncrowned0x0
# ==============================================================================

# Resolve workspace and package directories portably
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/../package.xml" ]; then
    PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
    WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
else
    WS_DIR="${KAIROS_WS:-/home/kairos/kairos_ws}"
    PKG_DIR="$WS_DIR/src/kairos_real_bringup"
fi

source /opt/ros/jazzy/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
elif [ -f "/home/kairos/kairos_ws/install/setup.bash" ]; then
    source "/home/kairos/kairos_ws/install/setup.bash"
fi

# Ensure direct Ethernet routes to LiDAR scanners
if ! ip route show | grep -q "192.168.0.10 dev enp3s0"; then
    sudo ip route add 192.168.0.10 dev enp3s0 2>/dev/null
fi
if ! ip route show | grep -q "192.168.0.11 dev enp3s0"; then
    sudo ip route add 192.168.0.11 dev enp3s0 2>/dev/null
fi

# Check if LiDAR drivers are already running
if ! ros2 topic list 2>/dev/null | grep -q "/front_laser/scan"; then
    echo -e "\033[1;36m[INFO] Starting SICK TiM5xx LiDAR drivers (Front + Rear)...\033[0m"
    ros2 launch kairos_real_bringup lidar.launch.py &
    PID_LIDAR=$!
    sleep 2
else
    echo -e "\033[1;32m[INFO] LiDAR drivers are already active.\033[0m"
    PID_LIDAR=""
fi

# Locate RViz config file portably
RVIZ_CFG="$PKG_DIR/config/lidar_view.rviz"
if [ ! -f "$RVIZ_CFG" ]; then
    RVIZ_CFG="$WS_DIR/src/kairos_real_bringup/config/lidar_view.rviz"
fi

# Launch RViz2 with pre-configured configuration
echo -e "\033[1;36m[INFO] Opening RViz2 (Front/Rear Laser View)...\033[0m"
rviz2 -d "$RVIZ_CFG"

if [ -n "$PID_LIDAR" ]; then
    echo -e "\033[1;33m[INFO] Stopping LiDAR drivers...\033[0m"
    kill -SIGINT $PID_LIDAR 2>/dev/null
    pkill -f "sick_tim_driver" 2>/dev/null
    pkill -f "robot_state_publisher" 2>/dev/null
    pkill -f "joint_state_publisher" 2>/dev/null
fi
