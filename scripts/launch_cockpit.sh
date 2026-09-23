#!/bin/bash
# ==============================================================================
# RB-KAIROS — Integrated Navigation Cockpit (ROS 2 Jazzy)
# Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
# GitHub: https://github.com/Uncrowned0x0
# ==============================================================================
# Launches in parallel and arranges on screen:
#  1. CANopen Motor Driver (Mecanum Wheels 50Hz)     [Dedicated terminal]
#  2. Dual SICK TiM5xx LiDARs + 3D Robot Model       [Dedicated terminal]
#  3. Intel RealSense D435 Camera                    [Dedicated terminal]
#  4. RViz2 3D Scene (Robot + Front/Rear 360° LiDAR) [GUI Window]
#  5. HD Camera Stream (rqt_image_view)              [GUI Window]
#  6. 48V Battery Monitor (battery_gui)              [GUI Window]
#  7. Control Module (PS4 Gamepad / Keyboard / Autonomous Nav2)
# ==============================================================================

# ANSI Color Codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m'

# Resolve workspace and package directories portably
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/../package.xml" ]; then
    PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
    WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
else
    WS_DIR="${KAIROS_WS:-/home/kairos/kairos_ws}"
    PKG_DIR="$WS_DIR/src/kairos_real_bringup"
fi

if [ ! -f "$WS_DIR/install/setup.bash" ] && [ -f "/home/kairos/kairos_ws/install/setup.bash" ]; then
    WS_DIR="/home/kairos/kairos_ws"
fi

MODE="${1:-2}" # 1: PS4 Gamepad, 2: Keyboard (default), 3: Autonomous Nav2

# Source ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

export DISPLAY="${DISPLAY:-:0}"
export QT_QPA_PLATFORM="xcb"

# Temporary directory for process tracking
PID_DIR="/tmp/kairos_cockpit"
mkdir -p "$PID_DIR"
rm -f "$PID_DIR"/*

# ------------------------------------------------------------------------------
# Hardware Pre-flight Checks
# ------------------------------------------------------------------------------
# 1. CAN Bus
if ! ip link show can0 2>/dev/null | grep -q "state UP"; then
    echo -e "${YELLOW}[CAN] can0 interface is DOWN. Activating at 1 Mbps...${NC}"
    sudo ip link set can0 up type can bitrate 1000000 2>/dev/null
fi

# 2. Static network routes to Ethernet LiDARs
if ! ip route show 2>/dev/null | grep -q "192.168.0.10 dev enp3s0"; then
    sudo ip route add 192.168.0.10 dev enp3s0 2>/dev/null
fi
if ! ip route show 2>/dev/null | grep -q "192.168.0.11 dev enp3s0"; then
    sudo ip route add 192.168.0.11 dev enp3s0 2>/dev/null
fi

# ------------------------------------------------------------------------------
# Window Geometric Layout Calculation based on Display Resolution
# ------------------------------------------------------------------------------
SCREEN_RES=$(xrandr 2>/dev/null | grep '\*' | awk '{print $1}' | head -n1)
SW=$(echo "$SCREEN_RES" | cut -d'x' -f1)
SH=$(echo "$SCREEN_RES" | cut -d'x' -f2)

if [ -z "$SW" ] || [ "$SW" -lt 1200 ]; then
    SW=2560
    SH=1600
fi

if [ "$SW" -ge 2000 ]; then
    # High Resolution Display (e.g. 2560 x 1600)
    RVIZ_X=20;    RVIZ_Y=40;   RVIZ_W=1380; RVIZ_H=940
    RQT_X=1430;   RQT_Y=40;   RQT_W=1100;  RQT_H=650
    BATT_X=1430;  BATT_Y=710; BATT_W=520;   BATT_H=530

    TERM_MOTORS_COLS=58; TERM_MOTORS_ROWS=18; TERM_MOTORS_X=1980; TERM_MOTORS_Y=710
    TERM_LIDAR_COLS=54;  TERM_LIDAR_ROWS=12;  TERM_LIDAR_X=1430;  TERM_LIDAR_Y=1270
    TERM_CAM_COLS=58;    TERM_CAM_ROWS=19;    TERM_CAM_X=1980;    TERM_CAM_Y=1140
    TELEOP_COLS=148;     TELEOP_ROWS=26;      TELEOP_X=20;        TELEOP_Y=1010
else
    # Standard Resolution Display (e.g. 1920 x 1080)
    RVIZ_X=10;    RVIZ_Y=35;   RVIZ_W=1040; RVIZ_H=620
    RQT_X=1070;   RQT_Y=35;   RQT_W=830;   RQT_H=480
    BATT_X=1070;  BATT_Y=535; BATT_W=400;   BATT_H=480

    TERM_MOTORS_COLS=46; TERM_MOTORS_ROWS=23; TERM_MOTORS_X=1490; TERM_MOTORS_Y=535
    TERM_LIDAR_COLS=46;  TERM_LIDAR_ROWS=11;  TERM_LIDAR_X=1070;  TERM_LIDAR_Y=790
    TERM_CAM_COLS=46;    TERM_CAM_ROWS=11;    TERM_CAM_X=1490;    TERM_CAM_Y=790
    TELEOP_COLS=114;     TELEOP_ROWS=17;      TELEOP_X=10;        TELEOP_Y=680
fi

# ------------------------------------------------------------------------------
# Cleanup and Safe Shutdown Function
# ------------------------------------------------------------------------------
cleanup() {
    echo -e "\n${YELLOW}[SHUTDOWN] Closing cockpit and putting robot into safe state...${NC}"

    # 1. Emergency motor stop command (cmd_vel 0)
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}" >/dev/null 2>&1

    # 2. Terminate registered child background processes
    for pidfile in "$PID_DIR"/*.pid; do
        if [ -f "$pidfile" ]; then
            PID=$(cat "$pidfile")
            kill -SIGINT "$PID" 2>/dev/null
            kill -TERM "$PID" 2>/dev/null
        fi
    done

    # 3. Terminate remaining nodes and windows by signature
    pkill -f "sick_tim_driver" 2>/dev/null
    pkill -f "realsense2_camera" 2>/dev/null
    pkill -f "kairos_serial_motor_driver" 2>/dev/null
    pkill -f "vectornav_driver" 2>/dev/null
    pkill -f "dual_laser_merger" 2>/dev/null
    pkill -f "kairos_led_driver" 2>/dev/null
    pkill -f "ekf_node" 2>/dev/null
    pkill -f "nav2" 2>/dev/null
    pkill -f "robot_state_publisher" 2>/dev/null
    pkill -f "joint_state_publisher" 2>/dev/null
    pkill -f "teleop_twist_keyboard" 2>/dev/null
    pkill -f "teleop_twist_joy" 2>/dev/null
    pkill -f "kairos_teleop_joy" 2>/dev/null
    pkill -f "kairos_teleop_keyboard" 2>/dev/null
    pkill -f "rviz2" 2>/dev/null
    pkill -f "rqt_image_view" 2>/dev/null
    pkill -f "battery_gui.py" 2>/dev/null

    rm -rf "$PID_DIR"
    echo -e "${GREEN}[OK] All windows closed and robot secured.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# ------------------------------------------------------------------------------
# Launch Background Drivers & Windows (1 Dedicated Terminal per Driver)
# ------------------------------------------------------------------------------
echo -e "${CYAN}${BOLD}======================================================================${NC}"
echo -e "${CYAN}${BOLD}    RB-KAIROS — INTEGRATED NAVIGATION COCKPIT STARTUP (ROS 2)         ${NC}"
echo -e "${CYAN}${BOLD}======================================================================${NC}"

# 1. Motors & CAN Driver Terminal
echo -e " ${GREEN}[1/6]${NC} Launching CANopen Motor Driver..."
gnome-terminal --title="RB-KAIROS [MOTOR & CAN DRIVER]" \
    --geometry="${TERM_MOTORS_COLS}x${TERM_MOTORS_ROWS}+${TERM_MOTORS_X}+${TERM_MOTORS_Y}" -- bash -c "
    echo \$\$ > $PID_DIR/motors.pid
    source /opt/ros/jazzy/setup.bash
    if [ -f \"$WS_DIR/install/setup.bash\" ]; then source \"$WS_DIR/install/setup.bash\"; fi
    echo -e '\033[1;36m=== CANopen MOTOR DRIVER (50 Hz SYNC) ===\033[0m\n'
    ros2 launch kairos_real_bringup base.launch.py
" &
sleep 1.5

# 2. SICK LiDARs + 3D Robot Model Terminal
echo -e " ${GREEN}[2/6]${NC} Launching SICK TiM5xx LiDAR Drivers (Front/Rear) & 3D Model..."
gnome-terminal --title="RB-KAIROS [SICK LIDARS & 3D ROBOT]" \
    --geometry="${TERM_LIDAR_COLS}x${TERM_LIDAR_ROWS}+${TERM_LIDAR_X}+${TERM_LIDAR_Y}" -- bash -c "
    echo \$\$ > $PID_DIR/lidar.pid
    source /opt/ros/jazzy/setup.bash
    if [ -f \"$WS_DIR/install/setup.bash\" ]; then source \"$WS_DIR/install/setup.bash\"; fi
    echo -e '\033[1;32m=== SICK TiM5xx LIDARS & 3D TF TREE ===\033[0m\n'
    ros2 launch kairos_real_bringup lidar.launch.py
" &
sleep 1.5

# 3. RealSense Camera Driver Terminal
echo -e " ${GREEN}[3/6]${NC} Launching Intel RealSense D435 Camera Driver..."
gnome-terminal --title="RB-KAIROS [REALSENSE D435 CAMERA]" \
    --geometry="${TERM_CAM_COLS}x${TERM_CAM_ROWS}+${TERM_CAM_X}+${TERM_CAM_Y}" -- bash -c "
    echo \$\$ > $PID_DIR/camera.pid
    source /opt/ros/jazzy/setup.bash
    if [ -f \"$WS_DIR/install/setup.bash\" ]; then source \"$WS_DIR/install/setup.bash\"; fi
    echo -e '\033[1;35m=== INTEL REALSENSE D435 CAMERA STREAM ===\033[0m\n'
    ros2 launch kairos_real_bringup camera.launch.py
" &
sleep 1.5

# 4. Battery GUI Dashboard Window
echo -e " ${GREEN}[4/6]${NC} Opening 48V Battery Dashboard..."
BATT_GUI="$PKG_DIR/kairos_real_bringup/battery_gui.py"
if [ ! -f "$BATT_GUI" ]; then
    BATT_GUI="$WS_DIR/src/kairos_real_bringup/kairos_real_bringup/battery_gui.py"
fi
python3 "$BATT_GUI" \
    --geometry="${BATT_W}x${BATT_H}+${BATT_X}+${BATT_Y}" &
P_BATT=$!
echo $P_BATT > "$PID_DIR/batt_gui.pid"

# 5. Visualization Windows (RViz2 + rqt_image_view)
echo -e " ${GREEN}[5/6]${NC} Opening RViz2 3D Scene and Video Stream..."
RVIZ_CFG="$PKG_DIR/config/lidar_view.rviz"
if [ ! -f "$RVIZ_CFG" ]; then
    RVIZ_CFG="$WS_DIR/src/kairos_real_bringup/config/lidar_view.rviz"
fi

rviz2 -d "$RVIZ_CFG" \
    --qwindowgeometry "${RVIZ_W}x${RVIZ_H}+${RVIZ_X}+${RVIZ_Y}" &
P_RVIZ=$!
echo $P_RVIZ > "$PID_DIR/rviz.pid"

ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw &
P_RQT=$!
echo $P_RQT > "$PID_DIR/rqt.pid"

# Background Window Geometry and Layout Enforcer
(
    for i in {1..12}; do
        sleep 0.5
        wmctrl -r "rqt_image_view" -e "0,${RQT_X},${RQT_Y},${RQT_W},${RQT_H}" 2>/dev/null
        wmctrl -r "RViz" -e "0,${RVIZ_X},${RVIZ_Y},${RVIZ_W},${RVIZ_H}" 2>/dev/null
        wmctrl -r "Battery Monitor" -e "0,${BATT_X},${BATT_Y},${BATT_W},${BATT_H}" 2>/dev/null
    done
) &

sleep 1

# 6. Control & Navigation Module (Foreground)
echo -e " ${GREEN}[6/6]${NC} Initializing control module..."

case $MODE in
    1)
        # MODE 1 : PS4 GAMEPAD TELEOPERATION
        echo -e "\n${BOLD}${GREEN}=== ACTIVE COCKPIT: PS4 GAMEPAD TELEOPERATION ===${NC}"
        echo -e "  -> ${BOLD}R1${NC} (Mandatory Deadman): Hold to allow movement (robot halts when released)"
        echo -e "  -> ${BOLD}Left Stick${NC}: Forward/Backward (vx) | Lateral Strafe (vy in Mecanum) or Steering (wz in diff)"
        echo -e "  -> ${BOLD}Right Stick${NC}: In-place analog rotation (omega_z)"
        echo -e "  -> ${BOLD}L1${NC}: Turbo Mode boosted to 0.15 m/s (hold while moving)"
        echo -e "  -> ${BOLD}R2 / L2${NC}: Toggle / Trigger holonomic Mecanum mode"
        echo -e "  -> ${BOLD}D-Pad${NC}: Precision adjustment micro-steps (4 cm/s)"
        echo -e "  -> ${YELLOW}Press Ctrl+C in this window to STOP EVERYTHING.${NC}\n"

        ros2 launch kairos_real_bringup teleop_joy.launch.py
        ;;

    2)
        # MODE 2 : SAFE KEYBOARD TELEOPERATION (AZERTY/QWERTY)
        echo -e "\n${BOLD}${BLUE}=== ACTIVE COCKPIT: SAFE KEYBOARD TELEOPERATION (LAB SAFETY) ===${NC}"
        echo -e "  -> ${GREEN}Initial speed: 0.08 m/s (8 cm/s) | Absolute maximum ceiling: 0.15 m/s${NC}"
        echo -e "  -> ${BOLD}Arrow Keys / ZQSD / WASD / IJKL${NC}: Movement (immediate active brake upon release)"
        echo -e "  -> ${BOLD}+ / -${NC}: Adjust speed (+/- 2 cm/s) WITHOUT moving the robot"
        echo -e "  -> ${BOLD}Spacebar${NC}: Immediate emergency stop"
        echo -e "  -> ${YELLOW}Press Ctrl+C in this window to STOP EVERYTHING.${NC}\n"

        ros2 run kairos_real_bringup kairos_teleop_keyboard
        ;;

    3)
        # MODE 3 : AUTONOMOUS MOBILE BASE (Nav2 Navigation)
        echo -e "\n${BOLD}${CYAN}=== ACTIVE COCKPIT: AUTONOMOUS MODE / NAV2 NAVIGATION ===${NC}"
        echo -e "  -> Active DWB holonomic controller for Mecanum base (vx/vy translations enabled)."
        echo -e "  -> EKF filtered odometry telemetry: ${BOLD}/odometry/filtered${NC}"
        echo -e "  -> Unified 360° blindspot-free scan: ${BOLD}/scan${NC}"
        echo -e "  -> You can send 2D navigation goals (Nav2 Goal) from RViz2."
        echo -e "  -> ${YELLOW}Press Ctrl+C in this window to STOP EVERYTHING.${NC}\n"

        ros2 launch kairos_real_bringup navigation.launch.py
        ;;
esac
