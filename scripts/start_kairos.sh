#!/bin/bash
# ==============================================================================
# RB-KAIROS — Master Startup, Control & Telemetry Script (ROS 2 Jazzy)
# Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
# GitHub: https://github.com/Uncrowned0x0
# ==============================================================================

# ANSI Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m' # No Color

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

# Locate companion scripts
if [ -f "$SCRIPT_DIR/launch_cockpit.sh" ]; then
    COCKPIT_SCRIPT="$SCRIPT_DIR/launch_cockpit.sh"
elif [ -f "/home/kairos/launch_cockpit.sh" ]; then
    COCKPIT_SCRIPT="/home/kairos/launch_cockpit.sh"
else
    COCKPIT_SCRIPT="launch_cockpit.sh"
fi

if [ -f "$SCRIPT_DIR/view_lidar.sh" ]; then
    VIEW_LIDAR_SCRIPT="$SCRIPT_DIR/view_lidar.sh"
elif [ -f "/home/kairos/view_lidar.sh" ]; then
    VIEW_LIDAR_SCRIPT="/home/kairos/view_lidar.sh"
else
    VIEW_LIDAR_SCRIPT="view_lidar.sh"
fi

# Source ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source "/opt/ros/jazzy/setup.bash"
else
    echo -e "${RED}[ERROR] ROS 2 Jazzy is not installed in /opt/ros/jazzy.${NC}"
    exit 1
fi

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo -e "${YELLOW}[WARNING] Workspace $WS_DIR/install/setup.bash not found. Build required.${NC}"
fi

# Cleanup on interruption (Ctrl+C)
cleanup() {
    echo -e "\n${YELLOW}[INFO] Stopping background processes...${NC}"
    kill $(jobs -p) 2>/dev/null
    wait $(jobs -p) 2>/dev/null
    echo -e "${GREEN}[INFO] Mobile base secured. Goodbye!${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM

# Quick CAN bus check
check_can() {
    if ip link show can0 2>/dev/null | grep -q "state UP"; then
        return 0
    else
        return 1
    fi
}

# Automatic CAN bus activation
ensure_can() {
    if ! check_can; then
        echo -e "${YELLOW}[CAN] Interface can0 inactive. Attempting activation at 1 Mbps...${NC}"
        sudo ip link set can0 up type can bitrate 1000000 2>/dev/null
        if check_can; then
            echo -e "${GREEN}[CAN] can0 interface activated successfully.${NC}"
        else
            echo -e "${RED}[CAN] Unable to activate can0 automatically.${NC}"
            echo -e "      Run: sudo ip link set can0 up type can bitrate 1000000"
            read -p "Press Enter to continue anyway..."
        fi
    fi
}

# Check static network routes to SICK LiDARs (enp3s0)
ensure_lidar_routes() {
    if ! ip route show 2>/dev/null | grep -q "192.168.0.10 dev enp3s0"; then
        sudo ip route add 192.168.0.10 dev enp3s0 2>/dev/null
    fi
    if ! ip route show 2>/dev/null | grep -q "192.168.0.11 dev enp3s0"; then
        sudo ip route add 192.168.0.11 dev enp3s0 2>/dev/null
    fi
}

# Quick battery voltage reading from Node 1 (in Volts)
get_quick_voltage() {
    python3 -c "
import can, struct, time
try:
    bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    bus.send(can.Message(arbitration_id=0x601, data=[0x40, 0x0F, 0x20, 0x01, 0, 0, 0, 0], is_extended_id=False))
    t0 = time.time()
    v = 0.0
    while time.time() - t0 < 0.15:
        msg = bus.recv(timeout=0.03)
        if msg and msg.arbitration_id == 0x581 and len(msg.data) >= 8:
            if msg.data[1] == 0x0F and msg.data[2] == 0x20 and msg.data[3] == 0x01:
                raw = struct.unpack('<i', bytes(msg.data[4:8]))[0]
                v = raw / 177.3160173
                break
    bus.shutdown()
    print(f'{v:.2f}')
except Exception:
    print('ERR')
" 2>/dev/null
}

# Display system banner and status
display_header() {
    clear
    echo -e "${CYAN}${BOLD}======================================================================${NC}"
    echo -e "${CYAN}${BOLD}      RB-KAIROS — MASTER CONTROL & TELEMETRY CENTER (ROS 2 JAZZY)     ${NC}"
    echo -e "${CYAN}${BOLD}======================================================================${NC}"

    # CAN Bus Status
    if check_can; then
        CAN_STR="${GREEN}ACTIVE (can0 @ 1 Mbps)${NC}"
    else
        CAN_STR="${RED}INACTIVE (can0 DOWN)${NC}"
    fi

    # Gamepad Status
    if [ -e "/dev/input/js0" ]; then
        JOY_STR="${GREEN}CONNECTED (/dev/input/js0)${NC}"
    else
        JOY_STR="${YELLOW}NOT DETECTED (Bluetooth disconnected)${NC}"
    fi

    # Wi-Fi Status
    WIFI_IP=$(ip -4 addr show wlp4s0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
    if [ -n "$WIFI_IP" ]; then
        NET_STR="${GREEN}$WIFI_IP (Wi-Fi Robotica)${NC}"
    else
        NET_STR="${YELLOW}Not connected to Wi-Fi${NC}"
    fi

    # Battery Telemetry
    V_RAW=$(get_quick_voltage)
    if [ "$V_RAW" == "ERR" ] || [ -z "$V_RAW" ]; then
        BATT_STR="${RED}CAN Read Error${NC}"
    else
        V_FLOAT=$(echo "$V_RAW" | awk '{print $1}')
        IS_POWER_CUT=$(echo "$V_FLOAT < 20.0" | bc -l 2>/dev/null)
        if [ "$IS_POWER_CUT" -eq 1 ]; then
            BATT_STR="${RED}${V_RAW} V — 48V CONTACTOR OPEN (Press blue button)${NC}"
        else
            PCT=$(echo "scale=1; ($V_FLOAT - 44.0) / (53.3 - 44.0) * 100" | bc -l 2>/dev/null)
            PCT_INT=$(printf "%.0f" "$PCT" 2>/dev/null)
            if [ "$PCT_INT" -gt 100 ]; then PCT_INT=100; fi
            if [ "$PCT_INT" -lt 0 ]; then PCT_INT=0; fi

            if [ "$PCT_INT" -ge 40 ]; then
                BATT_STR="${GREEN}${V_RAW} V (~${PCT_INT}%) — 48V ACTIVE${NC}"
            else
                BATT_STR="${YELLOW}${V_RAW} V (~${PCT_INT}%) — LOW BATTERY${NC}"
            fi
        fi
    fi

    # LiDAR Status
    if ping -c 1 -W 1 192.168.0.10 >/dev/null 2>&1; then
        LIDAR_STR="${GREEN}CONNECTED (SICK TiM5xx Front/Rear @ 15 Hz)${NC}"
    else
        LIDAR_STR="${YELLOW}NOT DETECTED (Check 48V power / Ethernet switch)${NC}"
    fi

    echo -e " ${BOLD}CAN Bus:${NC}      $CAN_STR"
    echo -e " ${BOLD}Network:${NC}      $NET_STR"
    echo -e " ${BOLD}LiDARs:${NC}       $LIDAR_STR"
    echo -e " ${BOLD}Gamepad:${NC}      $JOY_STR"
    echo -e " ${BOLD}Battery:${NC}      $BATT_STR"
    echo -e "${CYAN}----------------------------------------------------------------------${NC}"
}

# Live battery gauge in terminal
live_battery_cli() {
    echo -e "\n${BOLD}${CYAN}--- Live 48V Battery Gauge (Press Ctrl+C to exit) ---${NC}\n"
    while true; do
        python3 -c "
import can, struct, time
try:
    bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    voltages = {}
    for n in [1, 2, 3, 4]:
        bus.send(can.Message(arbitration_id=0x600 + n, data=[0x40, 0x0F, 0x20, 0x01, 0, 0, 0, 0], is_extended_id=False))
    t0 = time.time()
    while time.time() - t0 < 0.15:
        msg = bus.recv(timeout=0.03)
        if msg and (0x581 <= msg.arbitration_id <= 0x584) and len(msg.data) >= 8:
            if msg.data[1] == 0x0F and msg.data[2] == 0x20 and msg.data[3] == 0x01:
                n = msg.arbitration_id - 0x580
                raw = struct.unpack('<i', bytes(msg.data[4:8]))[0]
                voltages[n] = raw / 177.3160173
    bus.shutdown()
    v1 = voltages.get(1, 0.0)
    if v1 < 20.0:
        print(f'\r\033[K\033[1;31m[CONTACTOR OPEN]\033[0m Voltage: {v1:.2f} V (Press the blue button)', end='', flush=True)
    else:
        pct = max(0.0, min(100.0, (v1 - 44.0) / (53.3 - 44.0) * 100.0))
        bars = int(pct / 5)
        gauge = '█' * bars + '░' * (20 - bars)
        col = '\033[1;32m' if pct > 40 else '\033[1;33m'
        n_str = ' '.join([f'N{n}:{voltages.get(n, 0.0):.1f}V' for n in [1,2,3,4]])
        print(f'\r\033[K{col}[{gauge}] {pct:5.1f}%\033[0m | \033[1m{v1:.2f} V\033[0m | {n_str}', end='', flush=True)
except Exception as e:
    print(f'\r\033[K\033[31mRead error: {e}\033[0m', end='', flush=True)
"
        sleep 0.8
    done
}

# Low-level AMC motor drives diagnostics
run_can_diagnostics() {
    echo -e "\n${BOLD}${CYAN}--- Diagnostics for 4 AMC CANopen Motor Drives ---${NC}\n"
    python3 -c "
import can, struct, time

nodes = [1, 2, 3, 4]
names = {1: 'Front-Left (FL)', 2: 'Rear-Left (BL)', 3: 'Front-Right (FR)', 4: 'Rear-Right (BR)'}

try:
    bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    print(f'{\"Node\":<8} | {\"Wheel\":<22} | {\"Bus Voltage\":<14} | {\"Statusword\":<12} | {\"DS402 State\":<18}')
    print('-' * 82)

    for n in nodes:
        # Read DC Bus Voltage (0x200F:01)
        bus.send(can.Message(arbitration_id=0x600 + n, data=[0x40, 0x0F, 0x20, 0x01, 0, 0, 0, 0], is_extended_id=False))
        # Read Statusword (0x6041:00)
        bus.send(can.Message(arbitration_id=0x600 + n, data=[0x40, 0x41, 0x60, 0x00, 0, 0, 0, 0], is_extended_id=False))
        
        t0 = time.time()
        volt = None
        sw = None
        while time.time() - t0 < 0.2:
            resp = bus.recv(timeout=0.03)
            if resp and resp.arbitration_id == 0x580 + n and len(resp.data) >= 8:
                idx = (resp.data[2] << 8) | resp.data[1]
                sub = resp.data[3]
                if idx == 0x200F and sub == 0x01:
                    raw_v = struct.unpack('<i', bytes(resp.data[4:8]))[0]
                    volt = raw_v / 177.3160173
                elif idx == 0x6041:
                    sw = struct.unpack('<H', bytes(resp.data[4:6]))[0]

        v_str = f'{volt:.2f} V' if volt is not None else 'TIMEOUT'
        sw_str = f'0x{sw:04X}' if sw is not None else 'TIMEOUT'
        
        if sw is not None:
            if (sw & 0x006F) == 0x0027:
                st_str = '\033[32mOperation Enabled\033[0m'
            elif (sw & 0x004F) == 0x0040:
                st_str = '\033[33mSwitch On Disabled\033[0m'
            elif (sw & 0x0008) == 0x0008:
                st_str = '\033[31mFAULT (Trip)\033[0m'
            else:
                st_str = f'State 0x{sw:04X}'
        else:
            st_str = '\033[31mNO RESPONSE\033[0m'

        print(f'Node {n:<3} | {names[n]:<22} | {v_str:<14} | {sw_str:<12} | {st_str}')
    bus.shutdown()
except Exception as e:
    print(f'CAN Error: {e}')
"
    echo ""
    read -p "Press Enter to return to main menu..."
}

# MAIN MENU LOOP
while true; do
    ensure_can
    ensure_lidar_routes
    display_header

    echo -e " ${BOLD}Navigation Modes (Full Cockpit: Motors + 3D LiDAR + Camera + Battery):${NC}"
    echo -e "  ${GREEN}[1]${NC} 🎮  ${BOLD}PS4 Gamepad Cockpit${NC} (DualShock 4 Bluetooth + Full Telemetry)"
    echo -e "  ${BLUE}[2]${NC} ⌨️   ${BOLD}Interactive Keyboard Cockpit${NC} (Holonomic Control + Full Telemetry)"
    echo -e "  ${CYAN}[3]${NC} 🤖  ${BOLD}Autonomous / Nav2 Cockpit${NC} (Listens to /cmd_vel + Full Telemetry)"
    echo ""
    echo -e " ${BOLD}Individual Telemetry & Tools (Unit Testing):${NC}"
    echo -e "  ${YELLOW}[4]${NC} 🔋  ${BOLD}Live Battery Gauge (CLI Terminal)${NC}"
    echo -e "  ${MAGENTA}[5]${NC} 📊  ${BOLD}Visual Battery Monitor Window (GUI Window)${NC}"
    echo -e "  ${CYAN}[6]${NC} 🔍  ${BOLD}CAN Diagnostics & 4 AMC Drives State${NC}"
    echo -e "  ${GREEN}[7]${NC} 📷  ${BOLD}RealSense D435 Camera Only (Color & Depth Stream)${NC}"
    echo -e "  ${CYAN}[8]${NC} 📡  ${BOLD}360° LiDAR Radar Only (CLI Terminal)${NC}"
    echo -e "  ${BLUE}[9]${NC} 🗺️   ${BOLD}LiDAR Visualization Only (RViz2)${NC}"
    echo -e "  ${RED}[q]${NC} 🚪  Quit"
    echo ""
    read -p "Your choice [1-9, q]: " CHOICE

    case $CHOICE in
        1)
            # Check gamepad
            if [ ! -e "/dev/input/js0" ]; then
                echo -e "\n${YELLOW}[WARNING] No gamepad detected on /dev/input/js0.${NC}"
                echo -e "Ensure PS4 gamepad is powered on (Press PS button)."
                echo -e "If not paired, hold SHARE + PS until rapid white blinking."
                read -p "Do you want to launch the cockpit anyway? [y/N]: " REPLY
                if [[ ! "$REPLY" =~ ^[yY]$ ]]; then
                    continue
                fi
            fi
            bash "$COCKPIT_SCRIPT" 1
            ;;

        2)
            bash "$COCKPIT_SCRIPT" 2
            ;;

        3)
            bash "$COCKPIT_SCRIPT" 3
            ;;

        4)
            live_battery_cli
            ;;

        5)
            echo -e "\n${MAGENTA}[LAUNCH] Opening Battery GUI Window...${NC}"
            if [ -n "$DISPLAY" ]; then
                BATT_GUI="$PKG_DIR/kairos_real_bringup/battery_gui.py"
                if [ ! -f "$BATT_GUI" ]; then
                    BATT_GUI="$WS_DIR/src/kairos_real_bringup/kairos_real_bringup/battery_gui.py"
                fi
                python3 "$BATT_GUI" &
                echo -e "${GREEN}[OK] Window opened on your Remmina/RDP desktop.${NC}"
                sleep 1
            else
                echo -e "${RED}[ERROR] DISPLAY environment variable not set. Connect via RDP.${NC}"
                read -p "Press Enter to continue..."
            fi
            ;;

        6)
            run_can_diagnostics
            ;;

        7)
            echo -e "\n${GREEN}[LAUNCH] Starting RealSense D435 Camera Stream...${NC}"
            if [ -z "$DISPLAY" ]; then
                echo -e "${RED}[ERROR] DISPLAY environment variable not set. Connect via RDP.${NC}"
                read -p "Press Enter to continue..."
                continue
            fi
            if ! ros2 node list 2>/dev/null | grep -q "camera"; then
                echo -e "  -> Launching RealSense ROS 2 driver..."
                ros2 launch kairos_real_bringup camera.launch.py &
                PID_CAM=$!
                sleep 3
            else
                echo -e "  -> RealSense driver is already active."
                PID_CAM=""
            fi
            echo -e "  -> Opening rqt_image_view on /camera/camera/color/image_raw..."
            ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw
            if [ -n "$PID_CAM" ]; then
                echo -e "\n${YELLOW}[INFO] Stopping camera driver...${NC}"
                kill -SIGINT $PID_CAM 2>/dev/null
                pkill -f "realsense2_camera" 2>/dev/null
            fi
            ;;

        8)
            echo -e "\n${CYAN}[LAUNCH] Starting 360° LiDAR Radar in Console...${NC}"
            if ! ros2 topic list 2>/dev/null | grep -q "/front_laser/scan"; then
                echo -e "  -> Launching SICK TiM5xx driver..."
                ros2 launch kairos_real_bringup lidar.launch.py &
                PID_LIDAR=$!
                sleep 2
            else
                PID_LIDAR=""
            fi
            ros2 run kairos_real_bringup lidar_radar_cli
            if [ -n "$PID_LIDAR" ]; then
                echo -e "\n${YELLOW}[INFO] Stopping LiDAR driver...${NC}"
                kill -SIGINT $PID_LIDAR 2>/dev/null
                pkill -f "sick_tim_driver" 2>/dev/null
                pkill -f "robot_state_publisher" 2>/dev/null
                pkill -f "joint_state_publisher" 2>/dev/null
            fi
            ;;

        9)
            echo -e "\n${BLUE}[LAUNCH] 3D LiDAR Visualization in RViz2...${NC}"
            if [ -z "$DISPLAY" ]; then
                echo -e "${RED}[ERROR] DISPLAY environment variable not set. Connect via RDP.${NC}"
                read -p "Press Enter to continue..."
                continue
            fi
            bash "$VIEW_LIDAR_SCRIPT"
            ;;

        q|Q)
            echo -e "\n${GREEN}Closing RB-KAIROS control center. Goodbye!${NC}"
            exit 0
            ;;

        *)
            echo -e "${RED}Invalid option.${NC}"
            sleep 1
            ;;
    esac
done
