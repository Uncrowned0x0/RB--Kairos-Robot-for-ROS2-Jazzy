# RB-KAIROS Automation Scripts & System Tools Guide

This directory contains the operational shell scripts, system configuration utilities, and standalone visualization tools for the **RB-KAIROS Omnidirectional Mobile Base (ROS 2 Jazzy / Ubuntu 24.04 LTS)**.

---

## 📑 Table of Contents

1. [Overview & Architecture](#-overview--architecture)
2. [Prerequisites & Initial Setup (`setup_system.sh`)](#-1-initial-setup-setupsystemsh)
3. [Master Startup & Telemetry Menu (`start_kairos.sh`)](#-2-master-control-center-startkairossh)
4. [Integrated Navigation Cockpit (`launch_cockpit.sh`)](#-3-integrated-navigation-cockpit-launchcockpitsh)
5. [Standalone RealSense Camera Stream (`view_camera.sh`)](#-4-standalone-camera-viewer-viewcamerash)
6. [Standalone Dual SICK LiDAR Viewer (`view_lidar.sh`)](#-5-standalone-lidar-viewer-viewlidarsh)
7. [Workspace Dependencies Import (`kairos_dependencies.repos`)](#-6-workspace-dependencies-import-kairosdependenciesrepos)
8. [Quick Command Reference & Cheat Sheet](#-7-quick-command-reference)

---

## 🧭 Overview & Architecture

The scripts are organized to provide a modular, safe, and intuitive workflow from cold boot to high-level autonomous navigation:

```
kairos_real_bringup/
├── kairos_dependencies.repos       # VCS repo file to import all workspace dependencies
├── README.md                       # Main repository overview & documentation index
├── README_script.md                # Symlink to scripts/README_script.md
└── scripts/
    ├── README_script.md            # This documentation guide
    ├── setup_system.sh             # [Root/Sudo] One-time hardware & network permissions setup
    ├── start_kairos.sh             # Interactive terminal control center & live telemetry
    ├── launch_cockpit.sh           # Arranges parallel drivers + GUI windows + active teleop
    ├── view_camera.sh              # Standalone camera driver + rqt_image_view viewer
    └── view_lidar.sh               # Standalone dual LiDAR driver + RViz2 viewer
```

All scripts feature:
- **Portable path resolution**: Automatically detect whether they are executed from inside the package directory, the workspace root, or installed system paths.
- **Graceful termination**: Clean shutdown handlers (`trap cleanup SIGINT SIGTERM EXIT`) that issue zero-velocity halt commands (`cmd_vel`) and safely terminate child background processes.
- **Fail-safe validation**: Automatic pre-flight verification of the CAN bus interface, Ethernet LiDAR routing, and hardware connections.

---

## 🛠️ 1. Initial Setup: `setup_system.sh`

### Purpose
An automated, one-time system configuration script that configures Linux kernel modules, udev rules for serial/USB hardware, user group permissions, SocketCAN networking, and static Ethernet routes.

### What it Does
1. **Installs Udev Rules**: Copies `config/51-kairos-devices.rules` to `/etc/udev/rules.d/51-kairos-devices.rules` to create deterministic symlinks:
   - `/dev/ttyUSB_IMU` (VectorNav VN-100 IMU, FTDI serial `FTBWZI3U`)
   - `/dev/ttyUSB_LEDS` (Teensy 3.2 LED Controller, USB ID `16c0:0483`)
2. **Reloads Udev Subsystem**: Executes `udevadm control --reload-rules && udevadm trigger` to activate device rules without rebooting.
3. **Configures Group Permissions**: Adds the target user to `dialout` and `tty` groups, allowing non-root serial communication with IMU and microcontroller devices.
4. **Initializes SocketCAN (`can0`)**:
   - Loads kernel modules: `can`, `can_raw`, `can_dev`.
   - Configures `can0` interface to **1,000,000 baud (1 Mbps)** bitrate required by the Advanced Motion Controls (AMC) motor drives.
   - Sets interface state to `UP`.
5. **Sets Static Routes for SICK LiDARs**: Adds kernel network routes for Ethernet interface `enp3s0`:
   - `192.168.0.10` (Front SICK TiM571 LiDAR)
   - `192.168.0.11` (Rear SICK TiM571 LiDAR)

### Prerequisites & Permissions
- Administrative privileges (`sudo`).
- Executable permissions:
  ```bash
  chmod +x scripts/setup_system.sh
  ```

### Usage
```bash
# Run with standard user (will invoke sudo when needed)
./scripts/setup_system.sh

# Or run directly with sudo
sudo ./scripts/setup_system.sh
```

> [!NOTE]
> After running `setup_system.sh` for the first time, log out and log back in (or run `newgrp dialout` in your current terminal) for the new user groups to take effect.

---

## 🎮 2. Master Control Center: `start_kairos.sh`

### Purpose
The primary interactive console dashboard for operators. It performs live pre-flight hardware checks and presents an interactive menu to launch all robot driving modes, diagnostics, and sensor viewers.

### Real-Time Pre-Flight Header
Every time the menu is rendered, `start_kairos.sh` queries and displays:
- **CAN Bus Status**: Reports whether `can0` is active at 1 Mbps.
- **Network / Wi-Fi**: Current IP address on `wlp4s0` (e.g., `Robotica` network).
- **LiDAR Connectivity**: Pings the front SICK scanner (`192.168.0.10`) via Ethernet.
- **Gamepad Detection**: Verifies if `/dev/input/js0` (Sony DualShock 4) is connected.
- **Battery Telemetry**: Sends an SDO CAN query (`0x601`, index `0x200F:01`) to Node 1 to compute true DC bus voltage, percentage estimate, and checks if the 48V contactor is open.

### Menu Options
```
======================================================================
      RB-KAIROS — MASTER CONTROL & TELEMETRY CENTER (ROS 2 JAZZY)     
======================================================================
 CAN Bus:      ACTIVE (can0 @ 1 Mbps)
 Network:      192.168.0.183 (Wi-Fi Robotica)
 LiDARs:       CONNECTED (SICK TiM5xx Front/Rear @ 15 Hz)
 Gamepad:      CONNECTED (/dev/input/js0)
 Battery:      49.80 V (~62%) — 48V ACTIVE
----------------------------------------------------------------------
 Navigation Modes (Full Cockpit: Motors + 3D LiDAR + Camera + Battery):
  [1] 🎮   PS4 Gamepad Cockpit (DualShock 4 Bluetooth + Full Telemetry)
  [2] ⌨️    Interactive Keyboard Cockpit (Holonomic Control + Full Telemetry)
  [3] 🤖   Autonomous / Nav2 Cockpit (Listens to /cmd_vel + Full Telemetry)

 Individual Telemetry & Tools (Unit Testing):
  [4] 🔋   Live Battery Gauge (CLI Terminal)
  [5] 📊   Visual Battery Monitor Window (GUI Window)
  [6] 🔍   CAN Diagnostics & 4 AMC Drives State
  [7] 📷   RealSense D435 Camera Only (Color & Depth Stream)
  [8] 📡   360° LiDAR Radar Only (CLI Terminal)
  [9] 🗺️    LiDAR Visualization Only (RViz2)
  [q] 🚪   Quit
```

### Detailed Option Descriptions
- **`[1]` PS4 Gamepad Cockpit**: Launches `launch_cockpit.sh 1`. Checks for `/dev/input/js0` and prompts if Bluetooth gamepad is not detected.
- **`[2]` Keyboard Cockpit**: Launches `launch_cockpit.sh 2` for safe indoor laboratory driving with speed clamps ($0.08\text{ m/s}$ default, $0.15\text{ m/s}$ ceiling).
- **`[3]` Autonomous / Nav2 Cockpit**: Launches `launch_cockpit.sh 3` with Nav2 navigation stack accepting navigation goals from RViz2.
- **`[4]` Live Battery Gauge**: Interactive ASCII terminal gauge updated at 1.2 Hz showing voltages across all 4 motor drives ($N_1, N_2, N_3, N_4$).
- **`[5]` Visual Battery Monitor**: Launches the standalone PyQt/Tk battery dashboard (`battery_gui.py`) showing individual cell/drive health.
- **`[6]` CAN Diagnostics & 4 AMC Drives State**: Queries AMC DS402 status words (`0x6041`) and states (`Operation Enabled`, `Switch On Disabled`, `FAULT`).
- **`[7]` Camera Only**: Starts RealSense D435 ROS 2 driver and opens `rqt_image_view`.
- **`[8]` 360° LiDAR Radar Only**: Starts SICK drivers and displays the terminal ASCII proximity radar (`lidar_radar_cli`).
- **`[9]` LiDAR RViz2 Only**: Launches dual SICK drivers and opens RViz2 with laser scans.

### Usage
```bash
chmod +x scripts/start_kairos.sh
./scripts/start_kairos.sh
```

---

## 🖥️ 3. Integrated Navigation Cockpit: `launch_cockpit.sh`

### Purpose
Spawns the complete robot driver stack, dedicated terminal logging windows, and visual monitoring windows in parallel, then tiles them cleanly on the display according to screen resolution.

### Display Layouts (Calculated Dynamically)
The script interrogates `xrandr` to detect the screen resolution and adapts window geometries:
- **Standard Resolution (1920x1080)**: Tiles RViz2 ($1040\times620$), rqt_image_view ($830\times480$), Battery GUI ($400\times480$), Motor Terminal, LiDAR Terminal, Camera Terminal, and Teleoperation window ($114\times17$).
- **High Resolution (2560x1440 / 2560x1600)**: Expands RViz2 ($1380\times940$) and rqt_image_view ($1100\times650$) for maximum situational awareness.

### Window & Node Architecture
1. **Terminal 1 (`RB-KAIROS [MOTOR & CAN DRIVER]`)**:
   Runs `ros2 launch kairos_real_bringup base.launch.py` (50 Hz CANopen motor driver, VectorNav IMU, EKF odometry).
2. **Terminal 2 (`RB-KAIROS [SICK LIDARS & 3D ROBOT]`)**:
   Runs `ros2 launch kairos_real_bringup lidar.launch.py` (Front & Rear SICK TiM5xx drivers, laser merger, robot state publisher TF tree).
3. **Terminal 3 (`RB-KAIROS [REALSENSE D435 CAMERA]`)**:
   Runs `ros2 launch kairos_real_bringup camera.launch.py` (Intel RealSense D435 RGB-D driver).
4. **GUI Window 1**: RViz2 with pre-configured 3D model, coordinate frames, and 360° merged laser scans (`lidar_view.rviz`).
5. **GUI Window 2**: `rqt_image_view` showing color camera stream on `/camera/camera/color/image_raw`.
6. **GUI Window 3**: 48V Battery Monitor dashboard showing live bus voltages.
7. **Foreground Terminal**: Active control module based on requested `MODE`.

### Syntax & Operating Modes
```bash
./scripts/launch_cockpit.sh [MODE]
```

- **`MODE=1` — PS4 Gamepad Teleoperation:**
  ```bash
  ./scripts/launch_cockpit.sh 1
  ```
  - **`R1` (Mandatory Deadman Switch)**: Must be held down continuously to enable movement. Releasing halts motors instantly.
  - **Left Stick**: Linear velocity $v_x$ (forward/back) and lateral strafe $v_y$ (left/right holonomic mecanum).
  - **Right Stick**: Angular velocity $\omega_z$ (in-place rotation).
  - **`L1` (Turbo)**: Boosts maximum linear speed up to $0.15\text{ m/s}$.
  - **`L2` / `R2`**: Toggles omnidirectional mecanum vs differential drive modes.
  - **D-Pad**: Micro-stepping precision maneuvers ($0.04\text{ m/s}$).

- **`MODE=2` — Safe Keyboard Teleoperation (Default):**
  ```bash
  ./scripts/launch_cockpit.sh 2
  # Or simply:
  ./scripts/launch_cockpit.sh
  ```
  - **Arrow Keys / ZQSD / WASD**: Omnidirectional motion with immediate active braking on key release.
  - **`+` / `-`**: Incrementally adjust target speed by $\pm 0.02\text{ m/s}$ ($2\text{ cm/s}$) without moving the base.
  - **Spacebar**: Immediate emergency stop ($0\text{ m/s}$).
  - Built-in maximum hard limit of $0.15\text{ m/s}$ ($15\text{ cm/s}$).

- **`MODE=3` — Autonomous Navigation (Nav2):**
  ```bash
  ./scripts/launch_cockpit.sh 3
  ```
  - Launches Nav2 stack (`navigation.launch.py`) with holonomic DWB local controller.
  - Relies on `/odometry/filtered` (EKF fused CAN odometry + VectorNav IMU) and unified 360° laser scan (`/scan`).
  - Send 2D Goal Poses directly from RViz2 using the "Nav2 Goal" toolbar button.

### Graceful Shutdown
Pressing `Ctrl + C` in the main cockpit terminal invokes the `cleanup()` function:
1. Immediately sends a zero-velocity emergency stop message on `/cmd_vel`.
2. Sends `SIGINT` then `SIGTERM` to all background terminal processes via PID files.
3. Kills any lingering nodes by process signature (`pkill -f`).
4. Closes all GUI windows and leaves the robot in a secure state.

---

## 📷 4. Standalone Camera Viewer: `view_camera.sh`

### Purpose
Quick utility script to start the Intel RealSense D435 camera driver and inspect the live RGB video stream in `rqt_image_view`.

### Behavior
- Checks if a ROS 2 node named `camera` is already running:
  - If not running, launches `camera.launch.py` in the background.
  - If already running, connects directly to the existing stream.
- Opens `rqt_image_view` displaying `/camera/camera/color/image_raw`.
- When `rqt_image_view` is closed, the script automatically terminates the camera driver if it was started by this script.

### Usage
```bash
chmod +x scripts/view_camera.sh
./scripts/view_camera.sh
```

---

## 📡 5. Standalone LiDAR Viewer: `view_lidar.sh`

### Purpose
Quick utility script to test the dual SICK TiM5xx LiDAR scanners and visualize 360° laser scans in RViz2 without running motor drivers or teleoperation.

### Behavior
- Ensures static Ethernet routing to `192.168.0.10` and `192.168.0.11` via `enp3s0`.
- Checks if `/front_laser/scan` topic is being published:
  - If not active, launches `lidar.launch.py` in the background.
  - If already active, connects directly.
- Launches RViz2 loaded with `config/lidar_view.rviz`.
- Automatically terminates LiDAR drivers and TF publishers upon RViz2 exit (if launched by this script).

### Usage
```bash
chmod +x scripts/view_lidar.sh
./scripts/view_lidar.sh
```

---

## 📦 6. Workspace Dependencies Import: `kairos_dependencies.repos`

### Purpose
The `kairos_dependencies.repos` file at the repository root uses the standard ROS [vcstool](https://github.com/dirk-thomas/vcstool) YAML format. It lists all upstream repositories required to compile and run the full RB-KAIROS workspace.

### Referenced Upstream Repositories
| Directory Path | Upstream Repository URL | Branch | Description |
| :--- | :--- | :--- | :--- |
| `delto_m_ros2` | `https://github.com/tesollodelto/delto_m_ros2.git` | `humble` | Tesollo DG-5F robotic gripper hand ROS 2 interface |
| `Universal_Robots_ROS2_Description` | `https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git` | `jazzy` | UR5e robotic manipulator 3D description & meshes |
| `robotnik/robotnik_description` | `https://github.com/RobotnikAutomation/robotnik_description.git` | `jazzy-devel` | Robotnik base meshes and kinematic definitions |
| `robotnik/robotnik_common` | `https://github.com/RobotnikAutomation/robotnik_common.git` | `jazzy-devel` | Robotnik common utilities and launch tools |
| `robotnik/robotnik_sensors` | `https://github.com/RobotnikAutomation/robotnik_sensors.git` | `jazzy-devel` | Sensor descriptions (SICK LiDARs, cameras, IMUs) |
| `robotnik/robotnik_interfaces` | `https://github.com/RobotnikAutomation/robotnik_interfaces.git` | `jazzy-devel` | Custom messages and service definitions |
| `robotnik/robotnik_moveit_configs` | `https://github.com/RobotnikAutomation/robotnik_moveit_configs.git` | `jazzy-devel` | MoveIt 2 configuration packages |
| `robotnik/robotnik_simulation` | `https://github.com/RobotnikAutomation/robotnik_simulation.git` | `jazzy-devel` | Gazebo / Ignition simulation worlds and launch files |
| `robotnik/robotnik_teleop_panel` | `https://github.com/RobotnikAutomation/teleop_panel.git` | `ros2-devel` | RViz teleoperation panel plugins |

### Cloning a Complete Workspace in One Command
To clone the entire RB-KAIROS workspace on a new machine:

```bash
# 1. Create a clean workspace
mkdir -p ~/kairos_ws/src && cd ~/kairos_ws/src

# 2. Clone this core bringup repository
git clone git@github.com:Uncrowned0x0/RB--Kairos-Robot-for-ROS2-Jazzy.git kairos_real_bringup

# 3. Import all upstream dependencies with vcstool
vcs import . < kairos_real_bringup/kairos_dependencies.repos

# 4. Install system dependencies with rosdep
cd ~/kairos_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 5. Run the system setup script for hardware permissions & CAN bus
./src/kairos_real_bringup/scripts/setup_system.sh

# 6. Build the workspace
colcon build --symlink-install

# 7. Source the newly built workspace
source install/setup.bash
```

---

## ⚡ 7. Quick Command Reference

| Action | Command | Sudo Required? | Notes |
| :--- | :--- | :---: | :--- |
| **One-time System Setup** | `./scripts/setup_system.sh` | **Yes** | Configures udev rules, user groups, CAN bus, LiDAR network routes |
| **Interactive Control Menu** | `./scripts/start_kairos.sh` | No | Shows live hardware status, battery voltage, and numbered menu |
| **Full Cockpit (Keyboard)** | `./scripts/launch_cockpit.sh 2` | No | Starts all hardware drivers + RViz2 + rqt + Safe Keyboard Driving |
| **Full Cockpit (PS4 Gamepad)** | `./scripts/launch_cockpit.sh 1` | No | DualShock 4 driving (requires holding `R1` deadman switch) |
| **Full Cockpit (Nav2 Autonomous)** | `./scripts/launch_cockpit.sh 3` | No | Starts Nav2 stack; send goals in RViz2 |
| **Standalone Camera View** | `./scripts/view_camera.sh` | No | Displays Intel RealSense stream in `rqt_image_view` |
| **Standalone LiDAR View** | `./scripts/view_lidar.sh` | No | Displays dual SICK LiDAR scans in RViz2 |
| **Live Battery Terminal Gauge** | `./scripts/start_kairos.sh` $\to$ Option `4` | No | CLI ASCII battery gauge querying AMC drives via CAN |
| **AMC CAN Diagnostics** | `./scripts/start_kairos.sh` $\to$ Option `6` | No | Displays DC bus voltages and DS402 drive status words |
| **Clone Workspace Dependencies** | `vcs import src < kairos_dependencies.repos` | No | Clones all upstream git dependencies |

---

*RB-KAIROS System Documentation — Maintained by Kamil BENMADI (<kamil.benmadi@sigma-clermont.fr>)*
