# RB-KAIROS: Control Modes, Teleoperation & Navigation Guide

**Author:** Kamil BENMADI (<kamil.benmadi@sigma-clermont.fr>) — [GitHub](https://github.com/Uncrowned0x0) | [LinkedIn](https://www.linkedin.com/in/kamilb-)  
**Target Platform:** Robotnik RB-KAIROS Omnidirectional Mobile Base  
**Operating System:** Ubuntu 24.04 LTS  
**ROS 2 Distribution:** Jazzy Jalisco  
**Document Status:** Production Validated (September 2026)  

---

## 1. Overview of the Integrated Cockpit System

To eliminate manual juggling of multiple terminals, configuration files, and display coordinates, the RB-KAIROS environment includes an automated master launcher:

```bash
/home/kairos/launch_cockpit.sh [MODE]
```

Where `[MODE]` selects the primary navigation interface:
- **`1`**: PS4 DualShock Gamepad Teleoperation
- **`2`**: Safe Interactive Keyboard Teleoperation *(Default for lab safety)*
- **`3`**: Autonomous Navigation (Nav2 holonomic stack)

Alternatively, the interactive master menu can be launched via:
```bash
/home/kairos/start_kairos.sh
```

### What Happens When You Launch the Cockpit
The cockpit orchestrates the entire robot software stack in parallel, automatically organizing GUI windows according to screen resolution (1080p or 2560x1600):
1. **CANopen Motor Driver (50 Hz SYNC):** Manages 4 AMC drives on `can0`, enforces velocity clamping, watchdog timeout, and publishes `/odom_raw` and `/joint_states`.
2. **LiDAR & 3D TF Tree:** Connects to Front (`192.168.0.10`) and Rear (`192.168.0.11`) SICK TiM5xx scanners, merges them into `/scan`, and broadcasts the full 3D robot model with wheel transforms.
3. **Intel RealSense D435 Camera:** Streams synchronized color and depth point clouds on `/camera/camera/color/image_raw`.
4. **RViz2 3D Scene Window:** Positioned on the left half of the display, visualizes the 360° laser scans, TF tree, and robot geometry.
5. **HD Camera Stream Window:** Uses `rqt_image_view` to provide real-time forward situational awareness.
6. **48V Battery Monitor Window:** Displays live state of charge, DC bus voltages per drive, and contactor status.
7. **Control Module:** Runs in the active foreground terminal for direct, latency-free driving.

---

## 2. Mode 1: PS4 Gamepad (DualShock 4) Teleoperation

The PS4 DualShock 4 controller provides intuitive, proportional wireless driving with hardware-level safety switches.

### 2.1. Pairing the Gamepad via Bluetooth
1. If the controller is off, press and hold **`SHARE`** and the central **`PS`** button simultaneously for 3 seconds until the light bar flashes rapidly in white strobes (pairing mode).
2. On the robot desktop, open Bluetooth settings and pair with **"Wireless Controller"** (or it will auto-reconnect if already paired).
3. Test that Linux recognized the joystick:
   ```bash
   ls -l /dev/input/js0
   ```
4. Optional test utility:
   ```bash
   jstest /dev/input/js0
   ```

### 2.2. Launching Mode 1
```bash
/home/kairos/launch_cockpit.sh 1
```

### 2.3. Control Mapping & Safety Rules
The controller operates under strict safety interlocks:

```
                  [L1] TURBO BOOST              [R1] MANDATORY DEADMAN SWITCH
                  (Hold to reach 0.15 m/s)      (MUST be held to allow ANY movement)
                          |                                     |
                  [L2] TOGGLE/HOLD MECANUM      [R2] TOGGLE MECANUM MODE
                          |                                     |
                     +--+---+                                 +---+--+
                     |      |                                 |      |
          [D-PAD]    |  /\  |                                 | (/\) |
       Micro-steps   |<    >|         [SHARE]    [OPTIONS]    |([])()|
       (4 cm/s fine) |  \/  |                                 | (X)  |
                     +------+                                 +------+
                         \                                       /
                      ( L )                                   ( R )
               [LEFT ANALOG STICK]                    [RIGHT ANALOG STICK]
               - Forward / Backward (vx)              - In-Place Rotation (wz)
               - Lateral Strafe (vy in Mecanum)
               - Steering Yaw (wz in Diff mode)
```

#### Detailed Input Specification:
- **`R1` (Primary Deadman Safety):**
  - **MANDATORY:** You **must hold R1** with your right index finger at all times while driving.
  - **Immediate Stop:** Releasing R1 at any instant immediately clamps `/cmd_vel` to 0.0 m/s and activates electromagnetic motor braking.
  - **Neutral Safety:** Holding R1 alone when the sticks are centered produces **strictly zero motion**.
- **`Left Analog Stick` (Translation):**
  - Push Forward/Backward: Commands longitudinal speed $v_x$ (clean, zero angular leakage).
  - Push Left/Right:
    - In **Mecanum Mode**: Commands lateral crabbing speed $v_y$ (the robot glides sideways without turning).
    - In **Differential Mode**: Commands smooth steering rotation $\omega_z$.
- **`Right Analog Stick` (In-Place Spin):**
  - Push Left/Right: Commands pure rotation $\omega_z$ around the robot's center axis.
- **`L1` (Turbo Speed Boost):**
  - Default crawl speed is strictly capped at **$0.08\text{ m/s}$ ($8\text{ cm/s}$)** for safe lab operation.
  - Holding **`L1`** alongside stick displacement unlocks the turbo ceiling up to **$0.15\text{ m/s}$ ($15\text{ cm/s}$)**.
- **`R2` / `L2` (Mecanum Kinematic Toggle):**
  - Tapping **`R2`** or **`L2`** toggles between Differential (conventional tank/car steering) and Omnidirectional Mecanum mode.
  - Holding **`L2`** momentarily engages Mecanum crabbing even when default is differential.
- **`D-Pad (Arrows)` (Docking Micro-Steps):**
  - Pressing D-Pad directions commands a slow, fixed **$0.04\text{ m/s}$ ($4\text{ cm/s}$)** micro-step for millimeter-precise docking.

---

## 3. Mode 2: Safe Interactive Keyboard Teleoperation

Designed specifically for laboratory environments, testing benches, and scenarios where a physical gamepad is unavailable.

### 3.1. Launching Mode 2
```bash
/home/kairos/launch_cockpit.sh 2
# Or standalone:
ros2 run kairos_real_bringup kairos_teleop_keyboard
```

### 3.2. Key Mapping & Controls
The keyboard driver supports both **AZERTY** and **QWERTY** layouts:

| Action | Primary Keys | Alternative Keys | Kinematic Motion |
| :--- | :--- | :--- | :--- |
| **Move Forward** | `Up Arrow` | `Z` / `W` / `I` | Linear $v_x > 0$ |
| **Move Backward** | `Down Arrow` | `S` / `K` | Linear $v_x < 0$ |
| **Turn Left (Spin)** | `Left Arrow` | `Q` / `A` / `J` | Angular $\omega_z > 0$ |
| **Turn Right (Spin)** | `Right Arrow` | `D` / `L` | Angular $\omega_z < 0$ |
| **Mecanum Strafe Left** | `Shift + Left Arrow` | `U` | Lateral $v_y > 0$ |
| **Mecanum Strafe Right** | `Shift + Right Arrow` | `O` | Lateral $v_y < 0$ |
| **Increase Speed** | `+` (Plus key) | `P` | Increases linear speed by $+0.02\text{ m/s}$ *(Robot stays halted)* |
| **Decrease Speed** | `-` (Minus key) | `M` | Decreases linear speed by $-0.02\text{ m/s}$ *(Robot stays halted)* |
| **EMERGENCY STOP** | `SPACEBAR` | `X` / `C` | Instantaneous $0\text{ m/s}$ active brake |
| **Clean Exit** | `Ctrl + C` | - | Safely stops motors and exits terminal |

### 3.3. Built-In Safety Features
1. **Deadman Key-Release Halt:** The node requires continuous keypress events. If no key is received for **$250\text{ ms}$**, target velocity immediately drops to zero.
2. **Decoupled Speed Configuration:** Pressing `+` or `-` adjusts the internal velocity scaler **WITHOUT sending velocity to the wheels**. You can safely adjust desired speed while stationary.
3. **Hard Clamping:** Speed cannot be increased beyond $0.15\text{ m/s}$ ($15\text{ cm/s}$) nor decreased below $0.03\text{ m/s}$ ($3\text{ cm/s}$).

---

## 4. Mode 3: Autonomous Nav2 Navigation

Mode 3 activates the ROS 2 Nav2 navigation stack configured specifically for the omnidirectional Mecanum chassis.

### 4.1. Launching Mode 3
```bash
/home/kairos/launch_cockpit.sh 3
# Or standalone:
ros2 launch kairos_real_bringup navigation.launch.py
```

### 4.2. Navigation Pipeline Architecture
- **Robot State Publisher:** Publishes full TF tree from URDF (`base_footprint` -> `base_link` -> sensor frames).
- **EKF Fusion (`robot_localization`):** Fuses wheel odometry from forward Mecanum kinematics (`/odom_raw`) and IMU telemetry from the VectorNav VN-100 (`/imu/data`) to produce `/odometry/filtered` and broadcast `odom` -> `base_footprint`.
- **Dual LiDAR 360° Merger:** Combines front and rear SICK scans into `/scan` with $0.5^\circ$ angular resolution and no blind spots.
- **DWB Holonomic Controller:** Configured with `DWBLocalPlanner` supporting simultaneous $v_x$ and $v_y$ velocities, allowing the robot to translate sideways around obstacles without turning.
- **Costmaps:** 2D Inflation and Obstacle layers subscribed to `/scan`.

### 4.3. Sending Navigation Goals from RViz2
1. In the RViz2 window, verify that the 3D robot model, laser point clouds, and costmaps are visible.
2. Click the **"Nav2 Goal"** (or **"2D Goal Pose"**) tool button in the top RViz toolbar.
3. Click on any free space on the map where you want the robot to navigate.
4. While holding the left mouse button, drag the green arrow to orient the robot's target heading, then release.
5. The Nav2 planner computes a global path (yellow line) and the local controller steers the Mecanum wheels smoothly to the goal.

---

## 5. Standalone Inspection & Telemetry Utilities

You can launch individual utilities independently without running the entire cockpit:

### 5.1. 48V Battery Dashboard GUI
Displays live voltage per drive, total remaining charge percentage, and contactor state:
```bash
python3 /home/kairos/kairos_ws/src/kairos_real_bringup/kairos_real_bringup/battery_gui.py
```

### 5.2. Live RealSense Camera Stream
Launches the camera driver and opens `rqt_image_view` on `/camera/camera/color/image_raw`:
```bash
/home/kairos/view_camera.sh
```

### 5.3. 360° ASCII LiDAR Radar (CLI Console)
Renders a live 360° ASCII circular radar in your terminal showing obstacles up to 4 meters away:
```bash
ros2 run kairos_real_bringup lidar_radar_cli
```

### 5.4. SICK LiDAR 3D Visualization in RViz2
Launches front and rear LiDAR drivers and opens pre-configured RViz2 display:
```bash
/home/kairos/view_lidar.sh
```

### 5.5. Low-Level CANopen Motor Drive Diagnostics
Queries objects `0x200F` (DC voltage) and `0x6041` (DS402 Statusword) across all 4 AMC drives:
```bash
python3 -c "
import can, struct, time
bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
for n in [1, 2, 3, 4]:
    bus.send(can.Message(arbitration_id=0x600+n, data=[0x40, 0x41, 0x60, 0, 0, 0, 0, 0], is_extended_id=False))
t0 = time.time()
while time.time() - t0 < 0.2:
    msg = bus.recv(timeout=0.03)
    if msg and 0x581 <= msg.arbitration_id <= 0x584 and len(msg.data) >= 6:
        n = msg.arbitration_id - 0x580
        sw = struct.unpack('<H', bytes(msg.data[4:6]))[0]
        st = 'Operation Enabled' if (sw & 0x6F) == 0x27 else f'State 0x{sw:04X}'
        print(f'Drive {n}: Statusword=0x{sw:04X} -> {st}')
bus.shutdown()
"
```

---

*Author: Kamil BENMADI (<kamil.benmadi@sigma-clermont.fr>) — [GitHub](https://github.com/Uncrowned0x0) | [LinkedIn](https://www.linkedin.com/in/kamilb-)*
