# RB-KAIROS Mobile Base — Documentation Index & User Guide

Welcome to the official documentation for the **RB-KAIROS omnidirectional mobile base** under **ROS 2 Jazzy (Ubuntu 24.04 LTS)**.

This repository and documentation suite provide everything necessary to power up, operate, inspect, teleoperate, and develop software for the 4-wheel Mecanum robot base.

---

## 📚 Master Documentation Guides

Click on any guide below for detailed, step-by-step instructions:

| Document | Description | Target Reader |
| :--- | :--- | :--- |
| **[GUIDE_REMOTE_ACCESS_AND_STARTUP.md](docs/GUIDE_REMOTE_ACCESS_AND_STARTUP.md)** | Step-by-step instructions for powering up the robot, safety contactors (blue button), Wi-Fi/Ethernet network setup, connecting via Remote Desktop (RDP) on Linux/Windows/Mac, and headless HDMI operation. | Beginners, Operators, Everyone |
| **[GUIDE_CONTROL_MODES.md](docs/GUIDE_CONTROL_MODES.md)** | Complete guide to operating the robot: **Mode 1** (PS4 Gamepad teleop with deadman switch), **Mode 2** (Interactive safe keyboard driving), and **Mode 3** (Autonomous Nav2 navigation), plus camera, battery monitor, and radar tools. | Drivers, Operators, Researchers |
| **[GUIDE_DEVELOPER_CUSTOMIZATION.md](docs/GUIDE_DEVELOPER_CUSTOMIZATION.md)** | Comprehensive developer guide covering software architecture, exact file paths, bash build commands (`colcon build`), automated test suite, and step-by-step recipes to adjust max speeds, acceleration ramps, deadbands, and sensor topics. | Developers, Researchers, Engineers |
| **[README_script.md](scripts/README_script.md)** | Dedicated user guide for all automation scripts (`setup_system.sh`, `start_kairos.sh`, `launch_cockpit.sh`, `view_camera.sh`, `view_lidar.sh`) and workspace reproduction manifest (`kairos_dependencies.repos`). | Operators, Developers, Administrators |

---

## ⚡ 60-Second Quick Start

If you are already familiar with the safety procedures, here is the fastest way to get moving:

1. **Hardware Power-On:**
   - Twist the red **Emergency Stop** clockwise to pop it up.
   - Switch the rear breaker to **ON**.
   - Firmly press the **BLUE pushbutton** on the front panel (listen for the loud **"CLACK"** of the 48V power contactors).
   - Ensure the battery charging cable is **disconnected**.

2. **Connect Remotely:**
   - Join Wi-Fi **`Robotica`** on your laptop.
   - Open **Remmina** (or Windows `mstsc` / Mac Remote Desktop) and connect to **`192.168.0.183`** (Username: `kairos`, Password: `kairos`).

3. **Launch the Navigation Cockpit:**
   Open a terminal and run (either from home or from `scripts/`):
   ```bash
   # Option A: Interactive Control Menu
   /home/kairos/start_kairos.sh
   # (or: ./src/kairos_real_bringup/scripts/start_kairos.sh)

   # Option B: Direct PS4 Gamepad Driving
   /home/kairos/launch_cockpit.sh 1

   # Option C: Direct Safe Keyboard Driving (Default)
   /home/kairos/launch_cockpit.sh 2

   # Option D: Autonomous Nav2 Navigation
   /home/kairos/launch_cockpit.sh 3
   ```

4. **Shutdown & Safe State:**
   Press `Ctrl + C` in the main cockpit terminal. The launcher automatically sends a zero-velocity halt command and terminates all background sensor nodes safely.

---

## 🛡️ Critical Safety Rules

1. **Mandatory Deadman Switch (R1):** When using the PS4 Gamepad (Mode 1), the robot will **never** move unless you are actively holding the **`R1`** bumper. Releasing `R1` halts the robot immediately.
2. **Speed Limits:** Forward and lateral speeds are strictly clamped in the low-level motor driver to a maximum ceiling of **$0.15\text{ m/s}$ ($15\text{ cm/s}$)** for laboratory safety.
3. **48V Battery Management:**
   - Full: **`53.3 V`** (100%)
   - Nominal: **`48.0 V`** (~45%)
   - Low Warning: **`< 45.0 V`** (< 20%) — Plug in charger soon.
   - Cutoff Floor: **`44.0 V`** — Contactor trips to protect cells.
   - If voltage is `< 20.0 V`, the contactor is open: press the front blue button.
4. **Charger Disconnection:** **Never** drive the robot while plugged into AC mains charging power.

---

## 📁 Cleaned Workspace Directory Structure

The workspace at `/home/kairos/kairos_ws/` has been purged of all unrelated arm/sensor packages (UR5e control, Bota FT sensors, unrelated grippers) to prevent duplicate folders and developer confusion:

```
/home/kairos/kairos_ws/
├── TesolloHand DG-5F/          # Tesollo Hand utilities and configurations (KEPT in workspace)
└── src/
    ├── kairos_real_bringup/     # Native ROS 2 hardware drivers (Motors, SICK LiDARs, IMU, LEDs, GUI)
    │   ├── kairos_dependencies.repos # VCS manifest for cloning all workspace repositories
    │   ├── README_script.md    # Dedicated scripts documentation guide
    │   ├── scripts/            # Operational, setup, and visualization scripts
    │   ├── config/             # Hardware udev rules, EKF, Nav2, and RViz configurations
    │   └── launch/             # ROS 2 launch files (base, lidar, camera, nav2, teleop)
    ├── kairos_base_bridge/     # Bridge interfaces and kinematics definitions
    ├── rbkairos_description/   # Robot URDF, 3D meshes & visual models
    ├── robotnik/               # Robotnik description and simulation dependencies
    └── delto_m_ros2/           # Tesollo DG-5F robotic hand ROS 2 interface (KEPT)
```

---

*Author: Kamil BENMADI (<kamil.benmadi@sigma-clermont.fr>) — [GitHub](https://github.com/Uncrowned0x0) | [LinkedIn](https://www.linkedin.com/in/kamilb-)*
