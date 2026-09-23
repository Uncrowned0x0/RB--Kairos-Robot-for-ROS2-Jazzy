# RB-KAIROS: Developer Customization & Code Architecture Guide

**Author:** Kamil BENMADI (<kamil.benmadi@sigma-clermont.fr>) — [GitHub](https://github.com/Uncrowned0x0) | [LinkedIn](https://www.linkedin.com/in/kamilb-)  
**Target Audience:** Robotics Engineers, Researchers, and Students  
**Environment:** ROS 2 Jazzy Jalisco on Ubuntu 24.04 LTS  
**Primary Package:** `kairos_real_bringup` (`/home/kairos/kairos_ws/src/kairos_real_bringup/`)  
**Document Status:** Production Validated (September 2026)  

---

## 1. Codebase Architecture & File Structure

All drivers, launch configurations, kinematic converters, and safety managers for the RB-KAIROS mobile base are located inside `/home/kairos/kairos_ws/src/kairos_real_bringup/`.

```
/home/kairos/kairos_ws/src/kairos_real_bringup/
├── config/
│   ├── 51-kairos-devices.rules   # Udev device symlinks for VectorNav IMU and Teensy LED
│   ├── ekf.yaml                  # robot_localization EKF config (/odom_raw + /imu/data -> TF)
│   ├── lidar_view.rviz           # Pre-configured RViz2 layout for dual 360° laser scans
│   ├── nav2_params.yaml          # Nav2 navigation stack tuning (DWB holonomic controller)
│   └── ps4_teleop.yaml           # PS4 DualShock button bindings, axes & speed scales
├── kairos_real_bringup/
│   ├── __init__.py
│   ├── battery_gui.py            # Tkinter graphical dashboard for 48V battery & AMC drives
│   ├── dual_laser_merger.py      # Fuses front + rear 270° SICK scans into single 360° /scan
│   ├── kairos_led_driver.py      # Sends color commands to Teensyduino LED strips via serial
│   ├── kairos_serial_motor_driver.py # 50Hz CANopen CiA 402 driver for 4 AMC wheel drives
│   ├── kairos_teleop_joy.py      # DualShock 4 teleop with Mecanum mode & Deadman interlock
│   ├── kairos_teleop_keyboard.py # Interactive terminal keyboard teleoperation with active braking
│   ├── lidar_radar_cli.py        # Live 360° ASCII radar visualization in terminal
│   ├── sick_tim_driver.py        # Ethernet driver for SICK TiM551/561/571 (CoLa-A port 2111)
│   └── vectornav_driver.py       # High-speed VectorNav VN-100 IMU serial telemetry decoder
├── launch/
│   ├── base.launch.py            # Launches motor driver, VectorNav IMU, EKF node & LEDs
│   ├── camera.launch.py          # Launches Intel RealSense D435 RGB-D camera driver
│   ├── lidar.launch.py           # Launches Front + Rear SICK LiDARs + 360° Merger + URDF TF
│   ├── navigation.launch.py      # Launches Nav2 navigation stack for holonomic base
│   ├── sensors.launch.py         # Convenience launcher for Camera + Dual LiDARs
│   └── teleop_joy.launch.py      # Launches joy_node + kairos_teleop_joy
├── test/
│   ├── test_copyright.py         # ament_copyright test
│   ├── test_flake8.py            # PEP8 / flake8 code style verification
│   ├── test_kinematics_and_drivers.py # Unit tests for Mecanum kinematics & transforms
│   ├── test_pep257.py            # Docstring format verification
│   ├── test_teleop_joy.py        # 17 automated tests for PS4 deadman & safety logic
│   └── test_teleop_keyboard.py   # 6 automated tests for keyboard teleop safety
├── package.xml
├── setup.cfg
└── setup.py
```

### Convenience Launcher Scripts in `/home/kairos/`
- **`/home/kairos/launch_cockpit.sh`**: The master cockpit script that launches all robot systems in parallel and positions GUI windows on the screen.
- **`/home/kairos/start_kairos.sh`**: Interactive CLI menu offering full cockpit launch or individual tool testing.
- **`/home/kairos/view_camera.sh`**: Launches camera driver and opens `rqt_image_view`.
- **`/home/kairos/view_lidar.sh`**: Launches LiDAR drivers and opens `rviz2`.

---

## 2. Build & Verification Workflow

Whenever you modify any Python script, launch file, or YAML configuration, follow this standard ROS 2 workflow:

### 2.1. Sourcing ROS 2 Environment
Always ensure ROS 2 Jazzy is sourced in your terminal:
```bash
source /opt/ros/jazzy/setup.bash
```

### 2.2. Building the Workspace
Build with `--base-paths src` so colcon focuses exclusively on the ROS 2 packages inside `src/`:
```bash
cd /home/kairos/kairos_ws
colcon build --base-paths src --symlink-install
```

To build only the bringup package:
```bash
colcon build --base-paths src --packages-select kairos_real_bringup --symlink-install
```

### 2.3. Sourcing the Workspace Overlay
After a successful build, source the local install:
```bash
source /home/kairos/kairos_ws/install/setup.bash
```

### 2.4. Running the Test Suite
Ensure code quality and kinematic integrity by running the unit tests:
```bash
cd /home/kairos/kairos_ws
colcon test --base-paths src --packages-select kairos_real_bringup --event-handlers console_direct+
```

Verify test results:
```bash
colcon test-result --all --verbose
```

Run linters directly:
```bash
# Check PEP 8 compliance
ament_flake8 /home/kairos/kairos_ws/src/kairos_real_bringup

# Check PEP 257 docstring compliance
ament_pep257 /home/kairos/kairos_ws/src/kairos_real_bringup
```

---

## 3. Step-by-Step Customization Recipes

### Recipe 1: Increasing or Decreasing Maximum Driving Speed

The robot driving speed is governed by a multi-tier safety architecture:

```
[User Input] --> [Teleop Scaler] --> [Driver Safety Clamp] --> [Motor Drives]
                 (PS4 / Keyboard)    (Hard 0.15 m/s limit)
```

#### Step A: Adjust the Motor Driver Hard Safety Clamp
Open `/home/kairos/kairos_ws/src/kairos_real_bringup/kairos_real_bringup/kairos_serial_motor_driver.py`.  
Locate the ROS 2 parameter declarations around lines 32–33:
```python
self.declare_parameter('max_linear_velocity', 0.15)   # Strict crawl speed ceiling (m/s)
self.declare_parameter('max_angular_velocity', 0.30)  # Strict angular ceiling (rad/s)
```
- To increase max speed (e.g. to $0.25\text{ m/s}$): change `0.15` to `0.25`.
- To decrease max speed (e.g. to $0.10\text{ m/s}$): change `0.15` to `0.10`.

#### Step B: Adjust PS4 Gamepad Teleoperation Speeds
Open `/home/kairos/kairos_ws/src/kairos_real_bringup/config/ps4_teleop.yaml`.  
Locate lines 28–34:
```yaml
kairos_teleop_joy:
  ros__parameters:
    scale_linear_x: 0.08         # Standard crawl speed (m/s)
    scale_linear_y: 0.08         # Standard lateral strafe speed (m/s)
    scale_angular_z: 0.20        # Standard rotation speed (rad/s)
    scale_linear_x_turbo: 0.15   # Max speed with L1 turbo held (m/s)
    scale_linear_y_turbo: 0.15   # Max strafe with L1 turbo held (m/s)
    scale_angular_z_turbo: 0.30  # Max rotation with L1 turbo held (rad/s)
    scale_fine_linear: 0.04      # D-pad micro-step precision speed (m/s)
```
- Modify `scale_linear_x` to change normal forward speed.
- Modify `scale_linear_x_turbo` to match the new driver ceiling from Step A.

#### Step C: Adjust Keyboard Teleoperation Speeds
Open `/home/kairos/kairos_ws/src/kairos_real_bringup/kairos_real_bringup/kairos_teleop_keyboard.py`.  
Locate lines 44–49:
```python
self.declare_parameter('default_linear_speed', 0.08)
self.declare_parameter('default_angular_speed', 0.20)
self.declare_parameter('max_linear_speed', 0.15)
self.declare_parameter('min_linear_speed', 0.03)
```
- Update `default_linear_speed` and `max_linear_speed` as desired.

#### Step D: Adjust Nav2 Autonomous Navigation Speeds
Open `/home/kairos/kairos_ws/src/kairos_real_bringup/config/nav2_params.yaml`.  
Locate the `controller_server` section (`FollowPath` / `DWBLocalPlanner`):
```yaml
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.15
      min_vel_x: -0.15
      max_vel_y: 0.15
      min_vel_y: -0.15
      max_vel_theta: 0.30
      min_speed_xy: 0.02
      max_speed_xy: 0.15
```
- Update `max_vel_x`, `max_vel_y`, and `max_speed_xy` to match your target speed.

---

### Recipe 2: Modifying Acceleration & Deceleration Ramps

To make the robot accelerate more aggressively or glide to a gentler stop:
Open `kairos_serial_motor_driver.py` and locate lines 34–35:
```python
self.declare_parameter('max_linear_accel', 0.20)      # Linear accel ramp (m/s^2)
self.declare_parameter('max_angular_accel', 0.40)     # Angular accel ramp (rad/s^2)
```
- **Smoother, softer acceleration (e.g. carrying delicate payloads):**
  Decrease `max_linear_accel` to `0.10` $\text{m/s}^2$.
- **More responsive, instantaneous acceleration:**
  Increase `max_linear_accel` to `0.35` $\text{m/s}^2$.

---

### Recipe 3: Adjusting Joystick Deadbands (Eliminating Stick Drift)

If an older PS4 controller exhibits analog stick drift that causes slow creep:
Open `/home/kairos/kairos_ws/src/kairos_real_bringup/config/ps4_teleop.yaml`:
```yaml
kairos_teleop_joy:
  ros__parameters:
    deadzone: 0.08   # Default: ignores stick inputs smaller than 8%
```
- Increase `deadzone` to `0.12` or `0.15` to reject worn stick potentiometers.

---

### Recipe 4: Changing the Control Loop Frequency

The CANopen motor driver executes a real-time cyclic control loop that computes inverse kinematics, dispatches RPDO22 frames to all 4 drives, transmits the CANopen SYNC pulse, and integrates odometry.

Open `kairos_serial_motor_driver.py` line 30:
```python
self.declare_parameter('control_freq', 50.0)     # Control loop freq (Hz)
```
- Default is **50.0 Hz** ($20\text{ ms}$ period), which perfectly matches the AMC drive hardware internal filter bandwidth.
- Also verify `base.launch.py` line 22 if overridden there.

---

### Recipe 5: Tuning SICK LiDAR Topics, Ranges & IP Addresses

Open `/home/kairos/kairos_ws/src/kairos_real_bringup/launch/lidar.launch.py`:
- **Front LiDAR:**
  ```python
  'sensor_ip': '192.168.0.10',
  'sensor_port': 2111,
  'frame_id': 'front_laser_link',
  'scan_topic': '/front_laser/scan',
  'range_min': 0.05,
  'range_max': 25.0
  ```
- **Rear LiDAR:**
  ```python
  'sensor_ip': '192.168.0.11',
  'sensor_port': 2111,
  'frame_id': 'rear_laser_link',
  'scan_topic': '/rear_laser/scan',
  'range_min': 0.05,
  'range_max': 25.0
  ```
- **Dual Laser 360° Merger:**
  ```python
  'destination_frame': 'base_footprint',
  'scan_rate': 15.0,              # Hz
  'angle_increment_deg': 0.5,     # 720 bins covering 360 degrees
  'merged_scan_topic': '/scan'
  ```
To ignore very close obstacles (e.g. robot chassis reflection or cable bundles), increase `range_min` from `0.05` to `0.12`.

---

### Recipe 6: Robot Physical Dimensions & Kinematics Constants

If physical wheel components, gearboxes, or encoder disks are altered:
Open `kairos_serial_motor_driver.py` lines 23–29:
```python
self.declare_parameter('track_width', 0.538)     # Distance left/right wheel centers (m)
self.declare_parameter('wheel_base', 0.430)      # Distance front/rear wheel centers (m)
self.declare_parameter('wheel_diameter', 0.25)   # Mecanum outer diameter (m)
self.declare_parameter('gearbox_ratio', 9.56)    # Planetary reducer ratio
self.declare_parameter('speed_scale_factor', 6.5536) # AMC internal scale constant
self.declare_parameter('encoder_factor', 4000.0) # Optical encoder counts/revolution
```
The driver automatically recalculates the conversion factor:
$$dRps2Ref = \frac{K_{enc} \times G \times 6.5536}{2\pi}$$
and converts target wheel angular speed (in rad/s) into AMC digital velocity commands:
$$\text{Units} = \text{int32}(\omega \times dRps2Ref \times \text{spin\_dir})$$

---

### Recipe 7: Customizing LED Status Light Colors

Open `/home/kairos/kairos_ws/src/kairos_real_bringup/kairos_real_bringup/kairos_led_driver.py`.  
Locate `update_led_state()` around lines 122–136:
- **Red:** `COLOR:RED` (Emergency stop or critical fault)
- **Yellow:** `COLOR:YELLOW` (Low battery warning $< 20\%$ or $< 45\text{ V}$)
- **Blue:** `COLOR:BLUE` (Robot in active driving motion)
- **Green:** `COLOR:GREEN` (Robot idle, healthy, contactor closed)

You can publish custom states to the LED driver at runtime:
```bash
ros2 topic pub --once /led_state std_msgs/msg/String "{data: 'PURPLE'}"
```

---

*Author: Kamil BENMADI (<kamil.benmadi@sigma-clermont.fr>) — [GitHub](https://github.com/Uncrowned0x0) | [LinkedIn](https://www.linkedin.com/in/kamilb-)*
