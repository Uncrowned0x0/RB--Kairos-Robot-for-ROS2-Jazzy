# Copyright 2026 Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
CANopen motor driver node for RB-KAIROS 4-wheel Mecanum base.

Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
"""

import math
import struct
import threading
import time

import can
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, JointState
import tf2_ros


class CANMotorDriver(Node):

    def __init__(self):
        super().__init__('kairos_can_motor_driver')

        # Parameters matching Robotnik Summit / RB-KAIROS 48V configuration
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_bitrate', 1000000)
        self.declare_parameter('track_width', 0.538)     # Distance left/right (m)
        self.declare_parameter('wheel_base', 0.430)      # Distance front/rear (m)
        self.declare_parameter('wheel_diameter', 0.25)   # Mecanum diameter (m)
        self.declare_parameter('gearbox_ratio', 9.56)    # 48V AMC drives ratio
        self.declare_parameter('speed_scale_factor', 6.5536)
        self.declare_parameter('has_encoder', True)
        self.declare_parameter('encoder_factor', 4000.0)  # Optical counts/rev
        self.declare_parameter('control_freq', 50.0)     # Control loop freq (Hz)
        self.declare_parameter('cmd_timeout', 0.35)       # Watchdog timeout (s)
        self.declare_parameter('max_linear_velocity', 0.15)   # Strict crawl speed ceiling (m/s)
        self.declare_parameter('max_angular_velocity', 0.30)  # Strict angular ceiling (rad/s)
        self.declare_parameter('max_linear_accel', 0.20)      # Linear accel ramp (m/s^2)
        self.declare_parameter('max_angular_accel', 0.40)     # Angular accel ramp (rad/s^2)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_odom_tf', False)

        self.can_interface = self.get_parameter('can_interface').value
        self.can_bitrate = self.get_parameter('can_bitrate').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.gearbox_ratio = self.get_parameter('gearbox_ratio').value
        self.speed_scale_factor = self.get_parameter('speed_scale_factor').value
        self.has_encoder = self.get_parameter('has_encoder').value
        if self.has_encoder:
            self.encoder_factor = self.get_parameter('encoder_factor').value
        else:
            self.encoder_factor = 48.0
        self.control_freq = self.get_parameter('control_freq').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.max_linear_vel = float(self.get_parameter('max_linear_velocity').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_velocity').value)
        self.max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        self.max_angular_accel = float(self.get_parameter('max_angular_accel').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value

        # CAN Node IDs and Kinematic definitions (Robotnik standard)
        # Node 1: Front-Left, Node 2: Back-Left, Node 3: Front-Right, Node 4: Back-Right
        self.nodes = [1, 2, 3, 4]
        self.spin_dirs = {1: 1.0, 2: 1.0, 3: -1.0, 4: -1.0}
        self.joint_names = [
            'robot_front_left_wheel_joint',
            'robot_back_left_wheel_joint',
            'robot_front_right_wheel_joint',
            'robot_back_right_wheel_joint'
        ]

        # Velocity scaling factor from rad/s (at the wheel) to AMC driver units:
        # dRps2Ref = (encoder_factor * gearbox_ratio * SPEED_SCALE_FACTOR) / (2 * pi)
        numerator = self.encoder_factor * self.gearbox_ratio * self.speed_scale_factor
        self.dRps2Ref = numerator / (2.0 * math.pi)
        self.dRef2Rps = 1.0 / self.dRps2Ref if self.dRps2Ref > 0 else 1.0

        self.get_logger().info(
            f'Kinematics initialized: Gearbox={self.gearbox_ratio}, '
            f'Diameter={self.wheel_diameter}m, dRps2Ref={self.dRps2Ref:.2f} (units per rad/s)'
        )

        # State storage
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.target_rad_s = {node: 0.0 for node in self.nodes}
        self.actual_rad_s = {node: 0.0 for node in self.nodes}
        self.actual_pos_rad = {node: 0.0 for node in self.nodes}
        self.status_word = {node: 0 for node in self.nodes}
        self.last_cmd_time = 0.0
        self.last_motion_log_time = 0.0
        self.is_enabled = False
        self.running = True
        self.battery_voltage = 0.0

        # Odometry state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_wz = 0.0
        self.last_odom_time = self.get_clock().now()

        # Open CAN Bus
        try:
            self.bus = can.interface.Bus(
                channel=self.can_interface,
                bustype='socketcan',
                bitrate=self.can_bitrate
            )
            self.get_logger().info(
                f"Opened SocketCAN interface '{self.can_interface}' at {self.can_bitrate} bps."
            )
        except Exception as e:
            self.get_logger().error(f"Could not open CAN interface '{self.can_interface}': {e}")
            self.bus = None

        if self.bus:
            # Start background CAN RX thread
            self.rx_thread = threading.Thread(target=self.can_rx_loop, daemon=True)
            self.rx_thread.start()

            # Initialize AMC drives to Velocity Mode with RPDO22
            self.init_drives()

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.wheel_joint_pub = self.create_publisher(
            JointState, '/joint_states_wheels', 10
        )
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timers:
        # 1. Control loop (50Hz) for cyclic SYNC + RPDO22
        control_period = 1.0 / self.control_freq
        self.control_timer = self.create_timer(control_period, self.control_loop_callback)
        # 2. JointState publication (20Hz)
        self.joint_timer = self.create_timer(0.05, self.publish_joint_states)
        # 3. Auto-recovery watchdog & health check (1Hz)
        self.health_timer = self.create_timer(1.0, self.health_check_callback)

    def can_rx_loop(self):
        """Continuously read incoming CAN packets for feedback (TPDO1, TPDO21, TPDO22)."""
        while self.running and self.bus:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is None:
                    continue

                arb_id = msg.arbitration_id

                # Feedback TPDO22 (Velocity actual value): COB-ID = 0x3F0 + node
                if 0x3F1 <= arb_id <= 0x3F4 and len(msg.data) >= 4:
                    node = arb_id - 0x3F0
                    raw_vel = struct.unpack('<i', msg.data[0:4])[0]
                    rad_s = (raw_vel * self.dRef2Rps) * self.spin_dirs[node]
                    self.actual_rad_s[node] = rad_s

                # Feedback TPDO1 (Statusword): COB-ID = 0x4A0 + node
                elif 0x4A1 <= arb_id <= 0x4A4 and len(msg.data) >= 2:
                    node = arb_id - 0x4A0
                    self.status_word[node] = struct.unpack('<H', msg.data[0:2])[0]

                # Feedback TPDO21 (Position actual value): COB-ID = 0x401 <= arb_id <= 0x404
                elif 0x401 <= arb_id <= 0x404 and len(msg.data) >= 4:
                    node = arb_id - 0x400
                    raw_pos = struct.unpack('<i', msg.data[0:4])[0]
                    # Convert raw counts to rad
                    enc_total = self.encoder_factor * self.gearbox_ratio
                    pos_rad = (raw_pos / enc_total) * (2.0 * math.pi) * self.spin_dirs[node]
                    self.actual_pos_rad[node] = pos_rad

                # Feedback SDO (DC Bus Voltage from Node 1): COB-ID = 0x581
                elif arb_id == 0x581 and len(msg.data) >= 8:
                    if msg.data[1] == 0x0F and msg.data[2] == 0x20 and msg.data[3] == 0x01:
                        raw_volt = struct.unpack('<i', msg.data[4:8])[0]
                        self.battery_voltage = raw_volt / 177.3160173

            except Exception:
                pass

    def send_can_message(self, arbitration_id, data):
        """Transmit a CAN frame."""
        if self.bus:
            msg = can.Message(
                arbitration_id=arbitration_id,
                data=data,
                is_extended_id=False
            )
            try:
                self.bus.send(msg)
            except can.CanError as e:
                self.get_logger().warn(
                    f'CAN send error on ID 0x{arbitration_id:03X}: {e}',
                    throttle_duration_sec=2.0
                )

    def send_sdo(self, node, index, subindex, value, size=4):
        """Send a standard CANOpen expedited SDO write."""
        if size == 1:
            cmd = 0x2F
            val_bytes = struct.pack('<B', value & 0xFF) + b'\x00\x00\x00'
        elif size == 2:
            cmd = 0x2B
            val_bytes = struct.pack('<H', value & 0xFFFF) + b'\x00\x00'
        else:
            cmd = 0x23
            val_bytes = struct.pack('<i', value)

        idx_bytes = struct.pack('<H', index)
        sub_byte = struct.pack('<B', subindex)
        payload = bytes([cmd]) + idx_bytes + sub_byte + val_bytes
        self.send_can_message(0x600 + node, list(payload))
        time.sleep(0.01)

    def enable_drive(self, node):
        """Execute DS402 enable state sequence for a specific drive."""
        # 1. Fault reset (0x0080)
        self.send_sdo(node, 0x6040, 0x00, 0x0080, size=2)
        # 2. Shutdown (0x0006) -> Ready to Switch On
        self.send_sdo(node, 0x6040, 0x00, 0x0006, size=2)
        time.sleep(0.02)
        # 3. Switch On (0x0007) -> Switched On
        self.send_sdo(node, 0x6040, 0x00, 0x0007, size=2)
        time.sleep(0.02)
        # 4. Enable Operation (0x000F) -> Operation Enabled (Torque active)
        self.send_sdo(node, 0x6040, 0x00, 0x000F, size=2)
        time.sleep(0.02)

    def init_drives(self):
        """Configure the 4 AMC drives using Robotnik's native CANOpen sequence."""
        if not self.bus:
            return

        self.get_logger().info('Configuring AMC motor drives...')

        # 1. NMT: Start all nodes (Put into Operational State)
        self.send_can_message(0x000, [0x01, 0x00])
        time.sleep(0.05)

        for node in self.nodes:
            self.get_logger().info(f'Setting up Node {node}...')

            # Set Profile Velocity Mode (0x6060:00 = 0x03)
            self.send_sdo(node, 0x6060, 0x00, 0x03, size=1)

            # Configure Profile Acceleration (0x6083:00) and Deceleration (0x6084:00)
            self.send_sdo(node, 0x6083, 0x00, 1000000, size=4)
            self.send_sdo(node, 0x6084, 0x00, 1000000, size=4)

            # Configure RPDO22 (Velocity Command):
            # 0x1415:01 (COB-ID) = 0x280 + node (0x281, 0x282, 0x283, 0x284)
            self.send_sdo(node, 0x1415, 0x01, 0x280 + node, size=4)
            # 0x1415:02 (Transmission Type) = 0xFE (Asynchronous / Immediate)
            self.send_sdo(node, 0x1415, 0x02, 0xFE, size=1)

            # Configure TPDO22 (Velocity Feedback):
            # 0x1815:01 (COB-ID) = 0x3F0 + node (0x3F1, 0x3F2, 0x3F3, 0x3F4)
            self.send_sdo(node, 0x1815, 0x01, 0x3F0 + node, size=4)
            # 0x1815:02 (Transmission Type) = 0x01 (Synchronous cyclic every SYNC)
            self.send_sdo(node, 0x1815, 0x02, 0x01, size=1)

            # Configure TPDO1 (Statusword Feedback):
            # 0x1800:01 (COB-ID) = 0x4A0 + node
            self.send_sdo(node, 0x1800, 0x01, 0x4A0 + node, size=4)
            self.send_sdo(node, 0x1800, 0x02, 0x01, size=1)

            # DS402 Enable Sequence
            self.enable_drive(node)

        self.is_enabled = True
        self.get_logger().info(
            'All 4 drives successfully initialized and enabled in Operation Enabled state.'
        )

    def health_check_callback(self):
        """Continuously monitor drive status and automatically re-arm any drive that trips."""
        if not self.bus or not self.is_enabled:
            return

        # Query Node 1 DC Bus Voltage (SDO Upload Object 0x200F:01)
        self.send_can_message(
            0x601,
            [0x40, 0x0F, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00]
        )
        self.publish_battery_state()

        for node in self.nodes:
            sw = self.status_word[node]
            if sw == 0:
                continue

            # CiA 402 Operation Enabled check: (status & 0x006F) == 0x0027
            # Typically 0x0637 or 0x0237
            is_op_enabled = (sw & 0x006F) == 0x0027

            if not is_op_enabled:
                self.get_logger().warn(
                    f'Drive {node} dropped out of Operation Enabled '
                    f'(Statusword: 0x{sw:04X}). Auto-rearming...',
                    throttle_duration_sec=2.0
                )
                self.enable_drive(node)

    def cmd_vel_callback(self, msg: Twist):
        """Process /cmd_vel with strict safety limits and record target speeds."""
        self.last_cmd_time = time.time()

        # Hard safety clamping on incoming command
        self.target_vx = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        self.target_vy = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.y))
        self.target_wz = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))

        # Immediate deceleration if zero commanded
        if msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0

        now = time.time()
        is_moving = (
            abs(self.target_vx) > 0.001
            or abs(self.target_vy) > 0.001
            or abs(self.target_wz) > 0.001
        )
        if (now - self.last_motion_log_time) > 2.0 and is_moving:
            self.last_motion_log_time = now
            self.get_logger().info(
                f'Safe cmd_vel clamped: vx={self.target_vx:.2f} m/s, '
                f'vy={self.target_vy:.2f} m/s, wz={self.target_wz:.2f} rad/s'
            )

    def control_loop_callback(self):
        """Execute cyclic 50Hz control loop with watchdog, slew ramp, and RPDO22 dispatch."""
        if not self.bus or not self.is_enabled:
            return

        dt = 1.0 / self.control_freq

        # Watchdog: If no /cmd_vel received for > cmd_timeout, active stop immediately
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0

        # Active immediate braking when target is zero (zero glide, active electromagnetic hold)
        if self.target_vx == 0.0 and self.target_vy == 0.0 and self.target_wz == 0.0:
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_wz = 0.0
        else:
            # Slew-rate ramping on linear velocity vx (acceleration ramp only)
            dv_max = self.max_linear_accel * dt
            diff_x = self.target_vx - self.current_vx
            if abs(diff_x) <= dv_max:
                self.current_vx = self.target_vx
            else:
                self.current_vx += dv_max if diff_x > 0 else -dv_max

            # Slew-rate ramping on linear velocity vy (lateral crabbing)
            diff_y = self.target_vy - self.current_vy
            if abs(diff_y) <= dv_max:
                self.current_vy = self.target_vy
            else:
                self.current_vy += dv_max if diff_y > 0 else -dv_max

            # Slew-rate ramping on angular velocity wz
            dw_max = self.max_angular_accel * dt
            diff_w = self.target_wz - self.current_wz
            if abs(diff_w) <= dw_max:
                self.current_wz = self.target_wz
            else:
                self.current_wz += dw_max if diff_w > 0 else -dw_max

        # Mecanum Inverse Kinematics calculated on ramped velocities
        R = self.wheel_diameter / 2.0
        L = (self.track_width + self.wheel_base) / 2.0

        self.target_rad_s[1] = (self.current_vx - self.current_vy - self.current_wz * L) / R
        self.target_rad_s[2] = (self.current_vx + self.current_vy - self.current_wz * L) / R
        self.target_rad_s[3] = (self.current_vx + self.current_vy + self.current_wz * L) / R
        self.target_rad_s[4] = (self.current_vx - self.current_vy + self.current_wz * L) / R

        # Hard wheel angular speed clamp (absolute ceiling)
        max_wheel_rad_s = max(0.5, (self.max_linear_vel / R) * 1.5)
        for node in self.nodes:
            self.target_rad_s[node] = max(
                -max_wheel_rad_s, min(max_wheel_rad_s, self.target_rad_s[node])
            )

        # 1. Dispatch RPDO22 Velocity Commands to each node (COB-ID = 0x280 + node)
        for node in self.nodes:
            rad_s = self.target_rad_s[node]
            # Convert to driver integer units with spin orientation
            cmd_units = int(rad_s * self.dRps2Ref * self.spin_dirs[node])

            # Safety clamp to prevent any extreme buffer overflow
            cmd_units = max(-300000, min(300000, cmd_units))

            # Pack as 4-byte signed integer (little-endian)
            payload = list(struct.pack('<i', cmd_units))
            self.send_can_message(0x280 + node, payload)

        # 2. Transmit CANopen SYNC Pulse (COB-ID 0x080)
        self.send_can_message(0x080, [])

        # 3. Forward Mecanum Kinematics & Odometry Integration
        # Wheel feedback in rad/s: Node 1: FL, Node 2: BL, Node 3: FR, Node 4: BR
        w1 = self.actual_rad_s[1]
        w2 = self.actual_rad_s[2]
        w3 = self.actual_rad_s[3]
        w4 = self.actual_rad_s[4]

        # Forward Kinematics (RB-Kairos Mecanum):
        # vx = (R / 4) * (w1 + w2 + w3 + w4)
        # vy = (R / 4) * (-w1 + w2 + w3 - w4)
        # wz = (R / (4 * L)) * (-w1 - w2 + w3 + w4)
        vx_meas = (R / 4.0) * (w1 + w2 + w3 + w4)
        vy_meas = (R / 4.0) * (-w1 + w2 + w3 - w4)
        wz_meas = (R / (4.0 * L)) * (-w1 - w2 + w3 + w4)

        self.odom_vx = vx_meas
        self.odom_vy = vy_meas
        self.odom_wz = wz_meas

        now = self.get_clock().now()
        dt_odom = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now

        if 0.0 < dt_odom < 0.5:
            delta_yaw = wz_meas * dt_odom
            mid_yaw = self.odom_yaw + (delta_yaw / 2.0)
            delta_x = (vx_meas * math.cos(mid_yaw) - vy_meas * math.sin(mid_yaw)) * dt_odom
            delta_y = (vx_meas * math.sin(mid_yaw) + vy_meas * math.cos(mid_yaw)) * dt_odom
            self.odom_x += delta_x
            self.odom_y += delta_y
            new_yaw = self.odom_yaw + delta_yaw
            self.odom_yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))

        self.publish_odometry(now)

    def publish_odometry(self, stamp):
        """Publish raw Cartesian odometry on /odom_raw."""
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Position
        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (yaw to quaternion)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.odom_yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.odom_yaw / 2.0)

        # Pose covariance: [x, y, z, roll, pitch, yaw]
        odom_msg.pose.covariance = [
            0.001, 0.0,   0.0, 0.0, 0.0, 0.0,
            0.0,   0.001, 0.0, 0.0, 0.0, 0.0,
            0.0,   0.0,   1e6, 0.0, 0.0, 0.0,
            0.0,   0.0,   0.0, 1e6, 0.0, 0.0,
            0.0,   0.0,   0.0, 0.0, 1e6, 0.0,
            0.0,   0.0,   0.0, 0.0, 0.0, 0.03
        ]

        # Twist (robot frame)
        odom_msg.twist.twist.linear.x = self.odom_vx
        odom_msg.twist.twist.linear.y = self.odom_vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.odom_wz

        # Twist covariance
        odom_msg.twist.covariance = [
            0.001, 0.0,   0.0, 0.0, 0.0, 0.0,
            0.0,   0.001, 0.0, 0.0, 0.0, 0.0,
            0.0,   0.0,   1e6, 0.0, 0.0, 0.0,
            0.0,   0.0,   0.0, 1e6, 0.0, 0.0,
            0.0,   0.0,   0.0, 0.0, 1e6, 0.0,
            0.0,   0.0,   0.0, 0.0, 0.0, 0.03
        ]

        self.odom_pub.publish(odom_msg)

        if self.publish_odom_tf:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.odom_x
            t.transform.translation.y = self.odom_y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom_msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def publish_joint_states(self):
        """Publish feedback velocities and positions to /joint_states with dual naming."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Include both standard URDF joint names and legacy robot_ prefixed names
        urdf_wheel_names = [
            'front_left_wheel_joint',
            'back_left_wheel_joint',
            'front_right_wheel_joint',
            'back_right_wheel_joint'
        ]
        legacy_wheel_names = [
            'robot_front_left_wheel_joint',
            'robot_back_left_wheel_joint',
            'robot_front_right_wheel_joint',
            'robot_back_right_wheel_joint'
        ]
        msg.name = urdf_wheel_names + legacy_wheel_names
        pos = [self.actual_pos_rad[n] for n in self.nodes]
        vel = [self.actual_rad_s[n] for n in self.nodes]
        msg.position = pos + pos
        msg.velocity = vel + vel
        self.joint_pub.publish(msg)
        self.wheel_joint_pub.publish(msg)

    def publish_battery_state(self):
        """Publish battery state estimated from AMC drive DC bus voltage."""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(self.battery_voltage)
        msg.present = True

        # Estimation based on 48V battery discharge curve:
        # Full: 53.3V, Nominal: 48.0V, Cutoff: 44.0V
        if self.battery_voltage > 30.0:
            pct = (self.battery_voltage - 44.0) / (53.3 - 44.0)
            msg.percentage = float(max(0.0, min(1.0, pct)))
            if self.battery_voltage >= 53.0:
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
            else:
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.percentage = 0.0
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        self.battery_pub.publish(msg)

    def destroy_node(self):
        """Stop motors and disable power stages."""
        self.running = False
        if self.bus:
            self.get_logger().info('Stopping motors and disabling power stages...')
            for node in self.nodes:
                # Set velocity to 0
                self.send_can_message(0x280 + node, list(struct.pack('<i', 0)))
                # Shutdown DS402 (0x0006)
                self.send_sdo(node, 0x6040, 0x00, 0x0006, size=2)
            self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CANMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, exiting...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
