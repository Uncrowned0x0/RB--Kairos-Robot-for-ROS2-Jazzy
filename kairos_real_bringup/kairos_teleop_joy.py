#!/usr/bin/env python3
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
ROS 2 Node for teleoperating RB-KAIROS mobile base using a PS4 DualShock gamepad.

Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
"""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class KairosTeleopJoy(Node):
    """PS4 DualShock Teleoperation node with Mecanum mode and Deadman safety."""

    def __init__(self):
        """Initialize the teleop joy node with parameters and publishers."""
        super().__init__('kairos_teleop_joy')

        # Axis mapping for Linux joy_node / DualShock 4
        self.declare_parameter('axis_linear_x', 1)       # Left stick vertical: v_x
        self.declare_parameter('axis_linear_y', 0)       # Left stick horizontal: v_y
        self.declare_parameter('axis_angular_z', 3)      # Right stick horizontal: omega_z
        self.declare_parameter('axis_mecanum_l2', 2)     # L2 analog trigger
        self.declare_parameter('axis_mecanum_r2', 5)     # R2 analog trigger
        self.declare_parameter('axis_dpad_x', 6)         # D-pad horizontal: fine strafe
        self.declare_parameter('axis_dpad_y', 7)         # D-pad vertical: fine forward

        # Button mapping
        self.declare_parameter('button_deadman', 5)         # R1: Primary Deadman switch
        self.declare_parameter('button_deadman_alt', 6)     # L2: Alternative Deadman switch
        self.declare_parameter('button_turbo', 4)           # L1: Turbo switch
        self.declare_parameter('button_mecanum_toggle', 7)  # R2: Mecanum mode toggle / trigger
        self.declare_parameter('button_mecanum_l2', 6)      # L2: Mecanum mode trigger / toggle
        self.declare_parameter('button_rotate_left', 4)     # L1: In-place rotate left
        self.declare_parameter('button_rotate_right', 5)    # R1: In-place rotate right

        # Speed scaling parameters (m/s and rad/s)
        self.declare_parameter('scale_linear_x', 0.08)
        self.declare_parameter('scale_linear_y', 0.08)
        self.declare_parameter('scale_angular_z', 0.20)
        self.declare_parameter('scale_linear_x_turbo', 0.15)
        self.declare_parameter('scale_linear_y_turbo', 0.15)
        self.declare_parameter('scale_angular_z_turbo', 0.30)
        self.declare_parameter('scale_fine_linear', 0.04)

        # Operational behavior
        self.declare_parameter('require_deadman', True)
        self.declare_parameter('allow_any_deadman', False)
        self.declare_parameter('allow_alt_deadman', False)
        self.declare_parameter('enable_bumper_rotation', False)
        self.declare_parameter('mecanum_mode_default', False)
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('cmd_timeout', 0.35)

        # Retrieve parameters
        self.axis_linear_x = self.get_parameter('axis_linear_x').value
        self.axis_linear_y = self.get_parameter('axis_linear_y').value
        self.axis_angular_z = self.get_parameter('axis_angular_z').value
        self.axis_mecanum_l2 = self.get_parameter('axis_mecanum_l2').value
        self.axis_mecanum_r2 = self.get_parameter('axis_mecanum_r2').value
        self.axis_dpad_x = self.get_parameter('axis_dpad_x').value
        self.axis_dpad_y = self.get_parameter('axis_dpad_y').value

        self.button_deadman = self.get_parameter('button_deadman').value
        self.button_deadman_alt = self.get_parameter('button_deadman_alt').value
        self.button_turbo = self.get_parameter('button_turbo').value
        self.button_mecanum_toggle = self.get_parameter('button_mecanum_toggle').value
        self.button_mecanum_l2 = self.get_parameter('button_mecanum_l2').value
        self.button_rotate_left = self.get_parameter('button_rotate_left').value
        self.button_rotate_right = self.get_parameter('button_rotate_right').value

        self.scale_linear_x = float(self.get_parameter('scale_linear_x').value)
        self.scale_linear_y = float(self.get_parameter('scale_linear_y').value)
        self.scale_angular_z = float(self.get_parameter('scale_angular_z').value)
        self.scale_linear_x_turbo = float(self.get_parameter('scale_linear_x_turbo').value)
        self.scale_linear_y_turbo = float(self.get_parameter('scale_linear_y_turbo').value)
        self.scale_angular_z_turbo = float(self.get_parameter('scale_angular_z_turbo').value)
        self.scale_fine_linear = float(self.get_parameter('scale_fine_linear').value)

        self.require_deadman = self.get_parameter('require_deadman').value
        self.allow_any_deadman = self.get_parameter('allow_any_deadman').value
        self.allow_alt_deadman = self.get_parameter('allow_alt_deadman').value
        self.enable_bumper_rotation = self.get_parameter('enable_bumper_rotation').value
        self.mecanum_mode_toggled = self.get_parameter('mecanum_mode_default').value
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        # Publisher and Subscriber
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Internal state
        self.last_joy_time = time.time()
        self.last_trig_state = False
        self.last_r2_state = False
        self.is_moving = False
        self.last_log_time = 0.0

        # Watchdog timer (50ms cyclic check)
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_callback)

        mode_str = 'MECANUM (Holonomic)' if self.mecanum_mode_toggled else 'DIFFERENTIAL'
        self.get_logger().info(
            f'PS4 Teleoperation node initialized. Initial mode: {mode_str}, '
            f'Deadman required: {self.require_deadman}'
        )

    def apply_deadzone(self, value: float) -> float:
        """Apply smooth deadzone scaling to an analog axis input."""
        if abs(value) < self.deadzone:
            return 0.0
        sign = math.copysign(1.0, value)
        return sign * ((abs(value) - self.deadzone) / (1.0 - self.deadzone))

    @staticmethod
    def _get_button(buttons: list, idx: int) -> bool:
        """Safely retrieve button state with bounds check."""
        if 0 <= idx < len(buttons):
            return bool(buttons[idx])
        return False

    @staticmethod
    def _get_axis(axes: list, idx: int) -> float:
        """Safely retrieve axis state with bounds check."""
        if 0 <= idx < len(axes):
            return float(axes[idx])
        return 0.0

    def joy_callback(self, msg: Joy):
        """Process incoming Joy message and compute Twist command."""
        self.last_joy_time = time.time()

        # Read button inputs safely
        btn_l1 = self._get_button(msg.buttons, self.button_turbo)
        btn_r1 = self._get_button(msg.buttons, self.button_deadman)

        # Detect L2 and R2 triggers (as button or analog deflection < -0.2)
        btn_l2 = self._get_button(msg.buttons, self.button_mecanum_l2)
        btn_r2 = self._get_button(msg.buttons, self.button_mecanum_toggle)
        axis_l2 = self._get_axis(msg.axes, self.axis_mecanum_l2) < -0.2
        axis_r2 = self._get_axis(msg.axes, self.axis_mecanum_r2) < -0.2

        trig_l2 = btn_l2 or axis_l2
        trig_r2 = btn_r2 or axis_r2
        trig_active = trig_r2 or trig_l2

        # Toggle Mecanum mode on rising edge of either R2 or L2 trigger
        if trig_active and not self.last_trig_state:
            self.mecanum_mode_toggled = not self.mecanum_mode_toggled
            st = (
                'ENABLED (Holonomic strafe active via left stick)'
                if self.mecanum_mode_toggled else 'DIFFERENTIAL (Standard steering)'
            )
            trigger_name = 'R2' if trig_r2 else 'L2'
            self.get_logger().info(f'Kinematic mode switched by {trigger_name} -> {st}')
        self.last_trig_state = trig_active
        self.last_r2_state = trig_r2

        # Effective Mecanum mode: active if toggled ON OR momentarily held via L2 or R2
        effective_mecanum = self.mecanum_mode_toggled or trig_active

        # Check deadman safety switch
        if self.require_deadman:
            if self.allow_any_deadman:
                deadman_active = btn_r1 or btn_l1 or trig_l2 or trig_r2
            elif self.allow_alt_deadman:
                deadman_active = btn_r1 or trig_l2
            else:
                deadman_active = btn_r1
        else:
            deadman_active = True

        if not deadman_active:
            if self.is_moving:
                self.publish_zero()
                self.is_moving = False
            return

        # Read analog sticks
        raw_lx = self._get_axis(msg.axes, self.axis_linear_y)   # Left stick X (Horizontal)
        raw_ly = self._get_axis(msg.axes, self.axis_linear_x)   # Left stick Y (Vertical)
        raw_rx = self._get_axis(msg.axes, self.axis_angular_z)  # Right stick X (Horizontal)
        raw_dx = self._get_axis(msg.axes, self.axis_dpad_x)     # D-pad X
        raw_dy = self._get_axis(msg.axes, self.axis_dpad_y)     # D-pad Y

        lx = self.apply_deadzone(raw_lx)
        ly = self.apply_deadzone(raw_ly)
        rx = self.apply_deadzone(raw_rx)

        sticks_active = (abs(lx) > 0.0 or abs(ly) > 0.0 or abs(rx) > 0.0)
        dpad_active = (abs(raw_dx) > 0.5 or abs(raw_dy) > 0.5)

        # Turbo speed scaling when holding L1 alongside stick or D-pad input
        is_turbo = btn_l1 and (sticks_active or dpad_active)

        scale_x = self.scale_linear_x_turbo if is_turbo else self.scale_linear_x
        scale_y = self.scale_linear_y_turbo if is_turbo else self.scale_linear_y
        scale_w = self.scale_angular_z_turbo if is_turbo else self.scale_angular_z

        # 1. Linear velocity X: Forward / Backward (clean, strictly no angular coupling)
        vx = ly * scale_x
        if abs(raw_dy) > 0.5:
            vx += math.copysign(self.scale_fine_linear, raw_dy)

        # 2. Linear velocity Y (Lateral crabbing) and Angular velocity Z (Rotation)
        vy = 0.0
        wz = 0.0

        if effective_mecanum:
            # Mecanum Holonomic Mode:
            # Left stick horizontal commands lateral crabbing vy
            vy = lx * scale_y
            if abs(raw_dx) > 0.5:
                vy += math.copysign(self.scale_fine_linear, raw_dx)
            # Right stick horizontal commands rotation wz
            wz = rx * scale_w
        else:
            # Differential / Standard Mode:
            # No lateral strafe
            vy = 0.0
            # Rotation commanded from right stick or left stick horizontal
            if abs(rx) > 0.0:
                wz = rx * scale_w
            else:
                wz = lx * scale_w

        # 3. Optional Bumper In-Place Rotation (Disabled by default to protect deadman safety)
        # Active only if enable_bumper_rotation is explicitly enabled AND sticks are neutral
        if self.enable_bumper_rotation and not sticks_active and not dpad_active:
            btn_rot_l = self._get_button(msg.buttons, self.button_rotate_left)
            btn_rot_r = self._get_button(msg.buttons, self.button_rotate_right)
            if btn_rot_l and not btn_rot_r:
                wz = self.scale_angular_z
            elif btn_rot_r and not btn_rot_l:
                wz = -self.scale_angular_z

        # Clamp to max safety ceilings
        vx = max(-self.scale_linear_x_turbo, min(self.scale_linear_x_turbo, vx))
        vy = max(-self.scale_linear_y_turbo, min(self.scale_linear_y_turbo, vy))
        wz = max(-self.scale_angular_z_turbo, min(self.scale_angular_z_turbo, wz))

        # Publish Twist
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        self.pub.publish(twist)

        moving = (abs(vx) > 0.001 or abs(vy) > 0.001 or abs(wz) > 0.001)
        self.is_moving = moving

        now = time.time()
        if moving and (now - self.last_log_time > 2.0):
            self.last_log_time = now
            mode_tag = 'MECANUM' if effective_mecanum else 'DIFF'
            turbo_tag = ' [TURBO]' if is_turbo else ''
            self.get_logger().info(
                f'[{mode_tag}{turbo_tag}] vx={vx:.2f} m/s, vy={vy:.2f} m/s, wz={wz:.2f} rad/s'
            )

    def publish_zero(self):
        """Publish zero velocity Twist command to immediately halt the robot."""
        twist = Twist()
        self.pub.publish(twist)

    def watchdog_callback(self):
        """Watchdog to safely halt the robot if joystick communication drops."""
        if (time.time() - self.last_joy_time) > self.cmd_timeout:
            if self.is_moving:
                self.publish_zero()
                self.is_moving = False


def main(args=None):
    """Run the KairosTeleopJoy node."""
    rclpy.init(args=args)
    node = KairosTeleopJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, exiting...')
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
