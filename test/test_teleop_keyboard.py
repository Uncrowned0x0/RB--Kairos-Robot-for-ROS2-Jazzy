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

"""Unit tests for KairosSafeKeyboardTeleop logic and safety interlocks."""

import time

from kairos_real_bringup.kairos_teleop_keyboard import KairosSafeKeyboardTeleop
import rclpy


def get_test_keyboard_node():
    """Create and return a configured KairosSafeKeyboardTeleop instance."""
    if not rclpy.ok():
        rclpy.init()
    node = KairosSafeKeyboardTeleop()
    return node


def test_keyboard_init_defaults():
    """Verify default speed parameters and initial stopped state."""
    node = get_test_keyboard_node()
    assert node.linear_speed == 0.08
    assert node.angular_speed == 0.20
    assert node.max_lin == 0.15
    assert node.min_lin == 0.03
    assert node.vx == 0.0
    assert node.vy == 0.0
    assert node.wz == 0.0
    assert node.action_desc == 'STOPPED (Ready)'


def test_keyboard_adjust_speed_never_moves_robot():
    """Verify speed adjustment updates limits while strictly keeping velocities zero."""
    node = get_test_keyboard_node()
    initial_speed = node.linear_speed

    # Adjust speed upwards
    node.adjust_speed(0.02, 0.03)
    assert abs(node.linear_speed - (initial_speed + 0.02)) < 1e-5
    assert node.vx == 0.0
    assert node.vy == 0.0
    assert node.wz == 0.0

    # Adjust speed downwards
    node.adjust_speed(-0.04, -0.05)
    assert abs(node.linear_speed - (initial_speed - 0.02)) < 1e-5
    assert node.vx == 0.0
    assert node.vy == 0.0
    assert node.wz == 0.0


def test_keyboard_speed_clamping():
    """Verify speeds clamp strictly to maximum ceiling and minimum floor."""
    node = get_test_keyboard_node()

    # Exceed maximum
    node.adjust_speed(10.0, 10.0)
    assert node.linear_speed == node.max_lin
    assert node.angular_speed == node.max_ang

    # Drop below minimum
    node.adjust_speed(-20.0, -20.0)
    assert node.linear_speed == node.min_lin
    assert node.angular_speed == node.min_ang


def test_keyboard_emergency_stop():
    """Verify stop_robot immediately zeros all linear and angular velocities."""
    node = get_test_keyboard_node()
    node.handle_command(1.0, 0.0, 0.0, '▲ FORWARD')
    assert node.vx > 0.0

    node.stop_robot('EMERGENCY STOP (SPACEBAR)')
    assert node.vx == 0.0
    assert node.vy == 0.0
    assert node.wz == 0.0
    assert 'EMERGENCY STOP' in node.action_desc


def test_keyboard_directional_commands():
    """Verify forward, backward, left, right, and Mecanum lateral strafe commands."""
    node = get_test_keyboard_node()

    # Forward
    node.handle_command(1.0, 0.0, 0.0, '▲ FORWARD')
    assert abs(node.vx - node.linear_speed) < 1e-5
    assert node.vy == 0.0
    assert node.wz == 0.0

    # Backward
    node.handle_command(-1.0, 0.0, 0.0, '▼ BACKWARD')
    assert abs(node.vx - (-node.linear_speed)) < 1e-5
    assert node.vy == 0.0
    assert node.wz == 0.0

    # Turn Left
    node.handle_command(0.0, 0.0, 1.0, '◄ TURN LEFT')
    assert node.vx == 0.0
    assert node.vy == 0.0
    assert abs(node.wz - node.angular_speed) < 1e-5

    # Turn Right
    node.handle_command(0.0, 0.0, -1.0, '► TURN RIGHT')
    assert node.vx == 0.0
    assert node.vy == 0.0
    assert abs(node.wz - (-node.angular_speed)) < 1e-5

    # Mecanum Strafe Left
    node.handle_command(0.0, 1.0, 0.0, '◄◄ STRAFE LEFT')
    assert node.vx == 0.0
    assert abs(node.vy - node.linear_speed) < 1e-5
    assert node.wz == 0.0

    # Mecanum Strafe Right
    node.handle_command(0.0, -1.0, 0.0, '►► STRAFE RIGHT')
    assert node.vx == 0.0
    assert abs(node.vy - (-node.linear_speed)) < 1e-5
    assert node.wz == 0.0


def test_keyboard_deadman_timeout():
    """Verify deadman timeout halts velocities when key events cease."""
    node = get_test_keyboard_node()
    node.deadman_timeout = 0.05
    node.handle_command(1.0, 0.0, 0.0, '▲ FORWARD')
    assert node.vx > 0.0

    # Wait beyond deadman timeout
    time.sleep(0.07)
    node.publish_twist()

    assert node.vx == 0.0
    assert node.vy == 0.0
    assert node.wz == 0.0
    assert node.action_desc == 'STOPPED (Key released)'
