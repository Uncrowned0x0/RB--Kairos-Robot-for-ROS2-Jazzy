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

"""Unit tests for KairosTeleopJoy PS4 DualShock mapping and safety logic."""

from geometry_msgs.msg import Twist
from kairos_real_bringup.kairos_teleop_joy import KairosTeleopJoy
import rclpy
from sensor_msgs.msg import Joy


def create_test_node():
    """Create and return a configured KairosTeleopJoy instance for testing."""
    if not rclpy.ok():
        rclpy.init()
    node = KairosTeleopJoy()
    return node


def make_joy_msg(axes=None, buttons=None):
    """Generate a populated sensor_msgs/Joy message."""
    msg = Joy()
    # Default 8 axes: [LX, LY, L2, RX, RY, R2, DX, DY]
    msg.axes = axes if axes is not None else [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    # Default 14 buttons: [Sq, X, O, Tri, L1, R1, L2, R2, Share, Opt, L3, R3, PS, Pad]
    msg.buttons = buttons if buttons is not None else [0] * 14
    return msg


def capture_published_twist(node, joy_msg):
    """Feed Joy message into node callback and capture the resulting published Twist."""
    published = []

    def mock_pub(twist: Twist):
        published.append(twist)

    # Swap node.pub.publish with mock
    orig_pub = node.pub.publish
    node.pub.publish = mock_pub
    try:
        node.joy_callback(joy_msg)
    finally:
        node.pub.publish = orig_pub

    if published:
        return published[-1]
    return Twist()


def test_deadzone():
    """Verify deadzone function zeroes small noise and smoothly scales."""
    node = create_test_node()
    node.deadzone = 0.08

    assert node.apply_deadzone(0.0) == 0.0
    assert node.apply_deadzone(0.05) == 0.0
    assert node.apply_deadzone(-0.07) == 0.0
    assert abs(node.apply_deadzone(1.0) - 1.0) < 1e-5
    assert abs(node.apply_deadzone(-1.0) - (-1.0)) < 1e-5
    assert 0.0 < node.apply_deadzone(0.5) < 0.5


def test_deadman_alone_zero_velocity():
    """Verify holding Deadman R1 alone with neutral sticks produces strictly zero velocity."""
    node = create_test_node()
    # Default node: enable_bumper_rotation is False, require_deadman is True
    buttons = [0] * 14
    buttons[5] = 1  # R1 Deadman held
    msg = make_joy_msg(buttons=buttons)
    twist = capture_published_twist(node, msg)

    assert twist.linear.x == 0.0, f'Expected vx == 0, got {twist.linear.x}'
    assert twist.linear.y == 0.0, f'Expected vy == 0, got {twist.linear.y}'
    assert twist.angular.z == 0.0, (
        f'CRITICAL: Deadman button caused spurious rotation! wz={twist.angular.z}'
    )


def test_turbo_alone_zero_velocity():
    """Verify holding Turbo L1 alone with neutral sticks produces strictly zero velocity."""
    node = create_test_node()
    buttons = [0] * 14
    buttons[4] = 1  # L1 Turbo held
    buttons[5] = 1  # R1 Deadman held
    msg = make_joy_msg(buttons=buttons)
    twist = capture_published_twist(node, msg)

    assert twist.linear.x == 0.0
    assert twist.linear.y == 0.0
    assert twist.angular.z == 0.0


def test_deadzone_stick_no_spurious_spin():
    """Verify small stick deflection inside deadzone (< 0.08) produces zero motion and no spin."""
    node = create_test_node()
    # Left stick forward 0.04 (below deadzone 0.08). Deadman R1: button 5 = 1.
    axes = [0.0, 0.04, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    buttons = [0] * 14
    buttons[5] = 1

    msg = make_joy_msg(axes=axes, buttons=buttons)
    twist = capture_published_twist(node, msg)

    assert twist.linear.x == 0.0
    assert twist.linear.y == 0.0
    assert twist.angular.z == 0.0, f'Spurious angular spin inside deadzone: wz={twist.angular.z}'


def test_clean_forward_motion_no_angular_coupling():
    """Verify Left stick forward moves straight forward with zero angular velocity."""
    node = create_test_node()
    # Left stick forward: axis 1 = 1.0. Deadman R1: button 5 = 1.
    axes = [0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    buttons = [0] * 14
    buttons[5] = 1  # R1 held

    msg = make_joy_msg(axes=axes, buttons=buttons)
    twist = capture_published_twist(node, msg)

    assert abs(twist.linear.x - 0.08) < 1e-4, f'Expected vx ~0.08, got {twist.linear.x}'
    assert abs(twist.linear.y) < 1e-6, f'Expected vy == 0, got {twist.linear.y}'
    assert abs(twist.angular.z) < 1e-6, (
        f'CRITICAL: Angular coupling detected! wz={twist.angular.z}'
    )


def test_clean_backward_motion():
    """Verify Left stick backward moves straight backward with zero angular velocity."""
    node = create_test_node()
    axes = [0.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    buttons = [0] * 14
    buttons[5] = 1  # R1 held

    msg = make_joy_msg(axes=axes, buttons=buttons)
    twist = capture_published_twist(node, msg)

    assert abs(twist.linear.x - (-0.08)) < 1e-4
    assert abs(twist.linear.y) < 1e-6
    assert abs(twist.angular.z) < 1e-6


def test_differential_mode_left_stick_steering():
    """Verify in Differential mode (default), Left stick horizontal commands yaw steering."""
    node = create_test_node()
    node.mecanum_mode_toggled = False
    # Left stick Left: axis 0 = 1.0. Deadman R1: button 5 = 1. Triggers unpressed.
    axes = [1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    buttons = [0] * 14
    buttons[5] = 1

    msg = make_joy_msg(axes=axes, buttons=buttons)
    twist = capture_published_twist(node, msg)

    assert abs(twist.linear.x) < 1e-6
    assert abs(twist.linear.y) < 1e-6, f'Expected vy=0 in differential mode, got {twist.linear.y}'
    assert abs(twist.angular.z - 0.20) < 1e-4, f'Expected wz=0.20, got {twist.angular.z}'


def test_clean_lateral_crabbing_mecanum():
    """Verify Left stick horizontal produces clean lateral strafe in Mecanum mode."""
    node = create_test_node()
    node.mecanum_mode_toggled = True

    # Left stick Left: axis 0 = 1.0. Deadman R1: button 5 = 1.
    axes = [1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    buttons = [0] * 14
    buttons[5] = 1

    msg = make_joy_msg(axes=axes, buttons=buttons)
    twist = capture_published_twist(node, msg)

    assert abs(twist.linear.x) < 1e-6
    assert abs(twist.linear.y - 0.08) < 1e-4
    assert abs(twist.angular.z) < 1e-6


def test_clean_right_stick_rotation():
    """Verify Right stick horizontal (axis 3) commands rotation with no linear coupling."""
    node = create_test_node()
    # Right stick Left: axis 3 = 1.0. Deadman R1: button 5 = 1.
    axes = [0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0]
    buttons = [0] * 14
    buttons[5] = 1

    msg = make_joy_msg(axes=axes, buttons=buttons)
    twist = capture_published_twist(node, msg)

    assert abs(twist.linear.x) < 1e-6
    assert abs(twist.linear.y) < 1e-6
    assert abs(twist.angular.z - 0.20) < 1e-4


def test_mecanum_trigger_hold_l2():
    """Verify holding L2 triggers Mecanum crabbing even when default is Differential."""
    node = create_test_node()
    node.mecanum_mode_toggled = False
    node.last_trig_state = True  # Already held past edge

    # Holding L2 (button 6) + R1 (deadman 5) + Left stick left (axis 0 = 1.0)
    buttons = [0] * 14
    buttons[5] = 1
    buttons[6] = 1
    axes = [1.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0]  # L2 analog pulled
    msg = make_joy_msg(axes=axes, buttons=buttons)
    twist = capture_published_twist(node, msg)

    assert abs(twist.linear.x) < 1e-6
    assert abs(twist.linear.y - 0.08) < 1e-4, f'Expected vy=0.08, got {twist.linear.y}'
    assert abs(twist.angular.z) < 1e-6, f'Expected wz=0, got {twist.angular.z}'


def test_mecanum_toggle_r2():
    """Verify R2 button toggles Mecanum mode on and off."""
    node = create_test_node()
    node.mecanum_mode_toggled = False
    node.last_trig_state = False

    # Press R2 (button 7)
    buttons = [0] * 14
    buttons[7] = 1
    msg = make_joy_msg(buttons=buttons)
    node.joy_callback(msg)
    assert node.mecanum_mode_toggled is True, 'R2 failed to toggle Mecanum mode ON'

    # Release R2
    buttons[7] = 0
    msg = make_joy_msg(buttons=buttons)
    node.joy_callback(msg)
    assert node.mecanum_mode_toggled is True, 'Release should keep toggled state'

    # Press R2 again
    buttons[7] = 1
    msg = make_joy_msg(buttons=buttons)
    node.joy_callback(msg)
    assert node.mecanum_mode_toggled is False, 'R2 failed to toggle Mecanum mode OFF'


def test_mecanum_toggle_l2():
    """Verify L2 button also toggles Mecanum mode on and off."""
    node = create_test_node()
    node.mecanum_mode_toggled = False
    node.last_trig_state = False

    # Press L2 (button 6)
    buttons = [0] * 14
    buttons[6] = 1
    msg = make_joy_msg(buttons=buttons)
    node.joy_callback(msg)
    assert node.mecanum_mode_toggled is True, 'L2 failed to toggle Mecanum mode ON'

    # Release L2
    buttons[6] = 0
    msg = make_joy_msg(buttons=buttons)
    node.joy_callback(msg)
    assert node.mecanum_mode_toggled is True, 'Release should keep toggled state'

    # Press L2 again
    buttons[6] = 1
    msg = make_joy_msg(buttons=buttons)
    node.joy_callback(msg)
    assert node.mecanum_mode_toggled is False, 'L2 failed to toggle Mecanum mode OFF'


def test_bumper_rotation_when_explicitly_enabled():
    """Verify bumper rotation works if enable_bumper_rotation is explicitly enabled."""
    node = create_test_node()
    node.enable_bumper_rotation = True
    node.allow_any_deadman = True

    # L1 alone: button 4 = 1
    buttons_l1 = [0] * 14
    buttons_l1[4] = 1
    msg_l1 = make_joy_msg(buttons=buttons_l1)
    twist_l1 = capture_published_twist(node, msg_l1)

    assert abs(twist_l1.linear.x) < 1e-6
    assert abs(twist_l1.linear.y) < 1e-6
    assert abs(twist_l1.angular.z - 0.20) < 1e-4, f'Expected wz=0.20, got {twist_l1.angular.z}'

    # R1 alone: button 5 = 1
    buttons_r1 = [0] * 14
    buttons_r1[5] = 1
    msg_r1 = make_joy_msg(buttons=buttons_r1)
    twist_r1 = capture_published_twist(node, msg_r1)

    assert abs(twist_r1.linear.x) < 1e-6
    assert abs(twist_r1.linear.y) < 1e-6
    assert abs(twist_r1.angular.z - (-0.20)) < 1e-4, f'Expected wz=-0.20, got {twist_r1.angular.z}'


def test_turbo_scaling():
    """Verify holding L1 with stick input activates turbo velocity ceiling."""
    node = create_test_node()
    axes = [0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    buttons = [0] * 14
    buttons[5] = 1  # R1 Deadman
    buttons[4] = 1  # L1 Turbo

    msg = make_joy_msg(axes=axes, buttons=buttons)
    twist = capture_published_twist(node, msg)

    assert abs(twist.linear.x - 0.15) < 1e-4, f'Expected turbo vx=0.15, got {twist.linear.x}'
    assert abs(twist.angular.z) < 1e-6


def test_deadman_safety_rejection():
    """Verify sticks at maximum are completely rejected when deadman is not pressed."""
    node = create_test_node()
    node.require_deadman = True
    node.allow_any_deadman = False
    node.is_moving = True

    # Sticks pushed fully forward and right, but no buttons pressed
    axes = [1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0]
    buttons = [0] * 14

    msg = make_joy_msg(axes=axes, buttons=buttons)
    twist = capture_published_twist(node, msg)

    assert twist.linear.x == 0.0
    assert twist.linear.y == 0.0
    assert twist.angular.z == 0.0


def test_trigger_axis_isolation():
    """Verify unpressed or deflected trigger axes (2 and 5) never leak into angular velocity."""
    node = create_test_node()
    # Axis 2 (L2) resting at -1.0, Axis 5 (R2) resting at +1.0
    # Left stick forward: Axis 1 = 1.0. Deadman R1: Button 5 = 1.
    axes = [0.0, 1.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    buttons = [0] * 14
    buttons[5] = 1

    msg = make_joy_msg(axes=axes, buttons=buttons)
    twist = capture_published_twist(node, msg)

    assert abs(twist.linear.x - 0.08) < 1e-4
    assert abs(twist.angular.z) < 1e-6, f'Trigger axis leaked into wz: {twist.angular.z}'


def test_dpad_micro_stepping():
    """Verify D-pad provides fine 4 cm/s micro-stepping for docking."""
    node = create_test_node()
    # D-pad Up: Axis 7 = 1.0. Deadman R1: Button 5 = 1.
    axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0]
    buttons = [0] * 14
    buttons[5] = 1

    msg = make_joy_msg(axes=axes, buttons=buttons)
    twist = capture_published_twist(node, msg)

    assert abs(twist.linear.x - 0.04) < 1e-4
    assert abs(twist.angular.z) < 1e-6
