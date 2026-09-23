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
Safe interactive keyboard teleoperation node for RB-KAIROS mobile base.

Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
"""

import os
import select
import sys
import termios
import threading
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


BANNER = """\
\033[1;36m╔══════════════════════════════════════════════════════════════╗
║        🎮 RB-KAIROS SAFE KEYBOARD TELEOP (AZERTY/QWERTY)     ║
╚══════════════════════════════════════════════════════════════╝\033[0m
 \033[1;32mMotion Commands (Deadman safety: stops on key release)\033[0m
   • \033[1mUp Arrow\033[0m    or \033[1mZ / W / I\033[0m : Forward
   • \033[1mDown Arrow\033[0m  or \033[1mS / K\033[0m     : Backward
   • \033[1mLeft Arrow\033[0m  or \033[1mQ / A / J\033[0m : Turn Left
   • \033[1mRight Arrow\033[0m or \033[1mD / L\033[0m     : Turn Right
   • \033[1mShift + Arrows\033[0m or \033[1mU / O\033[0m : Mecanum Lateral Strafe (Left / Right)

 \033[1;33mSpeed Adjustment (NEVER MOVES THE ROBOT)\033[0m
   • \033[1m+\033[0m or \033[1mP\033[0m : Increase speed (+0.02 m/s) [Max ceiling = 0.15 m/s]
   • \033[1m-\033[0m or \033[1mM\033[0m : Decrease speed (-0.02 m/s) [Min floor = 0.03 m/s]

 \033[1;31mImmediate Emergency Stop\033[0m
   • \033[1mSPACEBAR\033[0m or \033[1mX\033[0m : IMMEDIATE STOP (0 m/s)
   • \033[1mCtrl+C\033[0m : Clean exit safely halting the robot
"""


class KairosSafeKeyboardTeleop(Node):
    """ROS 2 Node for safe interactive keyboard teleoperation."""

    def __init__(self):
        super().__init__('kairos_teleop_keyboard')

        self.declare_parameter('default_linear_speed', 0.08)
        self.declare_parameter('default_angular_speed', 0.20)
        self.declare_parameter('max_linear_speed', 0.15)
        self.declare_parameter('min_linear_speed', 0.03)
        self.declare_parameter('max_angular_speed', 0.30)
        self.declare_parameter('min_angular_speed', 0.05)
        self.declare_parameter('deadman_timeout', 0.25)

        self.linear_speed = float(self.get_parameter('default_linear_speed').value)
        self.angular_speed = float(self.get_parameter('default_angular_speed').value)
        self.max_lin = float(self.get_parameter('max_linear_speed').value)
        self.min_lin = float(self.get_parameter('min_linear_speed').value)
        self.max_ang = float(self.get_parameter('max_angular_speed').value)
        self.min_ang = float(self.get_parameter('min_angular_speed').value)
        self.deadman_timeout = float(self.get_parameter('deadman_timeout').value)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_key_time = 0.0
        self.action_desc = 'STOPPED (Ready)'
        self.running = True

        # 20Hz cyclic publishing loop
        self.timer = self.create_timer(0.05, self.publish_twist)

    def publish_twist(self):
        """Publish twist to /cmd_vel with deadman timeout enforcement."""
        now = time.time()
        # Enforce deadman timeout: if no key was pressed recently, reset to 0
        if (now - self.last_key_time) > self.deadman_timeout:
            if self.vx != 0.0 or self.vy != 0.0 or self.wz != 0.0:
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                self.action_desc = 'STOPPED (Key released)'
                self.print_status()

        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.angular.z = self.wz
        self.pub.publish(twist)

    def stop_robot(self, reason='EMERGENCY STOP'):
        """Immediately reset all target velocities to zero."""
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_key_time = 0.0
        self.action_desc = reason
        self.publish_twist()
        self.print_status()

    def adjust_speed(self, delta_lin, delta_ang):
        """Modify speed settings safely without sending motion."""
        # Speed modification strictly zeroes any motion
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.linear_speed = max(self.min_lin, min(self.max_lin, self.linear_speed + delta_lin))
        self.angular_speed = max(self.min_ang, min(self.max_ang, self.angular_speed + delta_ang))
        self.action_desc = f'Speed set to {self.linear_speed * 100:.1f} cm/s'
        self.publish_twist()
        self.print_status()

    def handle_command(self, vx_factor, vy_factor, wz_factor, desc):
        """Process directional movement command."""
        self.last_key_time = time.time()
        self.vx = vx_factor * self.linear_speed
        self.vy = vy_factor * self.linear_speed
        self.wz = wz_factor * self.angular_speed
        self.action_desc = desc
        self.print_status()

    def print_status(self):
        """Display single-line real-time dashboard in terminal."""
        bar_len = 10
        ratio = (self.linear_speed - self.min_lin) / max(0.001, (self.max_lin - self.min_lin))
        fill = int(ratio * bar_len)
        gauge = '█' * fill + '░' * (bar_len - fill)

        status_line = (
            f'\r\033[K[\033[1;33m{gauge}\033[0m '
            f'\033[1mSpeed: {self.linear_speed * 100:.1f} cm/s\033[0m | '
            f'Rot: {self.angular_speed:.2f} rad/s] -> '
            f'\033[1;32m{self.action_desc}\033[0m'
        )
        sys.stdout.write(status_line)
        sys.stdout.flush()


def read_key(fd):
    """Read a key sequence from standard input non-blockingly."""
    r, _, _ = select.select([fd], [], [], 0.05)
    if not r:
        return None

    ch = os.read(fd, 1).decode('latin-1', errors='ignore')
    if ch == '\x1b':
        # Potential escape sequence
        r2, _, _ = select.select([fd], [], [], 0.05)
        if r2:
            ch2 = os.read(fd, 1).decode('latin-1', errors='ignore')
            if ch2 == '[':
                seq = ''
                while True:
                    r3, _, _ = select.select([fd], [], [], 0.02)
                    if not r3:
                        break
                    c = os.read(fd, 1).decode('latin-1', errors='ignore')
                    seq += c
                    if c.isalpha() or c == '~':
                        break
                return '\x1b[' + seq
            return '\x1b' + ch2
    return ch


def main():
    """Run the keyboard teleoperation node."""
    if not sys.stdin.isatty():
        print('Error: kairos_teleop_keyboard must be run in an interactive terminal.')
        return

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    rclpy.init()
    node = KairosSafeKeyboardTeleop()

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    os.system('clear')
    print(BANNER)
    node.print_status()

    try:
        tty.setraw(fd)
        while rclpy.ok() and node.running:
            key = read_key(fd)
            if key is None:
                continue

            # Ctrl+C
            if key in ('\x03', '\x04'):
                break

            # Emergency Stop keys: Spacebar or X or C
            elif key in (' ', 'x', 'X', 'c', 'C'):
                node.stop_robot('EMERGENCY STOP (SPACEBAR)')

            # Speed Adjustments (+ / - / p / m)
            elif key in ('+', 'p', 'P', '='):
                node.adjust_speed(+0.02, +0.03)
            elif key in ('-', 'm', 'M'):
                node.adjust_speed(-0.02, -0.03)

            # Directional - Arrow Keys
            elif key == '\x1b[A':  # Up Arrow
                node.handle_command(1.0, 0.0, 0.0, '▲ FORWARD')
            elif key == '\x1b[B':  # Down Arrow
                node.handle_command(-1.0, 0.0, 0.0, '▼ BACKWARD')
            elif key == '\x1b[D':  # Left Arrow
                node.handle_command(0.0, 0.0, 1.0, '◄ TURN LEFT')
            elif key == '\x1b[C':  # Right Arrow
                node.handle_command(0.0, 0.0, -1.0, '► TURN RIGHT')

            # Shift + Arrow Keys (Crabbing Mecanum)
            elif key == '\x1b[1;2D':  # Shift+Left / Crab Left
                node.handle_command(0.0, 1.0, 0.0, '◄◄ STRAFE LEFT')
            elif key == '\x1b[1;2C':  # Shift+Right / Crab Right
                node.handle_command(0.0, -1.0, 0.0, '►► STRAFE RIGHT')

            # Directional - Letters (AZERTY & QWERTY & IJKL)
            elif key in ('z', 'Z', 'w', 'W', 'i', 'I'):  # Forward
                node.handle_command(1.0, 0.0, 0.0, '▲ FORWARD')
            elif key in ('s', 'S', 'k', 'K'):  # Backward
                node.handle_command(-1.0, 0.0, 0.0, '▼ BACKWARD')
            elif key in ('q', 'Q', 'a', 'A', 'j'):  # Turn Left
                node.handle_command(0.0, 0.0, 1.0, '◄ TURN LEFT')
            elif key in ('d', 'D', 'l'):  # Turn Right
                node.handle_command(0.0, 0.0, -1.0, '► TURN RIGHT')

            # Crabbing Mecanum (U / O / Shift+J / Shift+L)
            elif key in ('u', 'U', 'J'):  # Lateral Left
                node.handle_command(0.0, 1.0, 0.0, '◄◄ STRAFE LEFT')
            elif key in ('o', 'O', 'L'):  # Lateral Right
                node.handle_command(0.0, -1.0, 0.0, '►► STRAFE RIGHT')

    except Exception as e:
        print(f'\nTeleoperation error: {e}')
    finally:
        # Restore terminal settings
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        # Transmit zero velocity multiple times for safety
        stop_msg = Twist()
        for _ in range(5):
            node.pub.publish(stop_msg)
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()
        print('\n\033[1;32mTeleoperation safely closed. Robot halted.\033[0m')


if __name__ == '__main__':
    main()
