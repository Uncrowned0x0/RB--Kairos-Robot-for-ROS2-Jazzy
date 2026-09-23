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
RB-KAIROS LED Signaling Driver Node.

Monitors robot movement, battery voltage, and operational state,
and sends corresponding color status commands to the Teensyduino LED controller
on /dev/ttyUSB_LEDS (or /dev/ttyACM0).

Color Codes:
 - Green: Operational / Idle
 - Blue: Active motion (driving)
 - Yellow: Low battery warning (<20% / <45.0V)
 - Red: Emergency stop / Fault state

Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
"""

import os
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import serial
from std_msgs.msg import Bool, String


class KairosLedDriver(Node):
    """Controls RB-KAIROS status LED strips via Teensy serial link."""

    def __init__(self):
        super().__init__('kairos_led_driver')

        default_port = '/dev/ttyUSB_LEDS' if os.path.exists('/dev/ttyUSB_LEDS') else '/dev/ttyACM0'

        self.declare_parameter('serial_port', default_port)
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('update_rate', 5.0)  # 5 Hz
        self.declare_parameter('motion_threshold', 0.01)

        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baud = self.get_parameter('serial_baud').value
        self.update_rate = float(self.get_parameter('update_rate').value)
        self.motion_threshold = float(self.get_parameter('motion_threshold').value)

        # State tracking
        self.is_moving = False
        self.low_battery = False
        self.emergency_stop = False
        self.last_cmd_vel_time = 0.0
        self.current_state = 'IDLE'
        self.last_state_sent = None

        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10
        )
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_cb, 10
        )
        self.estop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.estop_cb, 10
        )
        self.custom_state_sub = self.create_subscription(
            String, '/led_state', self.custom_state_cb, 10
        )

        # Serial port
        self.serial_conn = None
        self.running = True
        self.lock = threading.Lock()

        # Update timer
        timer_period = 1.0 / max(1.0, self.update_rate)
        self.timer = self.create_timer(timer_period, self.update_led_state)

        self.get_logger().info(
            f'KairosLedDriver initialized on {self.serial_port} @ {self.serial_baud}'
        )

    def _open_serial(self):
        """Try opening primary and fallback ports."""
        ports = [self.serial_port]
        if self.serial_port != '/dev/ttyACM0' and os.path.exists('/dev/ttyACM0'):
            ports.append('/dev/ttyACM0')
        if os.path.exists('/dev/ttyUSB_LEDS') and '/dev/ttyUSB_LEDS' not in ports:
            ports.insert(0, '/dev/ttyUSB_LEDS')

        for p in ports:
            try:
                s = serial.Serial(p, self.serial_baud, timeout=0.5)
                self.get_logger().info(f'Connected to LED Teensy on {p}')
                return s
            except Exception:
                pass
        return None

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_vel_time = time.time()
        speed = abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z)
        self.is_moving = speed > self.motion_threshold

    def battery_cb(self, msg: BatteryState):
        if msg.voltage > 20.0:
            self.low_battery = msg.voltage < 45.0 or (msg.percentage > 0 and msg.percentage < 0.20)
        else:
            self.low_battery = False

    def estop_cb(self, msg: Bool):
        self.emergency_stop = msg.data

    def custom_state_cb(self, msg: String):
        with self.lock:
            self.current_state = msg.data.upper()

    def update_led_state(self):
        """Determine highest-priority state and transmit command."""
        # Timeout motion if no /cmd_vel for > 0.5s
        if time.time() - self.last_cmd_vel_time > 0.5:
            self.is_moving = False

        # Priority resolution
        if self.emergency_stop:
            desired_state = 'EMERGENCY'
            color_cmd = b'COLOR:RED\n'
        elif self.low_battery:
            desired_state = 'LOW_BATT'
            color_cmd = b'COLOR:YELLOW\n'
        elif self.is_moving:
            desired_state = 'MOVING'
            color_cmd = b'COLOR:BLUE\n'
        else:
            desired_state = 'IDLE'
            color_cmd = b'COLOR:GREEN\n'

        with self.lock:
            if self.current_state not in ['IDLE', 'MOVING', 'LOW_BATT', 'EMERGENCY']:
                desired_state = self.current_state
                color_cmd = f'COLOR:{desired_state}\n'.encode()

        if desired_state != self.last_state_sent:
            self.last_state_sent = desired_state
            self.get_logger().info(f'LED state updated to -> {desired_state}')

            # Transmit to serial if available
            if self.serial_conn is None or not self.serial_conn.is_open:
                self.serial_conn = self._open_serial()

            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.write(color_cmd)
                    self.serial_conn.flush()
                except Exception as e:
                    self.get_logger().debug(f'LED serial send error: {e}')
                    try:
                        self.serial_conn.close()
                    except Exception:
                        pass
                    self.serial_conn = None

    def destroy_node(self):
        self.running = False
        if self.serial_conn:
            try:
                # Set to OFF or GREEN on exit
                self.serial_conn.write(b'COLOR:GREEN\n')
                self.serial_conn.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KairosLedDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
