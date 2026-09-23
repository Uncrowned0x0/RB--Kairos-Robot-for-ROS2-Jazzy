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
SICK TiM5xx Ethernet ROS 2 LaserScan Driver.

Connects to SICK TiM551/561/571 laser scanners over Ethernet (SOPAS CoLa-A on TCP port 2111)
and publishes standard sensor_msgs/msg/LaserScan.

Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
"""

import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SickTimDriver(Node):
    """ROS 2 Driver for SICK TiM5xx Ethernet LiDAR scanners."""

    def __init__(self):
        super().__init__('sick_tim_driver')

        self.declare_parameter('sensor_ip', '192.168.0.10')
        self.declare_parameter('sensor_port', 2111)
        self.declare_parameter('frame_id', 'front_laser_link')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 25.0)

        self.sensor_ip = self.get_parameter('sensor_ip').value
        self.sensor_port = self.get_parameter('sensor_port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.get_logger().info(
            f'Starting SICK TiM5xx driver for {self.sensor_ip}:{self.sensor_port} -> '
            f'{self.scan_topic} ({self.frame_id})'
        )

        self.sock = None
        self.running = True
        self.stream_thread = threading.Thread(target=self.connect_and_stream, daemon=True)
        self.stream_thread.start()

    def connect_and_stream(self):
        """Connect to scanner, subscribe to scandata, and stream."""
        while self.running and rclpy.ok():
            try:
                self.get_logger().info(
                    f'Connecting to SICK LiDAR at {self.sensor_ip}:{self.sensor_port}...'
                )
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(3.0)
                self.sock.connect((self.sensor_ip, self.sensor_port))
                self.get_logger().info('Connected! Starting measurement stream...')

                # Start measurement and subscribe to continuous scandata
                self.sock.sendall(b'\x02sMN LMCstartmeas\x03')
                time.sleep(0.05)
                self.sock.sendall(b'\x02sEN LMDscandata 1\x03')

                buf = b''
                while self.running and rclpy.ok():
                    chunk = self.sock.recv(4096)
                    if not chunk:
                        self.get_logger().warn('TCP socket closed by sensor.')
                        break
                    buf += chunk

                    # Parse all complete telegrams in buffer
                    while b'\x03' in buf:
                        stx = buf.find(b'\x02')
                        etx = buf.find(b'\x03')
                        if stx == -1 or etx == -1:
                            break
                        if etx < stx:
                            buf = buf[etx + 1:]
                            continue

                        telegram = buf[stx + 1:etx]
                        buf = buf[etx + 1:]

                        if b'sSN LMDscandata' in telegram or b'sRA LMDscandata' in telegram:
                            self.parse_and_publish(telegram)

            except Exception as e:
                self.get_logger().warn(
                    f'LiDAR connection error ({self.sensor_ip}): {e}. Retrying in 2.0s...'
                )
                if self.sock:
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                    self.sock = None
                time.sleep(2.0)

    def parse_and_publish(self, raw_bytes: bytes):
        """Parse CoLa-A sSN LMDscandata telegram and publish LaserScan."""
        try:
            tokens = raw_bytes.decode('latin1').split()
            if 'DIST1' not in tokens:
                return

            idx = tokens.index('DIST1')
            # DIST1 scale offset start_ang step num_points data...
            # start_ang and step are in 1/10000 degrees hex
            start_ang_val = int(tokens[idx + 3], 16)
            if start_ang_val > 0x7FFFFFFF:
                start_ang_val -= 0x100000000
            # Optical axis (+X) is at +90 deg in raw SOPAS telegram coordinates.
            # Center the angular aperture (-135.0 deg to +135.0 deg) around 0.0 rad.
            start_ang_deg = (start_ang_val / 10000.0) - 90.0

            step_val = int(tokens[idx + 4], 16)
            step_deg = step_val / 10000.0

            num_points = int(tokens[idx + 5], 16)
            range_tokens = tokens[idx + 6:idx + 6 + num_points]

            ranges = []
            for r_hex in range_tokens:
                r_mm = int(r_hex, 16)
                r_m = r_mm / 1000.0
                if r_m < self.range_min or r_m > self.range_max:
                    ranges.append(float('inf'))
                else:
                    ranges.append(r_m)

            # Check optional RSSI1 (intensities)
            intensities = []
            if 'RSSI1' in tokens:
                idx_rssi = tokens.index('RSSI1')
                num_rssi = int(tokens[idx_rssi + 5], 16)
                rssi_tokens = tokens[idx_rssi + 6:idx_rssi + 6 + num_rssi]
                intensities = [float(int(x, 16)) for x in rssi_tokens]

            # Construct LaserScan
            scan = LaserScan()
            scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = self.frame_id
            scan.angle_min = math.radians(start_ang_deg)
            scan.angle_increment = math.radians(step_deg)
            scan.angle_max = scan.angle_min + scan.angle_increment * (num_points - 1)
            scan.time_increment = 1.0 / (15.0 * max(1, num_points))
            scan.scan_time = 1.0 / 15.0
            scan.range_min = self.range_min
            scan.range_max = self.range_max
            scan.ranges = ranges
            if intensities:
                scan.intensities = intensities

            self.pub.publish(scan)

        except Exception as e:
            self.get_logger().debug(f'Telegram parse error: {e}')

    def destroy_node(self):
        """Cleanup socket on exit."""
        self.running = False
        if self.sock:
            try:
                self.sock.sendall(b'\x02sEN LMDscandata 0\x03')
                self.sock.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SickTimDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
