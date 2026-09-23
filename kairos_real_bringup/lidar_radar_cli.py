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
Live ASCII Radar Visualization for RB-KAIROS SICK LiDARs.

Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
"""

import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarRadarCli(Node):
    """Subscribes to front and rear laser scans and renders ASCII radar."""

    def __init__(self):
        super().__init__('lidar_radar_cli')
        self.front_scan = None
        self.rear_scan = None

        self.create_subscription(LaserScan, '/front_laser/scan', self.on_front_scan, 5)
        self.create_subscription(LaserScan, '/rear_laser/scan', self.on_rear_scan, 5)

        self.timer = self.create_timer(0.2, self.render_radar)
        self.max_range = 4.0  # 4 meters scale

    def on_front_scan(self, msg: LaserScan):
        self.front_scan = msg

    def on_rear_scan(self, msg: LaserScan):
        self.rear_scan = msg

    def render_radar(self):
        # 60 chars wide x 25 lines high
        W, H = 61, 23
        grid = [[' ' for _ in range(W)] for _ in range(H)]
        cx, cy = W // 2, H // 2

        # Draw range circles (1m, 2m, 3m)
        for r_m, ch in [(1.0, '.'), (2.0, '-'), (3.0, '='), (4.0, '#')]:
            for a in range(0, 360, 5):
                rad = math.radians(a)
                c = int(cx + (r_m / self.max_range) * (W // 2) * math.cos(rad))
                r = int(cy - (r_m / self.max_range) * (H // 2) * math.sin(rad))
                if 0 <= c < W and 0 <= r < H:
                    grid[r][c] = ch

        # Plot Robot center
        grid[cy][cx] = 'R'

        min_f, min_f_ang = 99.0, 0.0
        if self.front_scan and self.front_scan.ranges:
            for i, d in enumerate(self.front_scan.ranges):
                if 0.1 < d <= self.max_range:
                    ang = (
                        self.front_scan.angle_min
                        + i * self.front_scan.angle_increment
                    )
                    # Front scanner mounted front-right, optical axis facing -45 deg, upside-down
                    theta = -math.pi / 4.0 - ang
                    if d < min_f:
                        min_f = d
                        min_f_ang = math.degrees(theta)

                    x_fwd = 0.29 + d * math.cos(theta)
                    y_left = -0.23 + d * math.sin(theta)

                    col = int(cx - (y_left / self.max_range) * (W // 2))
                    row = int(cy - (x_fwd / self.max_range) * (H // 2))
                    if 0 <= col < W and 0 <= row < H:
                        grid[row][col] = '█'

        min_r, min_r_ang = 99.0, 0.0
        if self.rear_scan and self.rear_scan.ranges:
            for i, d in enumerate(self.rear_scan.ranges):
                if 0.1 < d <= self.max_range:
                    ang = (
                        self.rear_scan.angle_min
                        + i * self.rear_scan.angle_increment
                    )
                    # Rear scanner mounted back-left, optical axis facing +135 deg, upside-down
                    theta = 3.0 * math.pi / 4.0 - ang
                    if d < min_r:
                        min_r = d
                        min_r_ang = math.degrees(theta)

                    x_fwd = -0.29 + d * math.cos(theta)
                    y_left = 0.23 + d * math.sin(theta)

                    col = int(cx - (y_left / self.max_range) * (W // 2))
                    row = int(cy - (x_fwd / self.max_range) * (H // 2))
                    if 0 <= col < W and 0 <= row < H:
                        grid[row][col] = '▒'

        out = []
        out.append('\033[H\033[2J')
        out.append('\033[1;36m=== RB-KAIROS 360° SICK TiM5xx LiDAR RADAR (Scale: 4m) ===\033[0m')
        if min_f < 90:
            f_str = f'\033[1;32m{min_f:.2f} m\033[0m (@ {min_f_ang:.0f}°)'
        else:
            f_str = '\033[33mClear (>4m)\033[0m'

        if min_r < 90:
            r_str = f'\033[1;33m{min_r:.2f} m\033[0m (@ {min_r_ang:.0f}°)'
        else:
            r_str = '\033[33mClear (>4m)\033[0m'

        out.append(f' Front Obstacle (█) : {f_str}  |  Rear Obstacle (▒) : {r_str}')
        out.append('+' + '-' * W + '+')
        for row in grid:
            out.append('|' + ''.join(row) + '|')
        out.append('+' + '-' * W + '+')
        legend = (
            '\033[2m Legend: [R]=Robot  █=Front LiDAR  '
            '▒=Rear LiDAR  .=1m -=2m ==3m #=4m\033[0m'
        )
        out.append(legend)
        out.append('\033[2m (Press Ctrl+C to exit radar)\033[0m')
        sys.stdout.write('\n'.join(out) + '\n')
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = LidarRadarCli()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
