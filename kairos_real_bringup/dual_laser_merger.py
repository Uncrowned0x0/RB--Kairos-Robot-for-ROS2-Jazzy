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
Dual SICK LiDAR 360° Merger Node.

Fuses /front_laser/scan (front-right) and /rear_laser/scan (back-left)
into a unified 360° /scan in base_footprint frame for Nav2 and SLAM.

Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_geometry_msgs  # noqa: F401
import tf2_ros


class DualLaserMerger(Node):
    """Fuses two opposing 270° LiDAR scans into a single continuous 360° LaserScan."""

    def __init__(self):
        super().__init__('dual_laser_merger')

        self.declare_parameter('front_scan_topic', '/front_laser/scan')
        self.declare_parameter('rear_scan_topic', '/rear_laser/scan')
        self.declare_parameter('merged_scan_topic', '/scan')
        self.declare_parameter('destination_frame', 'base_footprint')
        self.declare_parameter('scan_rate', 15.0)
        self.declare_parameter('angle_increment_deg', 0.5)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 25.0)

        self.front_topic = self.get_parameter('front_scan_topic').value
        self.rear_topic = self.get_parameter('rear_scan_topic').value
        self.merged_topic = self.get_parameter('merged_scan_topic').value
        self.dest_frame = self.get_parameter('destination_frame').value
        self.scan_rate = float(self.get_parameter('scan_rate').value)
        self.angle_inc = math.radians(float(self.get_parameter('angle_increment_deg').value))
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        # Precompute bins covering [-pi, pi]
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.num_bins = int(round((self.angle_max - self.angle_min) / self.angle_inc))

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Scans storage
        self.latest_front_scan = None
        self.latest_rear_scan = None

        # Cached transforms (translation x, y, rotation yaw)
        self.cached_tf = {}

        # Subscriptions
        self.sub_front = self.create_subscription(
            LaserScan, self.front_topic, self.front_scan_cb, 10
        )
        self.sub_rear = self.create_subscription(
            LaserScan, self.rear_topic, self.rear_scan_cb, 10
        )

        # Publisher
        self.pub_merged = self.create_publisher(LaserScan, self.merged_topic, 10)

        # Merge timer at sensor frequency
        timer_period = 1.0 / max(1.0, self.scan_rate)
        self.merge_timer = self.create_timer(timer_period, self.merge_callback)

        self.get_logger().info(
            f'DualLaserMerger started: {self.front_topic} + {self.rear_topic} -> '
            f'{self.merged_topic} ({self.dest_frame}, {self.num_bins} bins)'
        )

    def front_scan_cb(self, msg: LaserScan):
        self.latest_front_scan = msg

    def rear_scan_cb(self, msg: LaserScan):
        self.latest_rear_scan = msg

    def _get_transform(self, source_frame: str):
        """Get 2D transform (tx, ty, r00, r01, r10, r11) from source to destination."""
        if source_frame in self.cached_tf:
            return self.cached_tf[source_frame]

        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.dest_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            tx = tf_stamped.transform.translation.x
            ty = tf_stamped.transform.translation.y
            q = tf_stamped.transform.rotation

            # Precompute in-plane projection from 3D quaternion:
            # For a 2D laser point [lx, ly, 0]^T in sensor frame:
            #   gx = tx + r00 * lx + r01 * ly
            #   gy = ty + r10 * lx + r11 * ly
            # Correctly handles both upright and inverted (pitch=-pi) sensors.
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
            r01 = 2.0 * (qx * qy - qw * qz)
            r10 = 2.0 * (qx * qy + qw * qz)
            r11 = 1.0 - 2.0 * (qx * qx + qz * qz)

            self.cached_tf[source_frame] = (tx, ty, r00, r01, r10, r11)
            self.get_logger().info(
                f'Resolved transform {source_frame} -> {self.dest_frame}: '
                f'pos=({tx:.3f}, {ty:.3f}), rot=[{r00:.2f}, {r01:.2f}, {r10:.2f}, {r11:.2f}]'
            )
            return self.cached_tf[source_frame]
        except Exception:
            return None

    def merge_callback(self):
        """Fuse front and rear scans into unified 360° LaserScan."""
        if not self.latest_front_scan and not self.latest_rear_scan:
            return

        now = self.get_clock().now()
        ranges = [float('inf')] * self.num_bins
        intensities = [0.0] * self.num_bins
        has_intensities = False

        # Process each available scan
        for scan in [self.latest_front_scan, self.latest_rear_scan]:
            if scan is None:
                continue

            tf_data = self._get_transform(scan.header.frame_id)
            if tf_data is None:
                continue

            tx, ty, r00, r01, r10, r11 = tf_data
            cur_angle = scan.angle_min
            has_scan_intensities = len(scan.intensities) == len(scan.ranges)

            for i, r in enumerate(scan.ranges):
                valid_r = scan.range_min <= r <= scan.range_max
                if valid_r and not math.isnan(r) and not math.isinf(r):
                    # Local laser beam coordinates
                    lx = r * math.cos(cur_angle)
                    ly = r * math.sin(cur_angle)

                    # Transform into destination frame with full 3D rotation projection
                    gx = tx + r00 * lx + r01 * ly
                    gy = ty + r10 * lx + r11 * ly

                    # Polar coordinates in destination frame
                    dist = math.hypot(gx, gy)
                    if self.range_min <= dist <= self.range_max:
                        phi = math.atan2(gy, gx)
                        bin_offset = (phi - self.angle_min) / self.angle_inc
                        bin_idx = int(round(bin_offset)) % self.num_bins
                        if dist < ranges[bin_idx]:
                            ranges[bin_idx] = dist
                            if has_scan_intensities:
                                intensities[bin_idx] = scan.intensities[i]
                                has_intensities = True

                cur_angle += scan.angle_increment

        # Construct merged scan message
        merged = LaserScan()
        merged.header.stamp = now.to_msg()
        merged.header.frame_id = self.dest_frame
        merged.angle_min = self.angle_min
        merged.angle_max = self.angle_max
        merged.angle_increment = self.angle_inc
        merged.time_increment = 1.0 / (self.scan_rate * self.num_bins)
        merged.scan_time = 1.0 / self.scan_rate
        merged.range_min = self.range_min
        merged.range_max = self.range_max
        merged.ranges = ranges
        if has_intensities:
            merged.intensities = intensities

        self.pub_merged.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = DualLaserMerger()
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
