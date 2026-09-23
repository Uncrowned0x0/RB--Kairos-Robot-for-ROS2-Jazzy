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
Launch RB-KAIROS base components: CAN driver, VectorNav IMU, EKF, and LED signaling.

Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch RB-KAIROS base components: CAN driver, VectorNav IMU, EKF, and LED signaling."""
    pkg_dir = get_package_share_directory('kairos_real_bringup')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    # 1. CANopen Motor Driver (50 Hz control loop, Mecanum odometry on /odom_raw)
    motor_driver_node = Node(
        package='kairos_real_bringup',
        executable='kairos_serial_motor_driver',
        name='kairos_can_motor_driver',
        output='screen',
        parameters=[{
            'can_interface': 'can0',
            'can_bitrate': 1000000,
            'control_freq': 50.0,
            'cmd_timeout': 1.0,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'publish_odom_tf': False  # EKF handles odom -> base_footprint TF
        }]
    )

    # 2. VectorNav VN-100 IMU Driver (publishes /imu/data)
    vectornav_node = Node(
        package='kairos_real_bringup',
        executable='vectornav_driver',
        name='vectornav_driver',
        output='screen',
        parameters=[{
            'serial_port': (
                '/dev/ttyUSB_IMU' if os.path.exists('/dev/ttyUSB_IMU') else '/dev/ttyUSB0'
            ),
            'serial_baud': 921600,
            'frame_id': 'imu_link',
            'tf_ned_to_enu': True
        }]
    )

    # 3. EKF Filter (robot_localization: /odom_raw + /imu/data -> /odometry/filtered & odom TF)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    # 4. LED Signaling Driver (Teensy on /dev/ttyUSB_LEDS or /dev/ttyACM0)
    led_driver_node = Node(
        package='kairos_real_bringup',
        executable='kairos_led_driver',
        name='kairos_led_driver',
        output='screen',
        parameters=[{
            'serial_port': (
                '/dev/ttyUSB_LEDS' if os.path.exists('/dev/ttyUSB_LEDS') else '/dev/ttyACM0'
            ),
            'serial_baud': 115200,
            'update_rate': 5.0
        }]
    )

    # 5. Static TF compatibility connections and aliases:
    # base_footprint -> imu_link, base_footprint <-> robot_base_footprint,
    # imu_link <-> robot_imu_link
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['--x', '0.127', '--y', '0', '--z', '0.208',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', 'imu_link']
    )

    static_tf_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_footprint_alias',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', 'robot_base_footprint']
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu_alias',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'imu_link',
                   '--child-frame-id', 'robot_imu_link']
    )

    return LaunchDescription([
        motor_driver_node,
        vectornav_node,
        ekf_node,
        led_driver_node,
        static_tf_base_to_imu,
        static_tf_footprint,
        static_tf_imu
    ])
