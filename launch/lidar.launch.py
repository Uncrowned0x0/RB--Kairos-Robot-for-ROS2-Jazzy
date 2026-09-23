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
Launch Front and Rear SICK TiM5xx LiDARs, 360° Merger, and Robot State Publisher.

Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Launch Front and Rear SICK TiM5xx LiDARs + 360° Merger + Robot State Publisher."""
    # Generate robot description from RB-KAIROS plus xacro (with UR5e arm)
    pkg_dir = get_package_share_directory('robotnik_description')
    xacro_file = os.path.join(pkg_dir, 'robots', 'rbkairos', 'rbkairos_plus.urdf.xacro')
    robot_doc = xacro.process_file(xacro_file, mappings={'ur_type': 'ur5e'})
    robot_description = {'robot_description': robot_doc.toxml()}

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_pub_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'rate': 20.0,
            'source_list': ['/joint_states_wheels']
        }]
    )

    front_lidar_node = Node(
        package='kairos_real_bringup',
        executable='sick_tim_driver',
        name='front_laser',
        output='screen',
        parameters=[{
            'sensor_ip': '192.168.0.10',
            'sensor_port': 2111,
            'frame_id': 'front_laser_link',
            'scan_topic': '/front_laser/scan',
            'range_min': 0.05,
            'range_max': 25.0
        }]
    )

    rear_lidar_node = Node(
        package='kairos_real_bringup',
        executable='sick_tim_driver',
        name='rear_laser',
        output='screen',
        parameters=[{
            'sensor_ip': '192.168.0.11',
            'sensor_port': 2111,
            'frame_id': 'rear_laser_link',
            'scan_topic': '/rear_laser/scan',
            'range_min': 0.05,
            'range_max': 25.0
        }]
    )

    dual_laser_merger_node = Node(
        package='kairos_real_bringup',
        executable='dual_laser_merger',
        name='dual_laser_merger',
        output='screen',
        parameters=[{
            'front_scan_topic': '/front_laser/scan',
            'rear_scan_topic': '/rear_laser/scan',
            'merged_scan_topic': '/scan',
            'destination_frame': 'base_footprint',
            'scan_rate': 15.0,
            'angle_increment_deg': 0.5,
            'range_min': 0.05,
            'range_max': 25.0
        }]
    )

    return LaunchDescription([
        robot_state_pub_node,
        joint_state_pub_node,
        front_lidar_node,
        rear_lidar_node,
        dual_laser_merger_node
    ])
