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
Launch PS4 DualShock gamepad teleoperation node and joy node.

Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('kairos_real_bringup')
    config_file = os.path.join(pkg_share, 'config', 'ps4_teleop.yaml')

    use_twist_joy_arg = DeclareLaunchArgument(
        'use_twist_joy',
        default_value='false',
        description='Whether to use generic teleop_twist_joy instead of kairos_teleop_joy'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[config_file],
        output='screen'
    )

    # Primary RB-KAIROS DualShock Teleoperation Node
    kairos_teleop_joy_node = Node(
        package='kairos_real_bringup',
        executable='kairos_teleop_joy',
        name='kairos_teleop_joy',
        parameters=[config_file],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_twist_joy'))
    )

    # Fallback generic teleop_twist_joy node
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_file],
        remappings=[('/cmd_vel', '/cmd_vel')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_twist_joy'))
    )

    return LaunchDescription([
        use_twist_joy_arg,
        joy_node,
        kairos_teleop_joy_node,
        teleop_twist_joy_node
    ])
