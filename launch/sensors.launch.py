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
Launch all perceptual sensors on RB-KAIROS: RealSense D435 and Front & Rear SICK LiDARs.

Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Launch all perceptual sensors on RB-KAIROS: RealSense D435 + Front & Rear SICK LiDARs."""
    pkg_dir = get_package_share_directory('kairos_real_bringup')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'camera.launch.py')
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'lidar.launch.py')
        )
    )

    return LaunchDescription([
        camera_launch,
        lidar_launch
    ])
