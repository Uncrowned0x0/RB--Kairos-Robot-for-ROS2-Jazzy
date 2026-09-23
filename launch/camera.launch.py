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
Launch Intel RealSense D435 camera RGB and Depth streams.

Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    realsense_camera_dir = get_package_share_directory('realsense2_camera')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_camera_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true'
        }.items()
    )

    return LaunchDescription([
        realsense_launch
    ])
