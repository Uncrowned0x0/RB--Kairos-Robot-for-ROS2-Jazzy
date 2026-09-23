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

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'kairos_real_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Kamil BENMADI',
    author_email='kamil.benmadi@sigma-clermont.fr',
    maintainer='Kamil BENMADI',
    maintainer_email='kamil.benmadi@sigma-clermont.fr',
    description='RB-KAIROS Real Hardware Bringup Drivers',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'kairos_serial_motor_driver = kairos_real_bringup.kairos_serial_motor_driver:main',
            'vectornav_driver = kairos_real_bringup.vectornav_driver:main',
            'dual_laser_merger = kairos_real_bringup.dual_laser_merger:main',
            'kairos_led_driver = kairos_real_bringup.kairos_led_driver:main',
            'kairos_battery_gui = kairos_real_bringup.battery_gui:main',
            'sick_tim_driver = kairos_real_bringup.sick_tim_driver:main',
            'lidar_radar_cli = kairos_real_bringup.lidar_radar_cli:main',
            'kairos_teleop_keyboard = kairos_real_bringup.kairos_teleop_keyboard:main',
            'kairos_teleop_joy = kairos_real_bringup.kairos_teleop_joy:main',
        ],
    },
)
