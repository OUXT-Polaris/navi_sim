# Copyright (c) 2020 OUXT Polaris
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    front_lidar_parm_file = LaunchConfiguration(
        'front_lidar_parm_file',
        default=os.path.join(
            get_package_share_directory('navi_sim'),
            'config', 'front_lidar.yaml')
    )
    rviz_config_dir = os.path.join(
            get_package_share_directory('navi_sim'),
            'config',
            'navi_sim.rviz')
    description_dir = os.path.join(
            get_package_share_directory('wamv_description'), 'launch')
    description = LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),
        DeclareLaunchArgument(
            'front_lidar_parm_file',
            default_value=front_lidar_parm_file,
            description='front_lidar parameters'
        ),
        Node(
            package='navi_sim',
            executable='navi_sim_bringup_node',
            name='navi_sim_bringup',
            parameters=[front_lidar_parm_file],
            output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_dir, '/wamv_description.launch.py']),
        ),
    ])
    return description
