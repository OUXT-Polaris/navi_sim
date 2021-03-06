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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    navi_sim_launch_file_dir = os.path.join(get_package_share_directory('navi_sim'), 'launch')
    hermite_path_planner_package_path = get_package_share_directory('hermite_path_planner_bringup')
    hermite_path_planner_launch_dir = os.path.join(hermite_path_planner_package_path, 'launch')
    perception_bringup_package_path = get_package_share_directory('perception_bringup')
    perception_bringup_launch_dir = os.path.join(perception_bringup_package_path, 'launch')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([navi_sim_launch_file_dir, '/navi_sim.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([hermite_path_planner_launch_dir, '/bringup.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    perception_bringup_launch_dir, '/perception_bringup.launch.py'
                ]),
        )
    ])
