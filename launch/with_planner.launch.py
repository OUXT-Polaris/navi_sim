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
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration


def generate_launch_description():
    navi_sim_launch_file_dir = os.path.join(get_package_share_directory('navi_sim'), 'launch')
    hermite_path_planner_package_path = get_package_share_directory('hermite_path_planner_bringup')
    hermite_path_planner_launch_dir = os.path.join(hermite_path_planner_package_path, 'launch')
    perception_bringup_package_path = get_package_share_directory('perception_bringup')
    perception_bringup_launch_dir = os.path.join(perception_bringup_package_path, 'launch')
    scenario_filename = LaunchConfiguration("scenario_filename", default="go_straight.yaml")
    record = LaunchConfiguration("record", default=False)
    rosbag_directory = LaunchConfiguration("rosbag_directory", default="/tmp")
    planner_launch_prefix = LaunchConfiguration(
        "planner_launch_prefix",
        default="taskset -c 0")
    perception_launch_prefix = LaunchConfiguration(
        "perception_launch_prefix",
        default="taskset -c 1")
    simulation_launch_prefix = LaunchConfiguration(
        "simulation_launch_prefix",
        default="taskset -c 0")
    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario_filename",
            default_value=scenario_filename,
            description="filename of the scenario yaml file."),
        DeclareLaunchArgument(
            "record",
            default_value=record,
            description="If true, record rosbag data."),
        DeclareLaunchArgument(
            "planner_launch_prefix",
            default_value="taskset -c 0",
            description="launch prefix of planner executor"
        ),
        DeclareLaunchArgument(
            "perception_launch_prefix",
            default_value="taskset -c 1",
            description="launch prefix of perception executor"
        ),
        DeclareLaunchArgument(
            "simulation_launch_prefix",
            default_value="taskset -c 0",
            description="launch prefix of simulation executor"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [navi_sim_launch_file_dir, '/navi_sim.launch.py']),
            launch_arguments={
                'scenario_filename': scenario_filename,
                'record': record,
                'rosbag_directory': rosbag_directory,
                'launch_prefix': simulation_launch_prefix}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [hermite_path_planner_launch_dir, '/bringup.launch.py']),
            launch_arguments={'launch_prefix': planner_launch_prefix}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [perception_bringup_launch_dir, '/perception_bringup.launch.py']),
            launch_arguments={'launch_prefix': perception_launch_prefix}.items()
        )
    ])
