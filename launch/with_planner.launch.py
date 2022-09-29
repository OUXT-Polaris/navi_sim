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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.launch_configuration import LaunchConfiguration


import os


def generate_launch_description():
    navi_sim_launch_file_dir = os.path.join(
        get_package_share_directory("navi_sim"), "launch"
    )
    planner_package_path = get_package_share_directory("robotx_planner_bringup")
    planner_launch_dir = os.path.join(planner_package_path, "launch")
    perception_bringup_package_path = get_package_share_directory("perception_bringup")
    perception_bringup_launch_dir = os.path.join(
        perception_bringup_package_path, "launch"
    )
    scenario_filename = LaunchConfiguration(
        "scenario_filename", default="go_straight.yaml"
    )
    scenario_mode = LaunchConfiguration("scenario_mode", default=False)
    record = LaunchConfiguration("record", default=False)
    use_hardware = LaunchConfiguration("use_hardware",default=True)
    rosbag_directory = LaunchConfiguration("rosbag_directory", default="/tmp")
    behavior_config_package = LaunchConfiguration(
        "behavior_config_package", default="robotx_bt_planner"
    )
    behavior_config_filepath = LaunchConfiguration(
        "behavior_config_filepath", default="config/go.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario_filename",
                default_value=scenario_filename,
                description="filename of the scenario yaml file.",
            ),
            DeclareLaunchArgument(
                "scenario_mode",
                default_value=scenario_mode,
                description="If true, running with scenario_mode",
            ),
            DeclareLaunchArgument(
                "record",
                default_value=record,
                description="If true, record rosbag data.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [navi_sim_launch_file_dir, "/navi_sim.launch.py"]
                ),
                launch_arguments={
                    "scenario_filename": scenario_filename,
                    "record": record,
                    "rosbag_directory": rosbag_directory,
                }.items(),
            ),
            DeclareLaunchArgument(
                "behavior_config_package",
                default_value=behavior_config_package,
                description="package of behavior config file",
            ),
            DeclareLaunchArgument(
                "behavior_config_filepath",
                default_value=behavior_config_filepath,
                description="filepath of the behavior config",
            ),
            DeclareLaunchArgument(
                "use_hardware",
                default_value=use_hardware,
                description="If true, use real hardware."
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [planner_launch_dir, "/planner_bringup.launch.py"]
                ),
                launch_arguments={
                    "behavior_config_package": behavior_config_package,
                    "behavior_config_filepath": behavior_config_filepath,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [perception_bringup_launch_dir, "/perception_bringup.launch.py"]
                ),
                launch_arguments={
                    "use_hardware": use_hardware,
                }.items(),
            ),
        ]
    )
