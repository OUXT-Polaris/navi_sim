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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import yaml


def getLidarSimComponent(lidar_name):
    config_directory = os.path.join(
        get_package_share_directory('navi_sim'),
        'config')
    param_config = os.path.join(config_directory, lidar_name+'.yaml')
    params = {}
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)[lidar_name + '_node']['ros__parameters']
        print(params)
    object_config_path = os.path.join(
            get_package_share_directory('navi_sim'),
            'config',
            'objects.json')
    params["objects_path"] = object_config_path
    component = ComposableNode(
        package='navi_sim',
        plugin='navi_sim::LidarSimComponent',
        namespace='/sensing/'+lidar_name,
        name=lidar_name + '_node',
        remappings=[("lidar_points", "points_raw")],
        parameters=[params])
    return component


def getCameraSimComponent(camera_name):
    config_directory = os.path.join(
        get_package_share_directory('navi_sim'),
        'config')
    param_config = os.path.join(config_directory, camera_name+'.yaml')
    params = {}
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)[camera_name + '_node']['ros__parameters']
        print(params)
    object_config_path = os.path.join(
            get_package_share_directory('navi_sim'),
            'config',
            'objects.json')
    params["objects_path"] = object_config_path
    component = ComposableNode(
        package='navi_sim',
        plugin='navi_sim::CameraSimComponent',
        namespace='/sensing/'+camera_name,
        name=camera_name + '_node',
        remappings=[],
        parameters=[params])
    return component


def getNaviSimComponent():
    component = ComposableNode(
        package='navi_sim',
        plugin='navi_sim::NaviSimComponent',
        name='navi_sim_node')
    return component


def generate_launch_description():
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
        ComposableNodeContainer(
            name='navi_sim_bringup_container',
            namespace='sensing',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                getNaviSimComponent(),
                # getLidarSimComponent("front_lidar"),
                # getLidarSimComponent("rear_lidar"),
                # getLidarSimComponent("right_lidar"),
                # getLidarSimComponent("left_lidar"),
                getCameraSimComponent("front_left_camera")
            ],
            output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_dir, '/wamv_description.launch.py']),
        ),
    ])
    return description
