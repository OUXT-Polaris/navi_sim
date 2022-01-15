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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions.launch_configuration import LaunchConfiguration


import os
import yaml


def getLidarSimComponent(lidar_name):
    config_directory = os.path.join(
        get_package_share_directory('navi_sim'),
        'config')
    param_config = os.path.join(config_directory, lidar_name+'.yaml')
    params = {}
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)[lidar_name + '_node']['ros__parameters']
    object_config_path = os.path.join(
            get_package_share_directory('navi_sim'),
            'config',
            'objects.json')
    params['objects_path'] = object_config_path
    component = ComposableNode(
        package='navi_sim',
        plugin='navi_sim::LidarSimComponent',
        namespace='/sensing/'+lidar_name,
        name=lidar_name + '_node',
        remappings=[('lidar_points', 'points_raw')],
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
    object_config_path = os.path.join(
            get_package_share_directory('navi_sim'),
            'config',
            'objects.json')
    params['objects_path'] = object_config_path
    component = ComposableNode(
        package='navi_sim',
        plugin='navi_sim::CameraSimComponent',
        namespace='/sensing/'+camera_name,
        name=camera_name + '_node',
        remappings=[],
        parameters=[params])
    return component


def getScenarioTestComponent(scenario_filename):
    config_directory = os.path.join(
        get_package_share_directory('navi_sim'),
        'config')
    param_config = os.path.join(config_directory, 'scenario_test.yaml')
    params = {}
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['scenario_test_node']['ros__parameters']
    object_config_path = os.path.join(
            get_package_share_directory('navi_sim'),
            'config',
            'objects.json')
    params['objects_path'] = object_config_path
    params['scenario_filename'] = scenario_filename
    component = ComposableNode(
        package='navi_sim',
        plugin='navi_sim::ScenarioTestComponent',
        name='scenario_test_node',
        namespace='simulation',
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
    scenario_filename = LaunchConfiguration('scenario_filename', default='go_straight.yaml')
    record = LaunchConfiguration('record', default=False)
    rosbag_directory = LaunchConfiguration('rosbag_directory', default='/tmp')
    simulator = ComposableNodeContainer(
        name='navi_sim_bringup_container',
        namespace='sensing',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            getNaviSimComponent(),
            getScenarioTestComponent(scenario_filename),
            getLidarSimComponent('front_lidar'),
            getLidarSimComponent('rear_lidar'),
            getLidarSimComponent('right_lidar'),
            getLidarSimComponent('left_lidar'),
            getCameraSimComponent('front_left_camera'),
            getCameraSimComponent('front_right_camera'),
            getCameraSimComponent('rear_left_camera'),
            getCameraSimComponent('rear_right_camera'),
            getCameraSimComponent('left_camera'),
            getCameraSimComponent('right_camera')
        ],
        output='screen')
    description = LaunchDescription([
        DeclareLaunchArgument(
            'scenario_filename',
            default_value=scenario_filename,
            description='filename of the scenario yaml file.'),
        DeclareLaunchArgument(
            'record',
            default_value=record,
            description='If true, record rosbag data.'
        ),
        DeclareLaunchArgument(
            'rosbag_directory',
            default_value=rosbag_directory,
            description='output directory of the rosbag data'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),
        simulator,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=simulator,
                on_exit=[EmitEvent(event=Shutdown())])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_dir, '/wamv_description.launch.py']),
        ),
        ExecuteProcess(
            cmd=[
                'ros2',
                'bag',
                'record',
                '-a',
                '-o', rosbag_directory,
                '--compression-mode', 'file',
                '--compression-format', 'zstd'],
            output='screen',
            condition=IfCondition(record)
        )
    ])
    return description
