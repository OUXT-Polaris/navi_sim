import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    navi_sim_launch_file_dir = os.path.join(get_package_share_directory('navi_sim'), 'launch')
    planner_concatenator_param_file = LaunchConfiguration(
        'planner_concatenator_param_dir',
        default=os.path.join(
            get_package_share_directory('velocity_planner'),
            'config','planner_concatenator.yaml'))
    curve_planner_param_file = LaunchConfiguration(
        'curve_planner_param_file',
        default=os.path.join(
            get_package_share_directory('velocity_planner'),
            'config','curve_planner.yaml'))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([navi_sim_launch_file_dir, '/navi_sim.launch.py']),
        ),
        DeclareLaunchArgument(
            'planner_concatenator_param_file',
            default_value=planner_concatenator_param_file,
            description='planner concatenator paramters'),
        DeclareLaunchArgument(
            'curve_planner_param_file',
            default_value=curve_planner_param_file,
            description='curve planner parameters'
        ),
        Node(
            package='hermite_path_planner_bringup',
            node_executable='hermite_path_planner_bringup_node',
            parameters=[planner_concatenator_param_file,curve_planner_param_file],
            output='screen'),
    ])