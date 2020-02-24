import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    navi_sim_launch_file_dir = os.path.join(get_package_share_directory('navi_sim'), 'launch')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([navi_sim_launch_file_dir, '/navi_sim.launch.py']),
        ),
        Node(
            package='hermite_path_planner_bringup',
            node_executable='hermite_path_planner_bringup_node',
            output='screen'),
    ])