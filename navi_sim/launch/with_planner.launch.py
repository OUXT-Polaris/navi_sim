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
    hermite_path_planner_bringup_launch_file_dir = os.path.join(get_package_share_directory('hermite_path_planner_bringup'), 'launch')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([navi_sim_launch_file_dir, '/navi_sim.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([hermite_path_planner_bringup_launch_file_dir, '/bringup.launch.py']),
        )
    ])