import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration

def generate_launch_description():
    rviz_config_dir = os.path.join(
            get_package_share_directory('navi_sim'),
            'config',
            'navi_sim.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    description = LaunchDescription([
        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),
        Node(
            package='navi_sim',
            node_executable='navi_sim_node',
            node_name='navi_sim_node',
            output='screen'),
    ])
    return description