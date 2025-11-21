from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the existing Webots TurtleBot launch
    pkg_webots_turtlebot = os.path.join(
        get_package_share_directory('webots_ros2_turtlebot'),
        'launch',
        'robot_launch.py'
    )
    
    
    my_world_path = os.path.join(
        get_package_share_directory('walker'),
        'worlds',
        'my_world.wbt'
    )

    return LaunchDescription([
        # Launch Webots + TurtleBot3
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pkg_webots_turtlebot),
            launch_arguments={'world': my_world_path, 'turtlebot3_model': 'burger'}.items()
        ),

    ])
