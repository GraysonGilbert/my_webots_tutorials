import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    
    # Launch argument to enable/disable bag recording
    enable_bag_recording = LaunchConfiguration("enable_bag_recording")

    declare_enable_bag_recording = DeclareLaunchArgument(
        "enable_bag_recording",
        default_value="true",
        description="Enable rosbag recording"
    )

    # Bag recording command (records ALL topics)
    rosbag_record = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-a"],
        output="screen",
        condition=IfCondition(enable_bag_recording)
    )
    
    # Locate robot files
    package_dir = get_package_share_directory('walker')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    # Webots world
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    # Robot drive node
    my_robot_driver = WebotsController(
        robot_name='walker_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    # Obstacle avoider node
    obstacle_avoider = Node(
        package='walker',
        executable='webot_walker',
        name='obstacle_avoider',
        output='screen'
    )

    return LaunchDescription([
        declare_enable_bag_recording,
        rosbag_record,
        webots,
        my_robot_driver,
        obstacle_avoider,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])