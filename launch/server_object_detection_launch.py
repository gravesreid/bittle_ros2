from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bittle_ros2',
            executable='object_detection_to_command',
            name='object_detection_to_command',
            output='screen'
        ),
        Node(
            package='bittle_ros2',
            executable='object_detection_subscriber',
            output='screen'
        )
    ])