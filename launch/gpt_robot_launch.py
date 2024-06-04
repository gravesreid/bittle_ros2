from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bittle_ros2',
            executable='gpt_driver',
        ),
        Node(
            package='bittle_ros2',
            executable='photo_service',
        )
    ])