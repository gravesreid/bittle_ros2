from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bittle_ros2',
            executable='object_detection_subscriber',
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            output='screen'
        )
        ])