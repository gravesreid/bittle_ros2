from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rqt_robot_steering',
            executable='rqt_robot_steering',
            name='rqt_robot_steering',
            output='screen'
        ),
        Node(
            package='bittle_ros2',
            executable='object_detection_subscriber',
        )
    ])