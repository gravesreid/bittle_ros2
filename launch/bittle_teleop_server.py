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
            package='topic_tools',
            executable='relay',
            name='camera_raw_to_plain',
            output='screen',
            arguments=['/camera/image_raw/compressed', '/camera/image/compressed']
        ),
        Node(
            package='image_view',
            executable='image_view',
            name='image_view',
            output='screen',
            respawn=False,
            remappings=[('image', '/camera/image')],
            parameters=[
                {'autosize': True},
                {'image_transport': 'compressed'}
            ]
        )
    ])
