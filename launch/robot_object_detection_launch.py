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
            executable='serial_sender',
            name='serial_sender',
            output='screen'
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            output='screen',
            parameters=[
                {'video_device': '/dev/video0'},
                {'output_encoding': 'rgb8'},
                {'pixel_format': 'YUYV'},
                {'image_size': [640,480]},
                {'io_method': 'mmap'}
            ]
        )
    ])