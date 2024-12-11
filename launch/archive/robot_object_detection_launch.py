from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
                {'io_method': 'mmap'},
                {'image_raw_jpeg_quality': 40},
            ]
        )
    ])