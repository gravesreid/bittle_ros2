from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
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
                {'image_size': [1280,960]},
                {'io_method': 'mmap'}
            ]
        ),
        Node(
            package='bittle_ros2',
            executable='yolo_node',
            name='yolo_node',
            output='screen'
        ),
        Node(
            package='bittle_ros2',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen'
        )
    ])