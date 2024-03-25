from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bittle_ros2',
            executable='object_detection_driver',
            name='object_detection_driver',
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
        ),
        Node(
            package='bittle_ros2',
            executable='robot_object_detection_subscriber',
            name='robot_object_detection_subscriber',
            output='screen'
        )
    ])