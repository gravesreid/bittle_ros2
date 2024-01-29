from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bittle_driver',
            executable='bittle_driver',
            name='main_module',
            output='screen'
        )#,
       # Node(
       #     package='usb_cam',
       #     executable='usb_cam_node',
       #     name='camera',
       #     output='screen',
       #     parameters=[
       #         {'video_device': '/dev/video0'},
       #         {'image_width': 640},
       #         {'image_height': 480},
       #         {'pixel_format': 'yuyv'},
       #         {'camera_frame_id': 'camera'},
       #         {'io_method': 'mmap'}
       #     ]
       # )
    ])
