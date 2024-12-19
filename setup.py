from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bittle_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('bittle_ros2/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reid',
    maintainer_email='rgraves@andrew.cmu.edu',
    description='Bittle driver for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bittle_driver = bittle_ros2.bittle_driver:main',
            'video_subscriber = bittle_ros2.video_subscriber:main',
            'joystick_driver = bittle_ros2.joystick_driver:main',
            'image_save_subscriber = bittle_ros2.image_save_subscriber:main',
            'demo_driver = bittle_ros2.demo_driver:main',
            'serial_sender = bittle_ros2.serial_sender:main',
            'webvid_publisher = bittle_ros2.webvid_publisher:main',
            'webvid_subscriber = bittle_ros2.webvid_subscriber:main',
            'yolo_node = bittle_ros2.yolo_node:main',
            'apriltag_node = bittle_ros2.apriltag_node:main',
        ],
    },
)

