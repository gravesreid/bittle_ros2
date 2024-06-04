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
            'object_detection_subscriber = bittle_ros2.object_detection_subscriber:main',
            'joystick_driver = bittle_ros2.joystick_driver:main',
            'object_detection_driver = bittle_ros2.object_detection_driver:main',
            'image_save_subscriber = bittle_ros2.image_save_subscriber:main',
            'latency_test = bittle_ros2.latency_test:main',
            'pi_servo_driver = bittle_ros2.pi_servo_driver:main',
            'experiment_driver = bittle_ros2.experiment_driver:main',
            'experiment_subscriber = bittle_ros2.experiment_subscriber:main',
            'demo_driver = bittle_ros2.demo_driver:main',
            'robot_object_detection_subscriber = bittle_ros2.robot_object_detection_subscriber:main',
            'serial_sender = bittle_ros2.serial_sender:main',
            'object_detection_to_command = bittle_ros2.object_detection_to_command:main',
            'photo_service = bittle_ros2.photo_service:main',
            'photo_client = bittle_ros2.photo_client:main',
            'gpt_driver = bittle_ros2.gpt_driver:main'
        ],
    },
)

