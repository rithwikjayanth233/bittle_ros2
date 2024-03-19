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
            'experiment_huddle_1 = bittle_ros2.experiment_huddle_1:main',
            'experiment_driver = bittle_ros2.experiment_driver:main',
            'experiment_subscriber = bittle_ros2.experiment_subscriber:main',
            'robot_experiment_subscriber = bittle_ros2.robot_experiment_subscriber:main'
        ],
    },
)

