from setuptools import setup
import os
from glob import glob

package_name = 'waffle_pi_lane_tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='filipjakub',
    maintainer_email='filipjakub@todo.todo',
    description='Lane detection and tracking for TurtleBot3 Waffle Pi - ROS2 Humble port.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_lane = waffle_pi_lane_tracking.detect_lane:main',
            'control_robot = waffle_pi_lane_tracking.control_robot:main',
        ],
    },
)
