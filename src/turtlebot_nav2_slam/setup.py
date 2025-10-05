from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot_nav2_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Nav2 SLAM Navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_navigator = turtlebot_nav2_slam.waypoint_navigator:main',
        ],
    },
)
