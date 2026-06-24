from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mac_brain'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vmuser',
    maintainer_email='vmuser@robotanninh.local',
    description='ROS 2 brain package for security robot — SLAM, Nav2, web bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_republisher = mac_brain.pose_republisher:main',
            'draw_handler     = mac_brain.draw_handler:main',
        ],
    },
)
