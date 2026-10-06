from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pinky_autonomous'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tkoo',
    maintainer_email='tjkoobro@gmail.com',
    description='Lane-following autonomous driving for Pinky Pro (YOLO NCNN, ultrasonic, LED, SLAM return-home)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_drive = pinky_autonomous.autonomous_drive_node:main',
            'video_recorder   = pinky_autonomous.video_recorder_node:main',
            'ultrasonic_sensor = pinky_autonomous.ultrasonic_node:main',
        ],
    },
)
