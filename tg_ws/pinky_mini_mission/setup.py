from setuptools import find_packages, setup

package_name = 'pinky_mini_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taejackooo',
    maintainer_email='tjkoobro@gmail.com',
    description='Pinky mini missions: odometry out-and-back, waypoint following, clicked-point navigation',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'mission1 = pinky_mini_mission.mission1:main',
            'mission2 = pinky_mini_mission.mission2:main',
            'mission3_1 = pinky_mini_mission.mission3_1:main',
        ],
    },
)
