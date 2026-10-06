from setuptools import find_packages, setup

package_name = 'pinky_fms_core'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/robots.yaml']),
        ('share/' + package_name + '/launch', ['launch/fms_core.launch.xml', 'launch/mock_robot.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tkoo',
    maintainer_email='tjkoobro@gmail.com',
    description='FMS mission / coordinator nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fleet_mission = pinky_fms_core.fleet_mission:main',
            'fleet_coordinator = pinky_fms_core.fleet_coordinator:main',
            'mock_robot = pinky_fms_core.mock_robot:main',
        ],
    },
)
