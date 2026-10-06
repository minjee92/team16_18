from setuptools import find_packages, setup

package_name = 'pinky_fms_traffic'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/traffic_core.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tkoo',
    maintainer_email='tjkoobro@gmail.com',
    description='FMS traffic (collision/deadlock avoidance)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fleet_traffic = pinky_fms_traffic.fleet_traffic:main',
            'traffic_mock_robot = pinky_fms_traffic.traffic_mock_robot:main',
        ],
    },
)
