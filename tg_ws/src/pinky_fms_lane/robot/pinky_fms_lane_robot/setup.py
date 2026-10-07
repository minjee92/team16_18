from glob import glob

from setuptools import find_packages, setup

package_name = 'pinky_fms_lane_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.xml') + glob('launch/*.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minjee92',
    maintainer_email='minjee92@users.noreply.github.com',
    description='FMS lane driving (robot): localization launch (map_server + AMCL, no Nav2 navigation nodes)',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
