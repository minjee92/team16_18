from setuptools import find_packages, setup

package_name = 'mj_lane_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minjee92',
    maintainer_email='lmj0421@infobank.net',
    description='차선 후처리·조향·비상 정지 판정 (ROS·하드웨어 없음)',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
