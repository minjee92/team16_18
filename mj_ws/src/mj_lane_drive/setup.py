from setuptools import find_packages, setup

package_name = 'mj_lane_drive'

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
    description='ROS 공용 차선 주행 노드 (시뮬·실물 공통, 입력원을 인자로 고름)',
    license='Apache-2.0',
    entry_points={'console_scripts': ['lane_follow = mj_lane_drive.lane_follow_node:main']},
)
