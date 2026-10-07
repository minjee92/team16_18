from glob import glob

from setuptools import find_packages, setup

package_name = 'pinky_fms_lane'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/course', glob('course/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minjee92',
    maintainer_email='minjee92@users.noreply.github.com',
    description='FMS lane driving (control PC): course model, route planning, course check tool',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_course = pinky_fms_lane.draw_course:main',
        ],
    },
)
