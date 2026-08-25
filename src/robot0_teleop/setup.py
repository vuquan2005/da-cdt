import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot0_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vuquan',
    maintainer_email='66623851+vuquan2005@users.noreply.github.com',
    description='Teleoperation and manual joystick control package for robot0',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'teleop_node = robot0_teleop.teleop_node:main',
        ],
    },
)
