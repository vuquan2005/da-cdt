import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot0_vision'

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
    maintainer='TODO',
    maintainer_email='TODO@email.com',
    description='YOLO object detection and vision processing node for robot0',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = robot0_vision.yolo_detector_node:main',
        ],
    },
)
