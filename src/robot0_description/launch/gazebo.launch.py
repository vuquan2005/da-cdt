import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_robot0_gazebo = get_package_share_directory('robot0_gazebo')
    gazebo_launch_path = os.path.join(pkg_robot0_gazebo, 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path)
        )
    ])
