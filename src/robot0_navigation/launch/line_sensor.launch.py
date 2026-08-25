import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robot0_nav = get_package_share_directory('robot0_navigation')
    default_config_path = os.path.join(pkg_robot0_nav, 'config', 'line_sensor_params.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_config_path,
        description='Full path to the ROS2 parameters file for line sensor simulator'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    line_sensor_node = Node(
        package='robot0_navigation',
        executable='line_sensor_node',
        name='line_sensor_simulator_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        line_sensor_node
    ])
