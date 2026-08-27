import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_rear = LaunchConfiguration('enable_rear', default='true')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_enable_rear_cmd = DeclareLaunchArgument(
        'enable_rear',
        default_value='true',
        description='Enable rear sensor array'
    )

    line_sensor_node = Node(
        package='robot0_sensors',
        executable='line_sensor_node',
        name='line_sensor_simulator_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_rear_array': enable_rear,
            'num_sensors_front': 8,
            'sensor_spacing_front': 0.018,
            'offset_x_front': 0.18,
            'offset_y_front': 0.0,
            'num_sensors_rear': 8,
            'sensor_spacing_rear': 0.018,
            'offset_x_rear': -0.18,
            'offset_y_rear': 0.0,
            'line_width': 0.025,
            'update_rate': 50.0,
            'base_frame': 'base_link',
            'world_frame': 'odom',
        }]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_enable_rear_cmd,
        line_sensor_node
    ])
