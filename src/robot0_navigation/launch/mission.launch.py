import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_navigation = get_package_share_directory('robot0_navigation')

    target_pallet = LaunchConfiguration('pallet', default='cpu')
    target_shelf = LaunchConfiguration('shelf', default='1')
    target_slot = LaunchConfiguration('slot', default='left')
    dropoff_color = LaunchConfiguration('dropoff', default='')

    return LaunchDescription([
        DeclareLaunchArgument('pallet', default_value='cpu', description='Target Pallet type: cpu, aluminum, chip, qr'),
        DeclareLaunchArgument('shelf', default_value='1', description='Shelf level: 1 (Bottom) or 2 (Top)'),
        DeclareLaunchArgument('slot', default_value='left', description='Pallet slot: left or right'),
        DeclareLaunchArgument('dropoff', default_value='', description='Drop-off station: blue, red, green (empty for auto-mapping)'),

        Node(
            package='robot0_navigation',
            executable='autonomous_mission',
            name='autonomous_mission',
            output='screen',
            parameters=[{
                'pallet_type': target_pallet,
                'shelf_level': target_shelf,
                'slot_side': target_slot,
                'dropoff': dropoff_color,
            }]
        )
    ])
