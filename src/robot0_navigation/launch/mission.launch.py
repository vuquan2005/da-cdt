import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_navigation = get_package_share_directory('robot0_navigation')
    default_config = os.path.join(pkg_navigation, 'config', 'arena_coordinates.yaml')

    target_rack = LaunchConfiguration('rack', default='rack_left_bot')
    target_shelf = LaunchConfiguration('shelf', default='1')
    target_slot = LaunchConfiguration('slot', default='left')
    dropoff_color = LaunchConfiguration('dropoff', default='')

    return LaunchDescription([
        DeclareLaunchArgument('rack', default_value='rack_left_bot', description='Target Rack name (rack_left_bot, rack_left_mid, rack_left_top, rack_bot_mid_left)'),
        DeclareLaunchArgument('shelf', default_value='1', description='Shelf level: 1 (Bottom) or 2 (Top)'),
        DeclareLaunchArgument('slot', default_value='left', description='Pallet slot: left or right'),
        DeclareLaunchArgument('dropoff', default_value='', description='Central Drop-off color: blue, green, white, yellow, red (empty for home base)'),

        Node(
            package='robot0_navigation',
            executable='autonomous_mission',
            name='autonomous_mission',
            output='screen',
            parameters=[{
                'target_rack': target_rack,
                'target_shelf_level': target_shelf,
                'target_slot': target_slot,
                'dropoff_color': dropoff_color,
            }]
        )
    ])
