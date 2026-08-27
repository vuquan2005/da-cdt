import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robot0_nav = get_package_share_directory('robot0_navigation')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    target_rack = LaunchConfiguration('rack', default='rack_1')
    shelf_level = LaunchConfiguration('shelf', default='1')
    target_slot = LaunchConfiguration('slot', default='left')
    pallet_type = LaunchConfiguration('pallet', default='')
    dropoff_zone = LaunchConfiguration('dropoff', default='')
    tick_rate = LaunchConfiguration('rate', default='20.0')

    # Declare Launch Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_target_rack_cmd = DeclareLaunchArgument(
        'rack',
        default_value='rack_1',
        description='Target Rack (rack_1 or rack_2)'
    )

    declare_shelf_level_cmd = DeclareLaunchArgument(
        'shelf',
        default_value='1',
        description='Target Shelf Level (1 for Bottom, 2 for Top)'
    )

    declare_target_slot_cmd = DeclareLaunchArgument(
        'slot',
        default_value='left',
        description='Target Slot (left or right)'
    )

    declare_pallet_type_cmd = DeclareLaunchArgument(
        'pallet',
        default_value='',
        description='Optional pallet type shortcut (aluminum, cpu, qr, chip)'
    )

    declare_dropoff_zone_cmd = DeclareLaunchArgument(
        'dropoff',
        default_value='',
        description='Drop-off Zone (dropoff_1, dropoff_2, dropoff_3, dropoff_4, or home)'
    )

    declare_tick_rate_cmd = DeclareLaunchArgument(
        'rate',
        default_value='20.0',
        description='Behavior Tree tick rate in Hz'
    )

    mission_bt_node = Node(
        package='robot0_navigation',
        executable='pallet_bt_mission',
        name='pallet_bt_mission_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_rack': target_rack,
            'shelf_level': shelf_level,
            'target_slot': target_slot,
            'pallet_type': pallet_type,
            'dropoff_zone': dropoff_zone,
            'tick_rate_hz': tick_rate,
            'print_tree_interval_sec': 3.0,
        }]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_target_rack_cmd,
        declare_shelf_level_cmd,
        declare_target_slot_cmd,
        declare_pallet_type_cmd,
        declare_dropoff_zone_cmd,
        declare_tick_rate_cmd,
        mission_bt_node,
    ])
