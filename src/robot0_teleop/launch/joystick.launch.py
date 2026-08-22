import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot0_teleop = get_package_share_directory('robot0_teleop')
    default_config_path = os.path.join(pkg_robot0_teleop, 'config', 'joy_teleop.yaml')

    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to joystick YAML config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Joy Node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time
            }
        ],
        output='screen'
    )

    # Robot0 Teleop Node (Controls base /cmd_vel with auto-brake & lift /lift_joint_cmd)
    teleop_node = Node(
        package='robot0_teleop',
        executable='teleop_node',
        name='robot0_teleop',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        joy_node,
        teleop_node
    ])
