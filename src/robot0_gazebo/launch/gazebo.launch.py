import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_robot0_gazebo = get_package_share_directory('robot0_gazebo')
    pkg_robot0_description = get_package_share_directory('robot0_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    pkg_gazebo_prefix = get_package_prefix('robot0_gazebo')
    pkg_description_prefix = get_package_prefix('robot0_description')

    # Default Paths
    default_world_path = os.path.join(pkg_robot0_gazebo, 'worlds', 'robocon_arena.sdf')
    default_urdf_path = os.path.join(pkg_robot0_description, 'urdf', 'robot0.urdf')
    default_bridge_config_path = os.path.join(pkg_robot0_gazebo, 'config', 'ros_gz_bridge.yaml')
    default_rviz_config_path = os.path.join(pkg_robot0_description, 'rviz', 'robot0.rviz')

    # Environment variables for Gazebo resource finding (resolve model:// and package://)
    pkg_share_parent = os.path.dirname(pkg_robot0_gazebo)
    gazebo_models_dir = os.path.join(pkg_robot0_gazebo, 'models')

    resource_dirs = [
        pkg_share_parent,
        os.path.dirname(pkg_robot0_description),
        pkg_robot0_gazebo,
        pkg_robot0_description,
        gazebo_models_dir,
        os.path.join(pkg_robot0_description, 'models'),
    ]
    # Also include share paths from AMENT_PREFIX_PATH if available
    ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')
    for p in ament_prefix_path.split(':'):
        if p:
            resource_dirs.append(os.path.join(p, 'share'))

    resource_dirs = [d for d in dict.fromkeys(resource_dirs) if os.path.exists(d)]
    env_resource_paths = ':'.join(resource_dirs)

    plugin_dirs = [
        os.path.join(pkg_gazebo_prefix, 'lib'),
        os.path.join(pkg_description_prefix, 'lib'),
    ]
    for p in ament_prefix_path.split(':'):
        if p:
            plugin_dirs.append(os.path.join(p, 'lib'))

    plugin_dirs = [d for d in dict.fromkeys(plugin_dirs) if os.path.exists(d)]
    env_plugin_paths = ':'.join(plugin_dirs)

    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=env_resource_paths
    )
    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=env_resource_paths
    )
    set_sdf_path = AppendEnvironmentVariable(
        name='SDF_PATH',
        value=env_resource_paths
    )
    set_gz_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=env_resource_paths
    )
    set_ign_plugin_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value=env_plugin_paths
    )
    set_gz_plugin_path = AppendEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=env_plugin_paths
    )

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=default_world_path)
    use_rviz = LaunchConfiguration('rviz', default='true')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Full path to world model file to load'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 if true'
    )

    # Robot State Publisher (publishes TF & robot_description from robot0_description package)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', default_urdf_path]),
                value_type=str
            )
        }]
    )

    # Gazebo Simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )

    # Spawn robot entity in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'default',
            '-topic', 'robot_description',
            '-name', 'robot0',
            '-allow_renaming', 'true',
            '-x', '-0.985',
            '-y', '0.64',
            '-z', '0.08',
            '-Y', '3.14159265'
        ]
    )

    # ROS-Gazebo Parameter Bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': default_bridge_config_path,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,
        set_sdf_path,
        set_gz_model_path,
        set_ign_plugin_path,
        set_gz_plugin_path,
        declare_use_sim_time_cmd,
        declare_world_cmd,
        declare_use_rviz_cmd,
        gz_sim,
        robot_state_publisher_node,
        TimerAction(
            period=3.0,
            actions=[spawn_robot]
        ),
        bridge_node,
        rviz_node
    ])
