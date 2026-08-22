import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_robot0_description = get_package_share_directory('robot0_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    default_world_path = os.path.join(pkg_robot0_description, 'worlds', 'empty.sdf')
    default_urdf_path = os.path.join(pkg_robot0_description, 'urdf', 'robot0.urdf')
    default_bridge_config_path = os.path.join(pkg_robot0_description, 'config', 'ros_gz_bridge.yaml')
    default_rviz_config_path = os.path.join(pkg_robot0_description, 'rviz', 'robot0.rviz')

    # Environment variables for Gazebo resource finding (resolve package://robot0_description/...)
    pkg_share_parent = os.path.dirname(pkg_robot0_description)
    src_dir = '/workspaces/ros-cdt/src'
    
    env_resource_paths = f"{pkg_share_parent}:{src_dir}:{pkg_robot0_description}"
    
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

    # Robot State Publisher (publishes TF & robot_description for RViz)
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

    # Spawn robot entity from robot_description topic (includes xacro path expansions)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'default',
            '-topic', 'robot_description',
            '-name', 'robot0',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.08'
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
        declare_use_sim_time_cmd,
        declare_world_cmd,
        declare_use_rviz_cmd,
        gz_sim,
        robot_state_publisher_node,
        spawn_robot,
        bridge_node,
        rviz_node
    ])
