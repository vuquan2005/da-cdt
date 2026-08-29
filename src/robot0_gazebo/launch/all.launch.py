import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_robot0_gazebo = get_package_share_directory('robot0_gazebo')

    # Default Paths
    default_world_path = os.path.join(pkg_robot0_gazebo, 'worlds', 'simple_arena.sdf')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=default_world_path)
    use_rviz = LaunchConfiguration('rviz', default='true')
    use_controller = LaunchConfiguration('controller', default='true')
    use_joy = LaunchConfiguration('joy', default='true')
    use_vision = LaunchConfiguration('vision', default='true')
    use_line_sensor = LaunchConfiguration('line_sensor', default='true')
    conf = LaunchConfiguration('conf', default='0.5')
    imgsz = LaunchConfiguration('imgsz', default='640')

    # Declare Launch Arguments
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

    declare_use_controller_cmd = DeclareLaunchArgument(
        'controller',
        default_value='true',
        description='Launch Kinematics Base Controller if true'
    )

    declare_use_joy_cmd = DeclareLaunchArgument(
        'joy',
        default_value='true',
        description='Launch Joystick teleop if true'
    )

    declare_use_vision_cmd = DeclareLaunchArgument(
        'vision',
        default_value='true',
        description='Launch YOLO vision detector if true'
    )

    declare_use_line_sensor_cmd = DeclareLaunchArgument(
        'line_sensor',
        default_value='true',
        description='Launch Dual Array Line Sensor simulator if true'
    )

    declare_conf_cmd = DeclareLaunchArgument(
        'conf',
        default_value='0.5',
        description='YOLO confidence threshold'
    )

    declare_imgsz_cmd = DeclareLaunchArgument(
        'imgsz',
        default_value='640',
        description='YOLO inference image size (640, 480, 320)'
    )

    # 1. Mô phỏng Gazebo (Gazebo Fortress + Bridge + Robot State Publisher + RViz2)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot0_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'rviz': use_rviz
        }.items()
    )

    # 2. Cụm các Node ứng dụng
    nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot0_gazebo, 'launch', 'nodes.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'controller': use_controller,
            'joy': use_joy,
            'vision': use_vision,
            'line_sensor': use_line_sensor,
            'conf': conf,
            'imgsz': imgsz
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_cmd,
        declare_use_rviz_cmd,
        declare_use_controller_cmd,
        declare_use_joy_cmd,
        declare_use_vision_cmd,
        declare_use_line_sensor_cmd,
        declare_conf_cmd,
        declare_imgsz_cmd,
        gazebo_launch,
        nodes_launch
    ])


