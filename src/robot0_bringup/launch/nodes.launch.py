import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_robot0_controller = get_package_share_directory('robot0_controller')
    pkg_robot0_sensors = get_package_share_directory('robot0_sensors')
    pkg_robot0_teleop = get_package_share_directory('robot0_teleop')
    pkg_robot0_vision = get_package_share_directory('robot0_vision')
    pkg_robot0_navigation = get_package_share_directory('robot0_navigation')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_controller = LaunchConfiguration('controller', default='true')
    use_joy = LaunchConfiguration('joy', default='true')
    use_vision = LaunchConfiguration('vision', default='true')
    use_line_sensor = LaunchConfiguration('line_sensor', default='true')
    use_mission = LaunchConfiguration('mission', default='false')
    conf = LaunchConfiguration('conf', default='0.5')
    imgsz = LaunchConfiguration('imgsz', default='640')

    # Declare Launch Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
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

    declare_use_mission_cmd = DeclareLaunchArgument(
        'mission',
        default_value='false',
        description='Launch Behavior Tree autonomous pallet mission if true'
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

    # 1. Base Controller / Kinematics (kinematics_node)
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot0_controller, 'launch', 'controller.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_controller)
    )

    # 2. Vector Line Sensor Simulator
    line_sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot0_sensors, 'launch', 'line_sensor.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_line_sensor)
    )

    # 3. Joystick Teleoperation
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot0_teleop, 'launch', 'joystick.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_joy)
    )

    # 4. YOLO Object Detector
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot0_vision, 'launch', 'yolo_detector.launch.py')
        ),
        launch_arguments={
            'conf': conf,
            'imgsz': imgsz
        }.items(),
        condition=IfCondition(use_vision)
    )

    # 5. Behavior Tree Autonomous Navigation Mission
    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot0_navigation, 'launch', 'pallet_mission_bt.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_mission)
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_use_controller_cmd,
        declare_use_joy_cmd,
        declare_use_vision_cmd,
        declare_use_line_sensor_cmd,
        declare_use_mission_cmd,
        declare_conf_cmd,
        declare_imgsz_cmd,
        controller_launch,
        line_sensor_launch,
        joystick_launch,
        vision_launch,
        mission_launch
    ])
