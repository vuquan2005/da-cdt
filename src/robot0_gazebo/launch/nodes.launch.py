import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_robot0_teleop = get_package_share_directory('robot0_teleop')
    pkg_robot0_vision = get_package_share_directory('robot0_vision')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_joy = LaunchConfiguration('joy', default='true')
    use_vision = LaunchConfiguration('vision', default='true')
    conf = LaunchConfiguration('conf', default='0.5')
    imgsz = LaunchConfiguration('imgsz', default='640')

    # Declare Launch Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
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

    # 1. Điều khiển Joystick (joy_node + teleop_node)
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot0_teleop, 'launch', 'joystick.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_joy)
    )

    # 2. Thị giác máy tính YOLO (yolo_detector_node)
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

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_use_joy_cmd,
        declare_use_vision_cmd,
        declare_conf_cmd,
        declare_imgsz_cmd,
        joystick_launch,
        vision_launch
    ])
