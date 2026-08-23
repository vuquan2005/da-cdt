import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot0_vision = get_package_share_directory('robot0_vision')
    default_config_path = os.path.join(pkg_robot0_vision, 'config', 'yolo_params.yaml')
    default_model_path = os.path.join(pkg_robot0_vision, 'models', 'best.pt')

    # Launch Arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Path or filename of YOLO weights'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Topic name for input camera image'
    )

    conf_arg = DeclareLaunchArgument(
        'conf',
        default_value='0.5',
        description='Confidence threshold'
    )

    imgsz_arg = DeclareLaunchArgument(
        'imgsz',
        default_value='640',
        description='Inference image size (e.g. 640, 480, 320)'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='',
        description='Compute device (empty for auto, "cuda:0", "cpu")'
    )

    target_class_arg = DeclareLaunchArgument(
        'target_class',
        default_value='',
        description='Filter specific target class name (empty for all)'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_config_path,
        description='Path to YAML parameter file'
    )

    # YOLO Detector Node
    yolo_node = Node(
        package='robot0_vision',
        executable='yolo_detector',
        name='yolo_detector_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'model_path': LaunchConfiguration('model_path'),
                'image_topic': LaunchConfiguration('image_topic'),
                'confidence_threshold': LaunchConfiguration('conf'),
                'imgsz': LaunchConfiguration('imgsz'),
                'device': LaunchConfiguration('device'),
                'target_class': LaunchConfiguration('target_class'),
            }
        ]
    )

    return LaunchDescription([
        model_path_arg,
        image_topic_arg,
        conf_arg,
        imgsz_arg,
        device_arg,
        target_class_arg,
        params_file_arg,
        yolo_node
    ])

