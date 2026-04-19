import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package path
    pkg_vision = get_package_share_directory('vision')
    
    # Declare launch arguments
    use_baseline = LaunchConfiguration('use_baseline')
    declare_use_baseline_cmd = DeclareLaunchArgument(
        'use_baseline',
        default_value='false',
        description='Whether to use classic CV baseline instead of U-Net'
    )
    
    # Config file
    params_file = os.path.join(pkg_vision, 'config', 'vision_params.yaml')
    
    # Segmentation node
    segmentation_node = Node(
        package='vision',
        executable='segmentation_node',
        name='segmentation_node',
        output='screen',
        parameters=[params_file],
        condition=LaunchConfiguration('use_baseline') == 'false'
    )
    
    # Baseline detector node
    baseline_detector_node = Node(
        package='vision',
        executable='baseline_detector_node',
        name='baseline_detector_node',
        output='screen',
        parameters=[params_file],
        condition=LaunchConfiguration('use_baseline') == 'true'
    )

    return LaunchDescription([
        declare_use_baseline_cmd,
        segmentation_node,
        baseline_detector_node
    ])
