import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_vision = get_package_share_directory('vision')

    # Vision node (assuming segmentation_node or similar)
    vision_node = Node(
        package='vision',
        executable='segmentation_node',  # As specified in requirements
        name='segmentation_node',
        output='screen'
    )

    return LaunchDescription([
        vision_node
    ])
