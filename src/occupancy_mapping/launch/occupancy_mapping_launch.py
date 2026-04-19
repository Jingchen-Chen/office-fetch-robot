import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('occupancy_mapping')
    config_file = os.path.join(pkg_share, 'config', 'mapping_params.yaml')

    occupancy_grid_node = Node(
        package='occupancy_mapping',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[config_file]
    )

    scan_matcher_node = Node(
        package='occupancy_mapping',
        executable='scan_matcher_node',
        name='scan_matcher_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        occupancy_grid_node,
        scan_matcher_node
    ])
