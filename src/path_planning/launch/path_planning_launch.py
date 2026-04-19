import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('path_planning')
    config_file = os.path.join(pkg_share, 'config', 'planning_params.yaml')

    return LaunchDescription([
        Node(
            package='path_planning',
            executable='planner_node',
            name='planner_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='path_planning',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='path_planning',
            executable='next_best_view_node',
            name='next_best_view_node',
            output='screen',
            parameters=[config_file]
        )
    ])
