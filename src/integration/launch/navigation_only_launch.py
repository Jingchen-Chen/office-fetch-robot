import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_integration = get_package_share_directory('integration')
    pkg_aruco = get_package_share_directory('aruco_localization')
    pkg_mapping = get_package_share_directory('occupancy_mapping')
    pkg_planning = get_package_share_directory('path_planning')
    pkg_sim = get_package_share_directory('ipb_ros2_sim')

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_aruco, 'launch', 'aruco_localization_launch.py'))
    )
    
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_mapping, 'launch', 'occupancy_mapping_launch.py'))
    )
    
    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_planning, 'launch', 'path_planning_launch.py'))
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sim, 'launch', 'all.launch.py')),
        launch_arguments={'sim_config': 'indoor'}.items()
    )

    # RViz (still useful)
    rviz_config = os.path.join(pkg_integration, 'config', 'rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        aruco_launch,
        mapping_launch,
        planning_launch,
        rviz_node
    ])
