import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Packages
    pkg_integration = get_package_share_directory('integration')
    pkg_aruco = get_package_share_directory('aruco_localization')
    pkg_mapping = get_package_share_directory('occupancy_mapping')
    pkg_planning = get_package_share_directory('path_planning')
    pkg_vision = get_package_share_directory('vision')
    pkg_sim = get_package_share_directory('ipb_ros2_sim')

    # Launch files
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_aruco, 'launch', 'aruco_localization_launch.py'))
    )
    
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_mapping, 'launch', 'occupancy_mapping_launch.py'))
    )
    
    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_planning, 'launch', 'path_planning_launch.py'))
    )
    
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vision, 'launch', 'vision_launch.py'))
    )
    
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sim, 'launch', 'all.launch.py')),
        launch_arguments={'sim_config': 'indoor'}.items()
    )

    # RViz
    rviz_config = os.path.join(pkg_integration, 'config', 'rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Mission Controller
    mission_controller_node = Node(
        package='integration',
        executable='mission_controller',
        name='mission_controller',
        parameters=[{
            'target_object_class': 1,
            'home_x': 0.0,
            'home_y': 0.0,
            'exploration_coverage_threshold': 0.9
        }],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        aruco_launch,
        mapping_launch,
        planning_launch,
        vision_launch,
        rviz_node,
        mission_controller_node
    ])
