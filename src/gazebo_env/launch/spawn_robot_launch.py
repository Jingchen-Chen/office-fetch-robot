import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'gazebo_env'
    pkg_share = get_package_share_directory(package_name)

    # Xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'fetch_robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'fetch_robot'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        spawn_entity
    ])
