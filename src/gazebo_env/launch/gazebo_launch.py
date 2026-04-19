import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'gazebo_env'
    pkg_share = get_package_share_directory(package_name)

    # World file
    world_file = os.path.join(pkg_share, 'worlds', 'office.world')

    # Xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'fetch_robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

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

    # Set Gazebo model path to include our models
    models_path = os.path.join(pkg_share, 'models')
    # Prepend to GAZEBO_MODEL_PATH
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ':' + models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = models_path

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])
