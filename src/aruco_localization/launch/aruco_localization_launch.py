import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_localization')
    
    marker_map_file = os.path.join(pkg_share, 'config', 'marker_map.yaml')
    
    # Common parameters for the Gazebo camera
    # These can be overridden if needed
    camera_matrix = [928.33, 0.0, 640.0, 0.0, 928.33, 360.0, 0.0, 0.0, 1.0]
    dist_coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]
    marker_length = 0.15
    
    detector_node = Node(
        package='aruco_localization',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'marker_length': marker_length,
            'camera_matrix': camera_matrix,
            'dist_coeffs': dist_coeffs
        }]
    )
    
    estimator_node = Node(
        package='aruco_localization',
        executable='pose_estimator',
        name='pose_estimator',
        output='screen',
        parameters=[{
            'marker_map_file': marker_map_file,
            'marker_length': marker_length,
            'camera_matrix': camera_matrix,
            'dist_coeffs': dist_coeffs,
            'base_frame': 'base_link',
            'camera_frame': 'camera_link',
            'map_frame': 'map'
        }]
    )
    
    return LaunchDescription([
        detector_node,
        estimator_node
    ])
