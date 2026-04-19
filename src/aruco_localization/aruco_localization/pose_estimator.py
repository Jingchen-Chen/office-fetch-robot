#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
import tf2_ros
import json
import numpy as np
import cv2
import yaml
import os
import math

def euler_to_matrix(roll, pitch, yaw):
    """Calculate 3x3 rotation matrix from RPY (intrinsic ZYX convention)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    
    R_x = np.array([[1.0, 0.0, 0.0],
                    [0.0, cr, -sr],
                    [0.0, sr, cr]])
    R_y = np.array([[cp, 0.0, sp],
                    [0.0, 1.0, 0.0],
                    [-sp, 0.0, cp]])
    R_z = np.array([[cy, -sy, 0.0],
                    [sy, cy, 0.0],
                    [0.0, 0.0, 1.0]])
    
    # R = Rz * Ry * Rx
    return R_z @ R_y @ R_x

def matrix_to_quaternion(R):
    """Convert a 3x3 rotation matrix to a quaternion (x, y, z, w)."""
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S
    elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
        S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
        qw = (R[2,1] - R[1,2]) / S
        qx = 0.25 * S
        qy = (R[0,1] + R[1,0]) / S
        qz = (R[0,2] + R[2,0]) / S
    elif R[1,1] > R[2,2]:
        S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
        qw = (R[0,2] - R[2,0]) / S
        qx = (R[0,1] + R[1,0]) / S
        qy = 0.25 * S
        qz = (R[1,2] + R[2,1]) / S
    else:
        S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
        qw = (R[1,0] - R[0,1]) / S
        qx = (R[0,2] + R[2,0]) / S
        qy = (R[1,2] + R[2,1]) / S
        qz = 0.25 * S
    return [qx, qy, qz, qw]

def transform_to_matrix(t):
    """Convert a ROS2 Transform object to a 4x4 matrix."""
    T = np.eye(4)
    q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
    
    # Manually compute rotation matrix from quaternion
    qx, qy, qz, qw = q
    r00 = 1 - 2*(qy*qy + qz*qz)
    r01 = 2*(qx*qy - qz*qw)
    r02 = 2*(qx*qz + qy*qw)
    r10 = 2*(qx*qy + qz*qw)
    r11 = 1 - 2*(qx*qx + qz*qz)
    r12 = 2*(qy*qz - qx*qw)
    r20 = 2*(qx*qz - qy*qw)
    r21 = 2*(qy*qz + qx*qw)
    r22 = 1 - 2*(qx*qx + qy*qy)
    
    T[:3, :3] = np.array([[r00, r01, r02],
                          [r10, r11, r12],
                          [r20, r21, r22]])
    T[0, 3] = t.transform.translation.x
    T[1, 3] = t.transform.translation.y
    T[2, 3] = t.transform.translation.z
    return T

class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        
        # Parameters
        self.declare_parameter('marker_map_file', '')
        # Default camera matrix for typical ROS2 simulation
        self.declare_parameter('camera_matrix', [928.33, 0.0, 640.0, 0.0, 928.33, 360.0, 0.0, 0.0, 1.0])
        self.declare_parameter('dist_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('marker_length', 0.15)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('map_frame', 'map')

        marker_map_file = self.get_parameter('marker_map_file').value
        camera_matrix_flat = self.get_parameter('camera_matrix').value
        self.camera_matrix = np.array(camera_matrix_flat, dtype=np.float32).reshape((3, 3))
        self.dist_coeffs = np.array(self.get_parameter('dist_coeffs').value, dtype=np.float32)
        self.marker_length = self.get_parameter('marker_length').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.map_frame = self.get_parameter('map_frame').value

        # Load marker map
        self.marker_map = {}
        if marker_map_file:
            try:
                with open(marker_map_file, 'r') as f:
                    content = yaml.safe_load(f)
                    # Convert keys to int if necessary
                    self.marker_map = {int(k): v for k, v in content.get('markers', {}).items()}
                self.get_logger().info(f'Loaded {len(self.marker_map)} markers from {marker_map_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to load marker map: {e}')

        # TF components
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.subscription = self.create_subscription(
            String,
            '/aruco/markers',
            self.markers_callback,
            10)
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)

        self.get_logger().info('Pose Estimator Node started')

    def markers_callback(self, msg):
        try:
            detections = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Failed to parse marker data: {e}')
            return

        if not detections:
            return

        obj_points = []
        img_points = []

        for det in detections:
            marker_id = int(det['id'])
            if marker_id in self.marker_map:
                marker_cfg = self.marker_map[marker_id]
                pos = marker_cfg['position']
                orient = marker_cfg['orientation'] # [roll, pitch, yaw]
                
                # Get corners in marker local frame (centered, facing +Z)
                half_len = self.marker_length / 2.0
                local_corners = np.array([
                    [-half_len,  half_len, 0.0],
                    [ half_len,  half_len, 0.0],
                    [ half_len, -half_len, 0.0],
                    [-half_len, -half_len, 0.0]
                ])

                # Rotate and translate corners to map frame
                # RPY in world coordinate
                rot_matrix = euler_to_matrix(orient[0], orient[1], orient[2])
                
                for pt in local_corners:
                    world_pt = rot_matrix @ pt + np.array(pos)
                    obj_points.append(world_pt)
                
                for pt in det['corners']:
                    img_points.append(pt)

        if len(obj_points) < 4:
            return

        # Solve PnP - estimates transform from Map Frame to Camera Frame
        success, rvec, tvec = cv2.solvePnP(
            np.array(obj_points, dtype=np.float32),
            np.array(img_points, dtype=np.float32),
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if success:
            rmat, _ = cv2.Rodrigues(rvec)
            
            # Map -> Camera
            T_map_cam = np.eye(4)
            T_map_cam[:3, :3] = rmat
            T_map_cam[:3, 3] = tvec.flatten()
            
            # Camera -> Map
            T_cam_map = np.linalg.inv(T_map_cam)
            
            try:
                # Lookup transform from base_link to camera_link
                # This transform is assumed to be part of the robot's static TF tree
                t_base_cam = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, rclpy.time.Time())
                T_base_cam_mat = transform_to_matrix(t_base_cam)
                
                # T_map_base = T_map_cam^-1 * T_base_cam^-1? No.
                # p_map = T_cam_map * p_cam
                # p_cam = T_base_cam^-1 * p_base
                # p_map = T_cam_map * T_base_cam^-1 * p_base
                # => T_map_base = T_cam_map * T_base_cam^-1
                T_map_base = T_cam_map @ np.linalg.inv(T_base_cam_mat)
                
                # Extract results
                base_pos = T_map_base[:3, 3]
                base_quat = matrix_to_quaternion(T_map_base[:3, :3])
                
                now = self.get_clock().now().to_msg()
                
                # Publish PoseStamped
                pose_msg = PoseStamped()
                pose_msg.header.stamp = now
                pose_msg.header.frame_id = self.map_frame
                pose_msg.pose.position.x = base_pos[0]
                pose_msg.pose.position.y = base_pos[1]
                pose_msg.pose.position.z = base_pos[2]
                pose_msg.pose.orientation.x = base_quat[0]
                pose_msg.pose.orientation.y = base_quat[1]
                pose_msg.pose.orientation.z = base_quat[2]
                pose_msg.pose.orientation.w = base_quat[3]
                self.pose_pub.publish(pose_msg)
                
                # Publish TF: map -> base_link
                tf_msg = TransformStamped()
                tf_msg.header.stamp = now
                tf_msg.header.frame_id = self.map_frame
                tf_msg.child_frame_id = self.base_frame
                tf_msg.transform.translation.x = base_pos[0]
                tf_msg.transform.translation.y = base_pos[1]
                tf_msg.transform.translation.z = base_pos[2]
                tf_msg.transform.rotation.x = base_quat[0]
                tf_msg.transform.rotation.y = base_quat[1]
                tf_msg.transform.rotation.z = base_quat[2]
                tf_msg.transform.rotation.w = base_quat[3]
                self.tf_broadcaster.sendTransform(tf_msg)
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f'Could not lookup transform {self.base_frame} to {self.camera_frame}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
