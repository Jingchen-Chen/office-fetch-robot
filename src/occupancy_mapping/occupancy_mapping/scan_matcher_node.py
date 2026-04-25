import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion
import tf_transformations
from scipy.spatial import KDTree

class ScanMatcherNode(Node):
    def __init__(self):
        super().__init__('scan_matcher_node')

        # Parameters
        self.declare_parameter('max_iterations', 50)
        self.declare_parameter('convergence_threshold', 0.001)
        self.declare_parameter('max_correspondence_distance', 0.5)

        self.max_iter = self.get_parameter('max_iterations').value
        self.convergence_threshold = self.get_parameter('convergence_threshold').value
        self.max_dist = self.get_parameter('max_correspondence_distance').value

        # State
        self.map_data = None
        self.map_points = None
        self.map_kdtree = None
        self.current_pose = np.array([0.0, 0.0, 0.0]) # x, y, yaw

        # Subscriptions
        self.scan_sub = self.create_subscription(LaserScan, '/P1_robot0/lidar_2d', self.scan_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/pose/refined', 10)

        self.get_logger().info('Scan Matcher Node initialized')

    def map_callback(self, msg):
        self.map_data = msg
        # Extract occupied cells (e.g., probability > 50)
        grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        occupied_indices = np.argwhere(grid > 50)
        
        # Convert grid indices to world coordinates
        self.map_points = []
        for r, c in occupied_indices:
            x = c * msg.info.resolution + msg.info.origin.position.x
            y = r * msg.info.resolution + msg.info.origin.position.y
            self.map_points.append([x, y])
        
        if len(self.map_points) > 0:
            self.map_points = np.array(self.map_points)
            self.map_kdtree = KDTree(self.map_points)
            self.get_logger().info(f'Map updated with {len(self.map_points)} points')

    def scan_callback(self, msg):
        if self.map_kdtree is None:
            return

        # Convert scan to points in robot frame
        angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)
        # Handle cases where lengths might differ due to floating point precision
        if len(angles) > len(msg.ranges):
            angles = angles[:len(msg.ranges)]
        elif len(angles) < len(msg.ranges):
            ranges = np.array(msg.ranges)[:len(angles)]
        else:
            ranges = np.array(msg.ranges)
            
        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        scan_points_local = np.stack([
            ranges[valid] * np.cos(angles[valid]),
            ranges[valid] * np.sin(angles[valid]),
            np.ones_like(ranges[valid])
        ], axis=1) # [N, 3] for homogeneous coords

        # ICP loop
        pose_refine = self.current_pose.copy()
        
        for i in range(self.max_iter):
            # Transform scan points to world frame using current estimate
            tx, ty, yaw = pose_refine
            T = np.array([
                [np.cos(yaw), -np.sin(yaw), tx],
                [np.sin(yaw),  np.cos(yaw), ty],
                [0, 0, 1]
            ])
            scan_points_world = (T @ scan_points_local.T).T[:, :2] # [N, 2]

            # Find nearest neighbors
            distances, indices = self.map_kdtree.query(scan_points_world)
            
            # Filter matches by distance
            mask = distances < self.max_dist
            if np.sum(mask) < 3:
                break
                
            src = scan_points_world[mask]
            dst = self.map_points[indices[mask]]

            # Estimate transformation between scan_points_world and map_points
            # We want to find d_tx, d_ty, d_yaw such that T_delta @ scan_points_world matches dst
            # But it's easier to use the standard SVD-based rigid body registration
            # on the local points vs map points.
            
            # Center of mass
            mu_src = np.mean(src, axis=0)
            mu_dst = np.mean(dst, axis=0)
            
            # SVD for rotation
            H = (src - mu_src).T @ (dst - mu_dst)
            U, S, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            
            # Correct for reflection if necessary
            if np.linalg.det(R) < 0:
                Vt[1, :] *= -1
                R = Vt.T @ U.T
            
            t = mu_dst - R @ mu_src
            
            # Update refined pose
            # New T = T_delta @ T_old
            T_delta = np.eye(3)
            T_delta[:2, :2] = R
            T_delta[:2, 2] = t
            T_new = T_delta @ T
            
            tx_new = T_new[0, 2]
            ty_new = T_new[1, 2]
            yaw_new = np.arctan2(T_new[1, 0], T_new[0, 0])
            
            delta_norm = np.sqrt((tx_new - tx)**2 + (ty_new - ty)**2 + (yaw_new - yaw)**2)
            
            pose_refine = np.array([tx_new, ty_new, yaw_new])
            
            if delta_norm < self.convergence_threshold:
                break

        self.current_pose = pose_refine
        self.publish_refined_pose(pose_refine)

    def publish_refined_pose(self, pose):
        tx, ty, yaw = pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = tx
        msg.pose.position.y = ty
        
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanMatcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
