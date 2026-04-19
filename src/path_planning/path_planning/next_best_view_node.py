import rclpy
from rclpy.node import Node
import numpy as np
import math
from collections import deque

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

class NextBestViewNode(Node):
    def __init__(self):
        super().__init__('next_best_view_node')

        # Parameters
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('exploration_gain_weight', 1.0)
        self.declare_parameter('distance_cost_weight', 0.5)

        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.gain_weight = self.get_parameter('exploration_gain_weight').value
        self.dist_weight = self.get_parameter('distance_cost_weight').value

        # State
        self.map_data = None
        self.current_pose = None
        self.exploring = True

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/pose/refined', self.pose_callback, 10)
        self.aruco_sub = self.create_subscription(PoseStamped, '/aruco/pose', self.pose_callback, 10)

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/frontiers/markers', 10)

        # Timer for exploration logic (run every 5 seconds)
        self.timer = self.create_timer(5.0, self.exploration_step)

        self.get_logger().info('Next Best View Node initialized')

    def map_callback(self, msg):
        self.map_data = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def exploration_step(self):
        if self.map_data is None or self.current_pose is None or not self.exploring:
            return

        frontiers = self.find_frontiers()
        if not frontiers:
            self.get_logger().info('No more frontiers found. Exploration complete.')
            self.exploring = False
            return

        best_frontier = self.select_best_frontier(frontiers)
        if best_frontier:
            self.publish_goal(best_frontier)
            self.publish_markers(frontiers)

    def find_frontiers(self):
        width = self.map_data.info.width
        height = self.map_data.info.height
        data = np.array(self.map_data.data).reshape((height, width))
        
        frontier_cells = []
        # Find all cells that are free (0) and have at least one unknown (-1) neighbor
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if data[y, x] == 0:
                    # Check 4-neighbors for unknown
                    if -1 in [data[y+1, x], data[y-1, x], data[y, x+1], data[y, x-1]]:
                        frontier_cells.append((x, y))

        if not frontier_cells:
            return []

        # Cluster frontier cells using BFS
        clusters = []
        visited = set()
        for cell in frontier_cells:
            if cell in visited:
                continue
            
            cluster = []
            queue = deque([cell])
            visited.add(cell)
            
            while queue:
                curr = queue.popleft()
                cluster.append(curr)
                
                # Check neighbors in frontier_cells
                for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                    neighbor = (curr[0] + dx, curr[1] + dy)
                    if neighbor in frontier_cells and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)
            
            if len(cluster) >= self.min_frontier_size:
                clusters.append(cluster)
        
        return clusters

    def select_best_frontier(self, frontiers):
        robot_x = self.current_pose.pose.position.x
        robot_y = self.current_pose.pose.position.y
        
        best_score = -float('inf')
        best_f = None
        
        res = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        for cluster in frontiers:
            # Centroid
            cx = sum(c[0] for c in cluster) / len(cluster)
            cy = sum(c[1] for c in cluster) / len(cluster)
            
            wx = cx * res + origin_x
            wy = cy * res + origin_y
            
            dist = math.sqrt((wx - robot_x)**2 + (wy - robot_y)**2)
            gain = len(cluster) # Information gain proportional to size
            
            score = (self.gain_weight * gain) - (self.dist_weight * dist)
            
            if score > best_score:
                best_score = score
                best_f = (wx, wy)
                
        return best_f

    def publish_goal(self, coords):
        msg = PoseStamped()
        msg.header.frame_id = self.map_data.header.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = coords[0]
        msg.pose.position.y = coords[1]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Published new exploration goal: {coords[0]:.2f}, {coords[1]:.2f}')

    def publish_markers(self, frontiers):
        markers = MarkerArray()
        res = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        for i, cluster in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = self.map_data.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            for cell in cluster:
                p = Point()
                p.x = cell[0] * res + origin_x
                p.y = cell[1] * res + origin_y
                p.z = 0.1
                marker.points.append(p)
            
            markers.markers.append(marker)
        
        # Delete old markers
        for i in range(len(frontiers), 100):
            m = Marker()
            m.header.frame_id = self.map_data.header.frame_id
            m.ns = 'frontiers'
            m.id = i
            m.action = Marker.DELETE
            markers.markers.append(m)

        self.marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = NextBestViewNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
