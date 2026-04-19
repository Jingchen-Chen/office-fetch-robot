import rclpy
from rclpy.node import Node
import numpy as np
import heapq

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        # Parameters
        self.declare_parameter('inflation_radius', 0.3)
        self.declare_parameter('planning_rate', 1.0)

        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.planning_rate = self.get_parameter('planning_rate').value

        # State
        self.map_data = None
        self.current_pose = None
        self.goal_pose = None

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/pose/refined', self.pose_callback, 10)
        self.aruco_sub = self.create_subscription(PoseStamped, '/aruco/pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/planning/markers', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.planning_rate, self.plan)

        self.get_logger().info('Planner Node initialized')

    def map_callback(self, msg):
        self.map_data = msg
        # Inflate obstacles
        self.inflated_grid = self.inflate_map(msg)

    def pose_callback(self, msg):
        self.current_pose = msg

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.get_logger().info('New goal received')

    def inflate_map(self, map_msg):
        grid = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        inflated = grid.copy()
        
        # Grid inflation
        res = map_msg.info.resolution
        cells = int(np.ceil(self.inflation_radius / res))
        
        rows, cols = np.where(grid > 50) # Obstacles
        for r, c in zip(rows, cols):
            for dr in range(-cells, cells + 1):
                for dc in range(-cells, cells + 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < map_msg.info.height and 0 <= nc < map_msg.info.width:
                        if np.sqrt(dr**2 + dc**2) * res <= self.inflation_radius:
                            inflated[nr, nc] = 100
        return inflated

    def world_to_grid(self, x, y):
        if not self.map_data: return None, None
        origin = self.map_data.info.origin.position
        res = self.map_data.info.resolution
        gx = int((x - origin.x) / res)
        gy = int((y - origin.y) / res)
        return gx, gy

    def grid_to_world(self, gx, gy):
        if not self.map_data: return None, None
        origin = self.map_data.info.origin.position
        res = self.map_data.info.resolution
        wx = gx * res + origin.x + res/2.0
        wy = gy * res + origin.y + res/2.0
        return wx, wy

    def is_valid(self, gx, gy):
        if not self.map_data: return False
        if 0 <= gx < self.map_data.info.width and 0 <= gy < self.map_data.info.height:
            # Check for unknown (-1) or obstacle (100)
            val = self.inflated_grid[gy, gx]
            return val != -1 and val < 50
        return False

    def get_neighbors(self, gx, gy):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0: continue
                nx, ny = gx + dx, gy + dy
                if self.is_valid(nx, ny):
                    cost = np.sqrt(dx**2 + dy**2)
                    neighbors.append(((nx, ny), cost))
        return neighbors

    def heuristic(self, a, b):
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def plan(self):
        if self.map_data is None or self.current_pose is None or self.goal_pose is None:
            return

        start_x, start_y = self.current_pose.pose.position.x, self.current_pose.pose.position.y
        goal_x, goal_y = self.goal_pose.pose.position.x, self.goal_pose.pose.position.y

        sgx, sgy = self.world_to_grid(start_x, start_y)
        ggx, ggy = self.world_to_grid(goal_x, goal_y)

        if not self.is_valid(sgx, sgy):
            self.get_logger().warn('Start position is invalid')
            return
        if not self.is_valid(ggx, ggy):
            self.get_logger().warn('Goal position is invalid')
            return

        # A* algorithm
        start = (sgx, sgy)
        goal = (ggx, ggy)
        
        pq = [(0, start)]
        came_from = {}
        cost_so_far = {start: 0}

        while pq:
            _, current = heapq.heappop(pq)

            if current == goal:
                break

            for next_node, move_cost in self.get_neighbors(*current):
                new_cost = cost_so_far[current] + move_cost
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(goal, next_node)
                    heapq.heappush(pq, (priority, next_node))
                    came_from[next_node] = current

        if goal not in came_from:
            self.get_logger().error('Path not found')
            return

        # Reconstruct path
        path_msg = Path()
        path_msg.header.frame_id = self.map_data.header.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        curr = goal
        while curr != start:
            wx, wy = self.grid_to_world(*curr)
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            path_msg.poses.append(ps)
            curr = came_from[curr]
        
        # Add start
        wx, wy = self.grid_to_world(*start)
        ps = PoseStamped()
        ps.header = path_msg.header
        ps.pose.position.x = wx
        ps.pose.position.y = wy
        path_msg.poses.append(ps)

        path_msg.poses.reverse()
        self.path_pub.publish(path_msg)
        self.publish_markers(path_msg)

    def publish_markers(self, path):
        markers = MarkerArray()
        marker = Marker()
        marker.header = path.header
        marker.ns = 'path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        for p in path.poses:
            marker.points.append(p.pose.position)
        
        markers.markers.append(marker)
        self.marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
