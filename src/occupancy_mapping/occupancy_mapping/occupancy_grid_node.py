import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import tf_transformations

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')

        # Parameters
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width', 200)
        self.declare_parameter('height', 200)
        self.declare_parameter('origin_x', -5.0)
        self.declare_parameter('origin_y', -5.0)
        self.declare_parameter('log_odds_free', -0.4)
        self.declare_parameter('log_odds_occupied', 0.85)
        self.declare_parameter('log_odds_prior', 0.0)
        self.declare_parameter('publish_rate', 1.0)

        self.res = self.get_parameter('resolution').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.lo_free = self.get_parameter('log_odds_free').value
        self.lo_occupied = self.get_parameter('log_odds_occupied').value
        self.lo_prior = self.get_parameter('log_odds_prior').value
        
        # Initialize grid with prior
        self.grid_log_odds = np.full((self.height, self.width), self.lo_prior, dtype=np.float32)
        
        # Robot pose
        self.robot_pose = None

        # Subscriptions
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/aruco/pose', self.pose_callback, 10)
        
        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Timer for publishing map
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_map)

        self.get_logger().info('Occupancy Grid Node initialized')

    def pose_callback(self, msg):
        self.robot_pose = msg

    def scan_callback(self, msg):
        if self.robot_pose is None:
            return

        # Get robot position and orientation
        rx = self.robot_pose.pose.position.x
        ry = self.robot_pose.pose.position.y
        
        q = [
            self.robot_pose.pose.orientation.x,
            self.robot_pose.pose.orientation.y,
            self.robot_pose.pose.orientation.z,
            self.robot_pose.pose.orientation.w
        ]
        _, _, yaw = tf_transformations.euler_from_quaternion(q)

        # Robot cell coordinates
        cx0, cy0 = self.world_to_grid(rx, ry)
        
        if not self.is_in_grid(cx0, cy0):
            return

        # Process each laser ray
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        
        # Filter valid ranges
        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        angles = angles[valid]
        ranges = ranges[valid]

        for angle, dist in zip(angles, ranges):
            # Endpoint in world coordinates
            abs_angle = yaw + angle
            ex = rx + dist * np.cos(abs_angle)
            ey = ry + dist * np.sin(abs_angle)
            
            cx1, cy1 = self.world_to_grid(ex, ey)
            
            # Trace ray
            line = self.get_line(cx0, cy0, cx1, cy1)
            
            # Update free cells
            for i in range(len(line) - 1):
                lx, ly = line[i]
                if self.is_in_grid(lx, ly):
                    self.grid_log_odds[ly, lx] += self.lo_free
            
            # Update occupied cell if endpoint is in grid
            lx, ly = line[-1]
            if self.is_in_grid(lx, ly):
                self.grid_log_odds[ly, lx] += self.lo_occupied

        # Clamp log-odds to avoid numerical issues
        self.grid_log_odds = np.clip(self.grid_log_odds, -10.0, 10.0)

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.res)
        gy = int((y - self.origin_y) / self.res)
        return gx, gy

    def is_in_grid(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def get_line(self, x0, y0, x1, y1):
        """Bresenham's Line Algorithm"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1

        line = []
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                line.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                line.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        line.append((x, y))
        return line

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.info = MapMetaData()
        msg.info.resolution = self.res
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        
        # Convert log-odds to probability [0, 100]
        # p = 1 - 1 / (1 + exp(L))
        probs = 100 * (1 - 1 / (1 + np.exp(self.grid_log_odds)))
        msg.data = probs.astype(np.int8).flatten().tolist()
        
        self.map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
