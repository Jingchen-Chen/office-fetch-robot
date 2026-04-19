import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from vision_msgs.msg import Detection2DArray  # Assuming vision_msgs is used
import time

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # Parameters
        self.declare_parameter('target_object_class', 1)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('exploration_coverage_threshold', 0.9)
        
        self.target_class = self.get_parameter('target_object_class').value
        self.home_x = self.get_parameter('home_x').value
        self.home_y = self.get_parameter('home_y').value
        self.coverage_threshold = self.get_parameter('exploration_coverage_threshold').value
        
        # States
        self.states = ['IDLE', 'EXPLORING', 'MAPPING', 'SEARCHING', 'NAVIGATING_TO_OBJECT', 'FETCHING', 'RETURNING']
        self.current_state = 'IDLE'
        
        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/aruco/pose', self.pose_callback, 10)
        self.detection_sub = self.create_subscription(Detection2DArray, '/detection/objects', self.detection_callback, 10)
        self.nav_status_sub = self.create_subscription(String, '/navigation/status', self.nav_status_callback, 10)
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        
        # Internal variables
        self.current_pose = None
        self.latest_map = None
        self.nav_status = 'IDLE'
        self.detected_object_pose = None
        self.map_coverage = 0.0
        
        # Timer for state machine (10 Hz)
        self.timer = self.create_timer(0.1, self.update_state_machine)
        
        self.get_logger().info('Mission Controller initialized. Starting in IDLE state.')
        self.current_state = 'EXPLORING' # Start mission automatically

    def map_callback(self, msg):
        self.latest_map = msg
        # Simplified coverage calculation: ratio of known cells to total cells
        if msg.data:
            known_cells = sum(1 for cell in msg.data if cell != -1)
            self.map_coverage = known_cells / len(msg.data)

    def pose_callback(self, msg):
        self.current_pose = msg

    def detection_callback(self, msg):
        for detection in msg.detections:
            # Check if any detection matches target_class
            # This logic depends on how class ID is stored in Detection2D
            # Usually in detection.results[0].id
            if detection.results and detection.results[0].id == str(self.target_class):
                # If vision node provides 3D pose in Detection2D (unlikely but possible)
                # or if we just mark it as found.
                # For this simulation, we'll assume we found it.
                self.get_logger().info(f'Target object {self.target_class} detected!')
                # In a real system, we'd transform pixel to map coordinates.
                # Here we'll just simulate a pose near the robot for navigation logic.
                if self.current_pose:
                    self.detected_object_pose = PoseStamped()
                    self.detected_object_pose.header.frame_id = 'map'
                    self.detected_object_pose.pose.position.x = self.current_pose.pose.position.x + 1.0
                    self.detected_object_pose.pose.position.y = self.current_pose.pose.position.y
                    self.detected_object_pose.pose.orientation = self.current_pose.pose.orientation

    def nav_status_callback(self, msg):
        self.nav_status = msg.data

    def publish_status(self):
        msg = String()
        msg.data = self.current_state
        self.status_pub.publish(msg)

    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Sent goal to ({x}, {y})')

    def update_state_machine(self):
        self.publish_status()
        
        if self.current_state == 'IDLE':
            pass
            
        elif self.current_state == 'EXPLORING':
            # Logic: Send exploration goals (Next Best View handled by path_planning)
            # We just monitor coverage
            if self.map_coverage >= self.coverage_threshold:
                self.get_logger().info('Map coverage threshold reached. Switching to MAPPING.')
                self.current_state = 'MAPPING'
                
        elif self.current_state == 'MAPPING':
            # Logic: Wait until map is stable or refined
            # For simplicity, just wait a few seconds then start searching
            time.sleep(2.0)
            self.get_logger().info('Mapping complete. Switching to SEARCHING.')
            self.current_state = 'SEARCHING'
            
        elif self.current_state == 'SEARCHING':
            # Logic: Navigate to random points or patrol
            if self.detected_object_pose:
                self.get_logger().info('Object found! Navigating to object.')
                self.current_state = 'NAVIGATING_TO_OBJECT'
                self.send_goal(self.detected_object_pose.pose.position.x, 
                               self.detected_object_pose.pose.position.y)
            else:
                # Send a search goal if idle
                if self.nav_status == 'IDLE' or self.nav_status == 'REACHED':
                    # Simplified: just move 2 meters ahead
                    if self.current_pose:
                        self.send_goal(self.current_pose.pose.position.x + 2.0, 
                                       self.current_pose.pose.position.y)
            
        elif self.current_state == 'NAVIGATING_TO_OBJECT':
            if self.nav_status == 'REACHED':
                self.get_logger().info('Reached object. Starting FETCHING.')
                self.current_state = 'FETCHING'
                
        elif self.current_state == 'FETCHING':
            # Logic: Simplified - wait 3 seconds to simulate pick up
            self.get_logger().info('Fetching object...')
            time.sleep(3.0)
            self.get_logger().info('Object fetched. Returning home.')
            self.current_state = 'RETURNING'
            self.send_goal(self.home_x, self.home_y)
            
        elif self.current_state == 'RETURNING':
            if self.nav_status == 'REACHED':
                self.get_logger().info('Mission accomplished! Back at home.')
                self.current_state = 'IDLE'

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
