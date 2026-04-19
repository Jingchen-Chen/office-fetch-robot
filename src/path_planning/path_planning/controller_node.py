import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Parameters
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('goal_tolerance', 0.1)

        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # State
        self.path = None
        self.current_pose = None
        self.active = False

        # Subscriptions
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/pose/refined', self.pose_callback, 10)
        self.aruco_sub = self.create_subscription(PoseStamped, '/aruco/pose', self.pose_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/navigation/status', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Controller Node initialized')

    def path_callback(self, msg):
        self.path = msg
        self.active = True
        self.get_logger().info('New path received')

    def pose_callback(self, msg):
        self.current_pose = msg

    def get_euler_yaw(self, q):
        # Quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if not self.active or self.path is None or self.current_pose is None:
            return

        robot_x = self.current_pose.pose.position.x
        robot_y = self.current_pose.pose.position.y
        robot_yaw = self.get_euler_yaw(self.current_pose.pose.orientation)

        goal_pose = self.path.poses[-1].pose
        dist_to_goal = math.sqrt((goal_pose.position.x - robot_x)**2 + (goal_pose.position.y - robot_y)**2)

        if dist_to_goal < self.goal_tolerance:
            self.stop_robot("Goal reached")
            return

        # Find lookahead point
        lookahead_point = self.get_lookahead_point(robot_x, robot_y)
        
        # Compute heading error
        angle_to_target = math.atan2(lookahead_point[1] - robot_y, lookahead_point[0] - robot_x)
        heading_error = angle_to_target - robot_yaw
        
        # Normalize angle
        while heading_error > math.pi: heading_error -= 2 * math.pi
        while heading_error < -math.pi: heading_error += 2 * math.pi

        # Velocity commands
        cmd = Twist()
        
        # Linear velocity (proportional to distance, capped by max)
        cmd.linear.x = min(self.max_linear_vel, 0.5 * dist_to_goal)
        
        # Angular velocity (proportional to heading error)
        cmd.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, 2.0 * heading_error))

        # If heading error is too large, rotate in place first
        if abs(heading_error) > 0.5:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)
        
        status = String()
        status.data = f"Moving to goal, dist: {dist_to_goal:.2f}m"
        self.status_pub.publish(status)

    def get_lookahead_point(self, rx, ry):
        # Simplest: find point on path closest to lookahead distance
        best_pt = (self.path.poses[-1].pose.position.x, self.path.poses[-1].pose.position.y)
        
        for pose in reversed(self.path.poses):
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.sqrt((px - rx)**2 + (py - ry)**2)
            if dist <= self.lookahead_distance:
                return (px, py)
        
        return best_pt

    def stop_robot(self, reason):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.active = False
        
        status = String()
        status.data = reason
        self.status_pub.publish(status)
        self.get_logger().info(reason)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
