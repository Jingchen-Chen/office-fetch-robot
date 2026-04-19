import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json

class BaselineDetectorNode(Node):
    def __init__(self):
        super().__init__('baseline_detector_node')
        
        # Declare parameters
        # Example HSV for red: lower [0, 100, 100], upper [10, 255, 255]
        self.declare_parameter('target_color_lower', [0, 100, 100])
        self.declare_parameter('target_color_upper', [10, 255, 255])
        self.declare_parameter('min_contour_area', 500)
        
        # Get parameters
        self.lower_hsv = np.array(self.get_parameter('target_color_lower').value)
        self.upper_hsv = np.array(self.get_parameter('target_color_upper').value)
        self.min_area = self.get_parameter('min_contour_area').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Subscriptions and Publishers
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.detection_publisher = self.create_publisher(String, '/detection/objects', 10)
        self.debug_publisher = self.create_publisher(Image, '/detection/debug_image', 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')
            return
            
        # Color-based segmentation
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, self.lower_hsv, self.upper_hsv)
        
        # Morphological operations to clean up
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        debug_image = cv_image.copy()
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_area:
                # Calculate centroid
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    # Basic shape detection
                    # Approximate the contour
                    peri = cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
                    
                    shape = "unknown"
                    if len(approx) == 3:
                        shape = "triangle"
                    elif len(approx) == 4:
                        # Could be rectangle or square
                        (x, y, w, h) = cv2.boundingRect(approx)
                        ar = w / float(h)
                        shape = "square" if 0.95 <= ar <= 1.05 else "rectangle"
                    elif len(approx) == 5:
                        shape = "pentagon"
                    else:
                        shape = "circle"
                        
                    detections.append({
                        'class_id': 1,  # In baseline, we're detecting 1 target class by default
                        'shape': shape,
                        'centroid_x': cx,
                        'centroid_y': cy,
                        'area': float(area)
                    })
                    
                    # Draw for debug
                    cv2.drawContours(debug_image, [cnt], -1, (0, 255, 0), 2)
                    cv2.circle(debug_image, (cx, cy), 7, (255, 255, 255), -1)
                    cv2.putText(debug_image, shape, (cx - 20, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Publish detections
        detection_msg = String()
        detection_msg.data = json.dumps(detections)
        self.detection_publisher.publish(detection_msg)
        
        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_publisher.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BaselineDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
