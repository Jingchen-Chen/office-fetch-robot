#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Parameters
        self.declare_parameter('marker_length', 0.15)
        self.declare_parameter('camera_matrix', [928.33, 0.0, 640.0, 0.0, 928.33, 360.0, 0.0, 0.0, 1.0])
        self.declare_parameter('dist_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.marker_length = self.get_parameter('marker_length').value
        camera_matrix_flat = self.get_parameter('camera_matrix').value
        self.camera_matrix = np.array(camera_matrix_flat).reshape((3, 3))
        self.dist_coeffs = np.array(self.get_parameter('dist_coeffs').value)
        
        # Subscriptions
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Publishers
        self.marker_pub = self.create_publisher(String, '/aruco/markers', 10)
        self.debug_pub = self.create_publisher(Image, '/aruco/debug_image', 10)
        
        self.bridge = CvBridge()
        
        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # For newer OpenCV (4.7.0+), ArucoDetector is preferred. 
        # For compatibility with older versions in Humble, we use the legacy API or check availability.
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.detect_markers = self.detector.detectMarkers
        else:
            self.detect_markers = lambda img: cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)

        self.get_logger().info('Aruco Detector Node started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # Detect markers
        corners, ids, rejected = self.detect_markers(cv_image)

        if ids is not None:
            # Draw detected markers on the image for debugging
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            
            markers_data = []
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                # corners[i] is (1, 4, 2)
                marker_corners = corners[i][0].tolist() 
                markers_data.append({
                    'id': marker_id,
                    'corners': marker_corners
                })
            
            # Publish detection results as JSON string
            marker_msg = String()
            marker_msg.data = json.dumps(markers_data)
            self.marker_pub.publish(marker_msg)
            
            # Log detections
            self.get_logger().debug(f'Detected markers: {[m["id"] for m in markers_data]}')

        # Publish debug image
        debug_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        debug_image_msg.header = msg.header
        self.debug_pub.publish(debug_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
