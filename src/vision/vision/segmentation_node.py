import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch
import numpy as np
import cv2
import json
from .unet_model import UNet
from torchvision import transforms

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')
        
        # Declare parameters
        self.declare_parameter('model_path', 'best_model.pth')
        self.declare_parameter('num_classes', 2)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.num_classes = self.get_parameter('num_classes').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.device_name = self.get_parameter('device').value
        self.device = torch.device(self.device_name)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Load Model
        self.get_logger().info(f'Loading model from {self.model_path} on {self.device_name}...')
        self.model = UNet(in_channels=3, num_classes=self.num_classes).to(self.device)
        try:
            self.model.load_state_dict(torch.load(self.model_path, map_location=self.device))
            self.get_logger().info('Model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            
        self.model.eval()
        
        # Image transformation
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        
        # Subscriptions and Publishers
        self.subscription = self.create_subscription(
            Image,
            '/P1_robot0/cam_front/image_raw',
            self.image_callback,
            10
        )
        
        self.mask_publisher = self.create_publisher(Image, '/segmentation/mask', 10)
        self.detection_publisher = self.create_publisher(String, '/detection/objects', 10)

    def image_callback(self, msg):
        # Convert ROS image to CV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')
            return
            
        # Preprocess image
        # Standard U-Net expects certain dimensions (multiples of 16 or 32)
        # For inference, resize image if necessary
        original_h, original_w = cv_image.shape[:2]
        
        # Convert to RGB and transform
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        input_tensor = self.transform(rgb_image).unsqueeze(0).to(self.device)
        
        # Inference
        with torch.no_grad():
            output = self.model(input_tensor)
            
            # Use softmax if num_classes > 1
            if self.num_classes > 1:
                probs = torch.softmax(output, dim=1)
                mask = torch.argmax(probs, dim=1).squeeze(0).cpu().numpy()
            else:
                probs = torch.sigmoid(output)
                mask = (probs > self.confidence_threshold).squeeze(0).squeeze(0).cpu().numpy().astype(np.uint8)

        # Post-process for detection (class_id, centroid_x, centroid_y, area)
        detections = []
        
        # Loop through classes (ignoring background, assume class 0 is background)
        for class_id in range(1, self.num_classes):
            class_mask = (mask == class_id).astype(np.uint8)
            
            # Find contours
            contours, _ = cv2.findContours(class_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 100:  # Minimum area threshold
                    M = cv2.moments(cnt)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        
                        detections.append({
                            'class_id': class_id,
                            'centroid_x': cx,
                            'centroid_y': cy,
                            'area': float(area)
                        })

        # Publish detections
        detection_msg = String()
        detection_msg.data = json.dumps(detections)
        self.detection_publisher.publish(detection_msg)
        
        # Publish mask image
        # For visualization, multiply mask by 255 if binary, or scale by class id
        mask_vis = (mask * (255 // max(1, self.num_classes - 1))).astype(np.uint8)
        mask_msg = self.bridge.cv2_to_imgmsg(mask_vis, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_publisher.publish(mask_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
