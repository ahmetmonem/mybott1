#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedDetector(Node):
    def __init__(self):
        super().__init__('red_detector')
        
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.fire_pub = self.create_publisher(Point, '/detected_fire', 10)
        self.debug_pub = self.create_publisher(Image, '/red_detection', 10)
        
        self.get_logger().info('Red detector started - detecting pure red objects')
        
    def image_callback(self, msg):
        try:
            # Convert to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Convert to RGB for easier red detection
            rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Simple red detection: R > 150, G < 100, B < 100
            red_mask = (rgb[:,:,0] > 150) & (rgb[:,:,1] < 100) & (rgb[:,:,2] < 100)
            
            # Convert boolean mask to uint8
            mask = red_mask.astype(np.uint8) * 255
            
            # Clean up the mask
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Process detection
            fire_point = Point()
            debug_image = cv_image.copy()
            
            if contours:
                # Find largest red area
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 500:  # Minimum area
                    # Get center
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Draw detection
                        cv2.drawContours(debug_image, [largest_contour], -1, (0, 255, 0), 3)
                        cv2.circle(debug_image, (cx, cy), 15, (0, 0, 255), -1)
                        
                        # Normalize coordinates
                        h, w = cv_image.shape[:2]
                        fire_point.x = float(cx - w/2) / (w/2)
                        fire_point.y = float(cy - h/2) / (h/2)
                        fire_point.z = float(area) / (w * h)
                        
                        cv2.putText(debug_image, f"RED DETECTED: x={fire_point.x:.2f}", 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        
                        self.get_logger().info(f'Red object detected at x={fire_point.x:.2f}, area={fire_point.z:.4f}')
                    else:
                        fire_point.x = -999.0
                        fire_point.y = -999.0
                        fire_point.z = 0.0
                else:
                    fire_point.x = -999.0
                    fire_point.y = -999.0
                    fire_point.z = 0.0
            else:
                fire_point.x = -999.0
                fire_point.y = -999.0
                fire_point.z = 0.0
                cv2.putText(debug_image, "NO RED DETECTED", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # Publish
            self.fire_pub.publish(fire_point)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    detector = RedDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()