#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class FireDetector(Node):
    def __init__(self):
        super().__init__('fire_detector')
        
        # Parameters for fire detection
        self.declare_parameter('tuning_mode', False)
        self.declare_parameter('x_min', 0)
        self.declare_parameter('x_max', 100)
        self.declare_parameter('y_min', 0)
        self.declare_parameter('y_max', 100)
        self.declare_parameter('h_min', 0)
        self.declare_parameter('h_max', 25)
        self.declare_parameter('s_min', 50)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 100)
        self.declare_parameter('v_max', 255)
        self.declare_parameter('blur', 5)
        
        self.tuning_mode = self.get_parameter('tuning_mode').value
        
        # Create CV Bridge
        self.bridge = CvBridge()
        
        # Subscribe to image topic
        self.image_sub = self.create_subscription(
            Image,
            'image_in',
            self.image_callback,
            10
        )
        
        # Publishers
        self.fire_pub = self.create_publisher(Point, 'detected_fire', 10)
        
        if self.tuning_mode:
            self.image_pub = self.create_publisher(Image, 'image_tuning', 10)
            # Create trackbars for tuning
            cv2.namedWindow('Fire Detection Tuning')
            cv2.createTrackbar('H Min', 'Fire Detection Tuning', self.get_parameter('h_min').value, 179, self.nothing)
            cv2.createTrackbar('H Max', 'Fire Detection Tuning', self.get_parameter('h_max').value, 179, self.nothing)
            cv2.createTrackbar('S Min', 'Fire Detection Tuning', self.get_parameter('s_min').value, 255, self.nothing)
            cv2.createTrackbar('S Max', 'Fire Detection Tuning', self.get_parameter('s_max').value, 255, self.nothing)
            cv2.createTrackbar('V Min', 'Fire Detection Tuning', self.get_parameter('v_min').value, 255, self.nothing)
            cv2.createTrackbar('V Max', 'Fire Detection Tuning', self.get_parameter('v_max').value, 255, self.nothing)
            cv2.createTrackbar('Blur', 'Fire Detection Tuning', self.get_parameter('blur').value, 20, self.nothing)
            cv2.createTrackbar('X Min', 'Fire Detection Tuning', self.get_parameter('x_min').value, 100, self.nothing)
            cv2.createTrackbar('X Max', 'Fire Detection Tuning', self.get_parameter('x_max').value, 100, self.nothing)
            cv2.createTrackbar('Y Min', 'Fire Detection Tuning', self.get_parameter('y_min').value, 100, self.nothing)
            cv2.createTrackbar('Y Max', 'Fire Detection Tuning', self.get_parameter('y_max').value, 100, self.nothing)
    
    def nothing(self, x):
        pass
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Get dimensions
            h, w, _ = cv_image.shape
            
            # Get parameters
            if self.tuning_mode:
                h_min = cv2.getTrackbarPos('H Min', 'Fire Detection Tuning')
                h_max = cv2.getTrackbarPos('H Max', 'Fire Detection Tuning')
                s_min = cv2.getTrackbarPos('S Min', 'Fire Detection Tuning')
                s_max = cv2.getTrackbarPos('S Max', 'Fire Detection Tuning')
                v_min = cv2.getTrackbarPos('V Min', 'Fire Detection Tuning')
                v_max = cv2.getTrackbarPos('V Max', 'Fire Detection Tuning')
                blur_val = cv2.getTrackbarPos('Blur', 'Fire Detection Tuning')
                x_min_pct = cv2.getTrackbarPos('X Min', 'Fire Detection Tuning')
                x_max_pct = cv2.getTrackbarPos('X Max', 'Fire Detection Tuning')
                y_min_pct = cv2.getTrackbarPos('Y Min', 'Fire Detection Tuning')
                y_max_pct = cv2.getTrackbarPos('Y Max', 'Fire Detection Tuning')
            else:
                h_min = self.get_parameter('h_min').value
                h_max = self.get_parameter('h_max').value
                s_min = self.get_parameter('s_min').value
                s_max = self.get_parameter('s_max').value
                v_min = self.get_parameter('v_min').value
                v_max = self.get_parameter('v_max').value
                blur_val = self.get_parameter('blur').value
                x_min_pct = self.get_parameter('x_min').value
                x_max_pct = self.get_parameter('x_max').value
                y_min_pct = self.get_parameter('y_min').value
                y_max_pct = self.get_parameter('y_max').value
            
            # Calculate ROI boundaries
            x_min = int(w * x_min_pct / 100)
            x_max = int(w * x_max_pct / 100)
            y_min = int(h * y_min_pct / 100)
            y_max = int(h * y_max_pct / 100)
            
            # Apply blur if needed
            if blur_val > 0:
                cv_image = cv2.GaussianBlur(cv_image, (blur_val * 2 + 1, blur_val * 2 + 1), 0)
            
            # Convert to HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Create mask for fire colors (reds, oranges, yellows)
            # Fire typically has low hue values (0-25) for red/orange
            # and high saturation and value
            lower_fire = np.array([h_min, s_min, v_min])
            upper_fire = np.array([h_max, s_max, v_max])
            
            # Create mask
            mask = cv2.inRange(hsv, lower_fire, upper_fire)
            
            # Apply ROI mask
            roi_mask = np.zeros(mask.shape, dtype=np.uint8)
            roi_mask[y_min:y_max, x_min:x_max] = 255
            mask = cv2.bitwise_and(mask, roi_mask)
            
            # Apply morphological operations to remove noise
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            fire_detected = False
            fire_center = Point()
            
            if contours:
                # Find the largest contour (assumed to be fire)
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                # Only consider it as fire if the area is significant
                if area > 500:  # Minimum area threshold
                    # Calculate moments to find center
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Normalize coordinates (-1 to 1)
                        fire_center.x = float(cx - w/2) / (w/2)
                        fire_center.y = float(cy - h/2) / (h/2)
                        fire_center.z = float(area) / (w * h)  # Normalized area
                        
                        fire_detected = True
                        
                        # Draw detection for visualization
                        if self.tuning_mode:
                            cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 2)
                            cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), -1)
            
            # Publish fire detection
            if fire_detected:
                self.fire_pub.publish(fire_center)
            else:
                # Publish empty point if no fire detected
                fire_center.x = -999.0
                fire_center.y = -999.0
                fire_center.z = 0.0
                self.fire_pub.publish(fire_center)
            
            # Publish tuning image if in tuning mode
            if self.tuning_mode:
                # Draw ROI rectangle
                cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)
                
                # Create visualization image
                mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                combined = np.hstack([cv_image, mask_colored])
                
                # Publish the visualization
                tuning_msg = self.bridge.cv2_to_imgmsg(combined, 'bgr8')
                self.image_pub.publish(tuning_msg)
                
                # Also show in window if tuning
                cv2.imshow('Fire Detection Tuning', combined)
                cv2.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f'Error in fire detection: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    fire_detector = FireDetector()
    rclpy.spin(fire_detector)
    fire_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()