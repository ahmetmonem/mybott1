#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class FireDebugger(Node):
    def __init__(self):
        super().__init__('fire_debugger')
        
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for debug visualization
        self.debug_pub = self.create_publisher(Image, '/fire_debug', 10)
        self.fire_pub = self.create_publisher(Point, '/detected_fire', 10)
        
        self.get_logger().info('Fire debugger started - showing what camera sees')
        
        # Create windows
        cv2.namedWindow('Camera View', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Fire Detection', cv2.WINDOW_NORMAL)
        
        # Create trackbars for HSV tuning
        cv2.createTrackbar('H Min', 'Fire Detection', 0, 179, self.nothing)
        cv2.createTrackbar('H Max', 'Fire Detection', 10, 179, self.nothing)
        cv2.createTrackbar('S Min', 'Fire Detection', 200, 255, self.nothing)
        cv2.createTrackbar('S Max', 'Fire Detection', 255, 255, self.nothing)
        cv2.createTrackbar('V Min', 'Fire Detection', 100, 255, self.nothing)
        cv2.createTrackbar('V Max', 'Fire Detection', 255, 255, self.nothing)
        
    def nothing(self, x):
        pass
        
    def image_callback(self, msg):
        try:
            # Convert to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Show original image
            cv2.imshow('Camera View', cv_image)
            
            # Get trackbar values
            h_min = cv2.getTrackbarPos('H Min', 'Fire Detection')
            h_max = cv2.getTrackbarPos('H Max', 'Fire Detection')
            s_min = cv2.getTrackbarPos('S Min', 'Fire Detection')
            s_max = cv2.getTrackbarPos('S Max', 'Fire Detection')
            v_min = cv2.getTrackbarPos('V Min', 'Fire Detection')
            v_max = cv2.getTrackbarPos('V Max', 'Fire Detection')
            
            # Convert to HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Create mask for red colors
            # Red wraps around in HSV, so we need two ranges
            if h_max < h_min:
                # Red color wraps around
                lower1 = np.array([0, s_min, v_min])
                upper1 = np.array([h_max, s_max, v_max])
                lower2 = np.array([h_min, s_min, v_min])
                upper2 = np.array([179, s_max, v_max])
                mask1 = cv2.inRange(hsv, lower1, upper1)
                mask2 = cv2.inRange(hsv, lower2, upper2)
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                lower = np.array([h_min, s_min, v_min])
                upper = np.array([h_max, s_max, v_max])
                mask = cv2.inRange(hsv, lower, upper)
            
            # Apply morphological operations
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Draw detection
            debug_image = cv_image.copy()
            fire_point = Point()
            fire_detected = False
            
            if contours:
                # Find largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 100:  # Minimum area threshold
                    # Get center
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Draw on debug image
                        cv2.drawContours(debug_image, [largest_contour], -1, (0, 255, 0), 2)
                        cv2.circle(debug_image, (cx, cy), 10, (0, 0, 255), -1)
                        
                        # Calculate normalized coordinates
                        h, w = cv_image.shape[:2]
                        fire_point.x = float(cx - w/2) / (w/2)
                        fire_point.y = float(cy - h/2) / (h/2)
                        fire_point.z = float(area) / (w * h)
                        fire_detected = True
                        
                        # Add text
                        cv2.putText(debug_image, f"Fire: ({fire_point.x:.2f}, {fire_point.y:.2f}) Area: {fire_point.z:.4f}", 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            if not fire_detected:
                fire_point.x = -999.0
                fire_point.y = -999.0
                fire_point.z = 0.0
                cv2.putText(debug_image, "No Fire Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Publish fire detection
            self.fire_pub.publish(fire_point)
            
            # Show mask and debug image
            mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            combined = np.hstack([debug_image, mask_colored])
            cv2.imshow('Fire Detection', combined)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(combined, 'bgr8')
            self.debug_pub.publish(debug_msg)
            
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    debugger = FireDebugger()
    
    try:
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        debugger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()