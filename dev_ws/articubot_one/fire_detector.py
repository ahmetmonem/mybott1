#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np
import math

class FireDetector(Node):
    def __init__(self):
        super().__init__('fire_detector')
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.water_pump_publisher = self.create_publisher(Float64MultiArray, '/water_pump_cont/commands', 10)
        
        # Subscribers
        self.camera_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Fire detection parameters (HSV color space)
        self.fire_lower_hsv = np.array([0, 50, 50])    # Lower bound for fire colors
        self.fire_upper_hsv = np.array([35, 255, 255]) # Upper bound for fire colors
        
        # Alternative range for orange/red fire
        self.fire_lower_hsv2 = np.array([160, 50, 50])
        self.fire_upper_hsv2 = np.array([180, 255, 255])
        
        # Control parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.min_fire_area = 1000  # Minimum area to consider as fire
        self.target_distance = 100  # Target distance from fire (pixels)
        
        # Water pump parameters
        self.pump_angle = 0.0
        self.is_spraying = False
        self.spray_distance_threshold = 2000  # Fire area threshold to start spraying
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # State variables
        self.fire_detected = False
        self.fire_center_x = 0
        self.fire_center_y = 0
        self.fire_area = 0
        self.image_width = 640
        self.image_height = 480
        
        self.get_logger().info('Fire Detection Node Started')
    
    def camera_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_height, self.image_width = cv_image.shape[:2]
            
            # Detect fire in the image
            self.detect_fire(cv_image)
            
        except Exception as e:
            self.get_logger().error(f'Error in camera callback: {str(e)}')
    
    def detect_fire(self, image):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create masks for fire colors
        mask1 = cv2.inRange(hsv, self.fire_lower_hsv, self.fire_upper_hsv)
        mask2 = cv2.inRange(hsv, self.fire_lower_hsv2, self.fire_upper_hsv2)
        
        # Combine masks
        fire_mask = cv2.bitwise_or(mask1, mask2)
        
        # Apply morphological operations to reduce noise
        kernel = np.ones((5,5), np.uint8)
        fire_mask = cv2.morphologyEx(fire_mask, cv2.MORPH_OPEN, kernel)
        fire_mask = cv2.morphologyEx(fire_mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(fire_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find the largest fire contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > self.min_fire_area:
                # Calculate centroid
                M = cv2.moments(largest_contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    self.fire_detected = True
                    self.fire_center_x = cx
                    self.fire_center_y = cy
                    self.fire_area = area
                    
                    self.get_logger().info(f'Fire detected at ({cx}, {cy}) with area {area}')
                else:
                    self.fire_detected = False
            else:
                self.fire_detected = False
        else:
            self.fire_detected = False
    
    def control_loop(self):
        twist = Twist()
        
        if self.fire_detected:
            # Calculate error from center of image
            error_x = self.fire_center_x - (self.image_width / 2)
            error_y = self.fire_center_y - (self.image_height / 2)
            
            # Check if fire is close enough to start spraying
            if self.fire_area > self.spray_distance_threshold:
                # Stop moving and start spraying
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
                # Aim water pump towards fire
                self.aim_water_pump()
                self.spray_water(True)
                
                self.get_logger().info('Spraying water at fire!')
                
            else:
                # Move towards fire
                # Angular control to center the fire
                if abs(error_x) > 50:  # Dead zone
                    twist.angular.z = -self.angular_speed * (error_x / (self.image_width / 2))
                
                # Linear control - move forward if fire is centered
                if abs(error_x) < 100:
                    twist.linear.x = self.linear_speed
                
                self.spray_water(False)
                
        else:
            # Search for fire - rotate slowly
            twist.linear.x = 0.0
            twist.angular.z = 0.3
            self.spray_water(False)
            
            self.get_logger().info('Searching for fire...')
        
        # Publish velocity commands
        self.cmd_vel_publisher.publish(twist)
    
    def aim_water_pump(self):
        # Calculate pump angle based on fire position
        error_x = self.fire_center_x - (self.image_width / 2)
        # Convert pixel error to angle (approximate)
        angle_error = (error_x / (self.image_width / 2)) * 0.5  # Max 0.5 radians
        
        self.pump_angle = angle_error
        
        # Publish pump angle
        pump_msg = Float64MultiArray()
        pump_msg.data = [self.pump_angle]
        self.water_pump_publisher.publish(pump_msg)
    
    def spray_water(self, spray):
        if spray != self.is_spraying:
            self.is_spraying = spray
            if spray:
                self.get_logger().info('Starting water spray')
                # In a real robot, this would activate the water pump
                # For simulation, we just log the action
            else:
                self.get_logger().info('Stopping water spray')

def main(args=None):
    rclpy.init(args=args)
    
    fire_detector = FireDetector()
    
    try:
        rclpy.spin(fire_detector)
    except KeyboardInterrupt:
        pass
    
    fire_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()