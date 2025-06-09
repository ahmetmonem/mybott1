#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import math

class FireFollower(Node):
    def __init__(self):
        super().__init__('fire_follower')
        
        # Parameters
        self.declare_parameter('angular_gain', 1.5)
        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('max_linear_vel', 0.4)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('min_fire_area', 0.001)
        self.declare_parameter('target_fire_area', 0.015)
        self.declare_parameter('search_angular_vel', 0.4)
        self.declare_parameter('stop_distance_area', 0.025)
        
        # Get parameters
        self.angular_gain = self.get_parameter('angular_gain').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.min_fire_area = self.get_parameter('min_fire_area').value
        self.target_fire_area = self.get_parameter('target_fire_area').value
        self.search_angular_vel = self.get_parameter('search_angular_vel').value
        self.stop_distance_area = self.get_parameter('stop_distance_area').value
        
        # Subscribe to fire detection
        self.fire_sub = self.create_subscription(
            Point,
            'detected_fire',
            self.fire_callback,
            10
        )
        
        # Publisher for cmd_vel - publish directly to the controller
        self.cmd_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # State variables
        self.fire_detected = False
        self.fire_x = 0.0
        self.fire_y = 0.0
        self.fire_area = 0.0
        self.last_fire_time = self.get_clock().now()
        
        # Search state
        self.search_start_time = self.get_clock().now()
        self.search_direction = 1  # 1 for clockwise, -1 for counter-clockwise
        
        self.get_logger().info('Fire follower node started - Autonomous mode')
        
    def fire_callback(self, msg):
        """Handle incoming fire detection messages"""
        if msg.x != -999.0:  # Fire detected
            self.fire_detected = True
            self.fire_x = msg.x
            self.fire_y = msg.y
            self.fire_area = msg.z
            self.last_fire_time = self.get_clock().now()
            self.get_logger().debug(f'Fire detected at x:{self.fire_x:.2f}, area:{self.fire_area:.4f}')
        else:
            # Check if fire was recently lost
            time_since_fire = (self.get_clock().now() - self.last_fire_time).nanoseconds / 1e9
            if time_since_fire > 1.0:  # Lost fire for more than 1 second
                self.fire_detected = False
    
    def control_loop(self):
        """Main control loop for following fire"""
        cmd = Twist()
        
        if self.fire_detected:
            # Fire is detected - track and approach it
            
            # Angular velocity - proportional to x offset
            # Make it more responsive to centering the fire
            angular_error = -self.fire_x  # Negative because positive x means turn left
            cmd.angular.z = self.angular_gain * angular_error
            
            # Limit angular velocity
            cmd.angular.z = max(-self.max_angular_vel, 
                              min(self.max_angular_vel, cmd.angular.z))
            
            # Linear velocity - based on fire area
            # Stop if fire is too close (large area)
            if self.fire_area > self.stop_distance_area:
                # Fire is very close, stop
                cmd.linear.x = 0.0
                self.get_logger().info('Fire reached - stopping')
            elif self.fire_area > self.min_fire_area:
                # Calculate distance error
                area_error = self.target_fire_area - self.fire_area
                
                # Only move forward if fire is roughly centered
                if abs(self.fire_x) < 0.3:  # Fire is centered (within 30% of center)
                    cmd.linear.x = self.linear_gain * area_error
                    
                    # Limit linear velocity
                    cmd.linear.x = max(-self.max_linear_vel, 
                                     min(self.max_linear_vel, cmd.linear.x))
                    
                    # Reduce linear velocity if turning
                    if abs(cmd.angular.z) > 0.3:
                        cmd.linear.x *= 0.5
                else:
                    # Fire not centered, just rotate
                    cmd.linear.x = 0.0
            else:
                # Fire too small/far, move forward slowly while centering
                if abs(self.fire_x) < 0.5:
                    cmd.linear.x = 0.2
                else:
                    cmd.linear.x = 0.0
            
            self.get_logger().debug(
                f'Fire tracking - X: {self.fire_x:.2f}, '
                f'Area: {self.fire_area:.4f}, '
                f'Linear: {cmd.linear.x:.2f}, '
                f'Angular: {cmd.angular.z:.2f}'
            )
        else:
            # No fire detected - search for it by rotating
            # Do a 360 degree sweep
            current_time = self.get_clock().now()
            search_duration = (current_time - self.search_start_time).nanoseconds / 1e9
            
            # Complete 360 degree turn in about 9 seconds at 0.4 rad/s
            if search_duration > 9.0:
                # Reset search start time
                self.search_start_time = current_time
                # Alternate search direction
                self.search_direction *= -1
                self.get_logger().info(f'Completed 360° search, changing direction')
            
            cmd.linear.x = 0.0
            cmd.angular.z = self.search_angular_vel * self.search_direction
            self.get_logger().debug(f'Searching for fire... (rotating {"clockwise" if self.search_direction > 0 else "counter-clockwise"})')
        
        # Publish command
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    fire_follower = FireFollower()
    rclpy.spin(fire_follower)
    fire_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()