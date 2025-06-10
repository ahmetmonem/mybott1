#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import math
import random

class FireFollower(Node):
    def __init__(self):
        super().__init__('fire_follower')
        
        # Parameters - INCREASED SPEEDS
        self.declare_parameter('angular_gain', 2.5)
        self.declare_parameter('linear_gain', 1.2)
        self.declare_parameter('max_linear_vel', 0.8)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('min_fire_area', 0.0005)
        self.declare_parameter('target_fire_area', 0.02)
        self.declare_parameter('search_angular_vel', 0.6)
        self.declare_parameter('search_linear_vel', 0.3)  # Forward speed while searching
        self.declare_parameter('stop_distance_area', 0.1)
        self.declare_parameter('full_rotation_time', 10.5)
        
        # Get parameters
        self.angular_gain = self.get_parameter('angular_gain').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.min_fire_area = self.get_parameter('min_fire_area').value
        self.target_fire_area = self.get_parameter('target_fire_area').value
        self.search_angular_vel = self.get_parameter('search_angular_vel').value
        self.search_linear_vel = self.get_parameter('search_linear_vel').value
        self.stop_distance_area = self.get_parameter('stop_distance_area').value
        self.full_rotation_time = self.get_parameter('full_rotation_time').value
        
        # Subscribe to fire detection
        self.fire_sub = self.create_subscription(
            Point,
            'detected_fire',
            self.fire_callback,
            10
        )
        
        # Publisher for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # State variables
        self.fire_detected = False
        self.fire_x = 0.0
        self.fire_y = 0.0
        self.fire_area = 0.0
        self.last_fire_time = self.get_clock().now()
        
        # Search state
        self.search_start_time = self.get_clock().now()
        self.search_direction = 1
        self.total_rotation = 0.0
        self.last_time = self.get_clock().now()
        
        # Exploration mode variables
        self.exploration_mode = 'spiral'  # 'spiral', 'random_walk', or '360_scan'
        self.mode_start_time = self.get_clock().now()
        self.spiral_radius = 0.5
        self.random_turn_time = 0
        self.random_turn_duration = 0
        self.scanning_in_place = False
        self.scan_start_time = None
        
        self.get_logger().info('Fire follower with exploration started')
        
    def fire_callback(self, msg):
        """Handle incoming fire detection messages"""
        if msg.x != -999.0:  # Fire detected
            self.fire_detected = True
            self.fire_x = msg.x
            self.fire_y = msg.y
            self.fire_area = msg.z
            self.last_fire_time = self.get_clock().now()
            # Reset search when fire is found
            self.search_start_time = self.get_clock().now()
            self.total_rotation = 0.0
            self.scanning_in_place = False
        else:
            # Check if fire was recently lost
            time_since_fire = (self.get_clock().now() - self.last_fire_time).nanoseconds / 1e9
            if time_since_fire > 1.0:
                self.fire_detected = False
    
    def get_exploration_command(self, current_time):
        """Get movement command for exploration/search mode"""
        cmd = Twist()
        
        # Switch between exploration modes periodically
        mode_duration = (current_time - self.mode_start_time).nanoseconds / 1e9
        
        if self.exploration_mode == 'spiral':
            # Spiral outward search pattern
            if mode_duration < 20:  # Spiral for 20 seconds
                # Increase radius over time
                self.spiral_radius = 0.5 + (mode_duration * 0.05)
                cmd.linear.x = self.search_linear_vel
                cmd.angular.z = self.search_angular_vel / (1 + self.spiral_radius)
            else:
                # Switch to random walk
                self.exploration_mode = 'random_walk'
                self.mode_start_time = current_time
                self.get_logger().info('Switching to random walk exploration')
                
        elif self.exploration_mode == 'random_walk':
            # Random walk pattern
            if mode_duration < 20:  # Random walk for 20 seconds
                # Change direction randomly
                if self.random_turn_time <= 0:
                    self.random_turn_duration = random.uniform(2, 5)
                    self.random_turn_time = self.random_turn_duration
                    self.search_direction = random.choice([-1, 0, 1])
                    
                self.random_turn_time -= 0.05  # Subtract timer period
                
                if self.search_direction == 0:
                    # Move straight
                    cmd.linear.x = self.search_linear_vel * 1.5
                    cmd.angular.z = 0
                else:
                    # Turn while moving
                    cmd.linear.x = self.search_linear_vel
                    cmd.angular.z = self.search_angular_vel * self.search_direction * 0.7
            else:
                # Switch to 360 scan
                self.exploration_mode = '360_scan'
                self.mode_start_time = current_time
                self.scanning_in_place = True
                self.scan_start_time = current_time
                self.get_logger().info('Switching to 360 degree scan')
                
        elif self.exploration_mode == '360_scan':
            # Do a 360 scan in place
            if self.scanning_in_place:
                scan_duration = (current_time - self.scan_start_time).nanoseconds / 1e9
                if scan_duration < self.full_rotation_time:
                    # Just rotate in place
                    cmd.linear.x = 0
                    cmd.angular.z = self.search_angular_vel
                else:
                    # Finished scanning, move forward and switch mode
                    self.scanning_in_place = False
                    self.exploration_mode = 'spiral'
                    self.mode_start_time = current_time
                    self.spiral_radius = 0.5
                    self.get_logger().info('Completed 360 scan, switching to spiral')
            else:
                # Move forward after scan
                cmd.linear.x = self.search_linear_vel * 2  # Move fast after scan
                cmd.angular.z = 0
                
        return cmd
    
    def control_loop(self):
        """Main control loop for following fire"""
        cmd = Twist()
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if self.fire_detected:
            # Reset exploration mode for next search
            self.exploration_mode = 'spiral'
            self.spiral_radius = 0.5
            
            # Fire is detected - track and approach it FAST
            angular_error = -self.fire_x
            cmd.angular.z = self.angular_gain * angular_error
            cmd.angular.z = max(-self.max_angular_vel, 
                              min(self.max_angular_vel, cmd.angular.z))
            
            # Linear velocity - aggressive approach
            if self.fire_area > self.stop_distance_area:
                # Fire is very close, stop
                cmd.linear.x = 0.0
                self.get_logger().info('Fire reached - stopping')
            else:
                # Move forward aggressively when fire is visible
                if abs(self.fire_x) < 0.5:  # Wider tolerance for movement
                    if self.fire_area < self.target_fire_area:
                        # Far from fire - move fast
                        cmd.linear.x = self.max_linear_vel * 0.8
                    else:
                        # Getting close - slow down proportionally
                        area_error = self.target_fire_area - self.fire_area
                        cmd.linear.x = self.linear_gain * area_error
                        cmd.linear.x = max(0.1, min(self.max_linear_vel, cmd.linear.x))
                    
                    # Don't slow down too much when turning
                    if abs(cmd.angular.z) > 0.5:
                        cmd.linear.x *= 0.7
                else:
                    # Fire not centered - rotate fast but keep moving slowly
                    cmd.linear.x = 0.1
            
        else:
            # No fire detected - use exploration mode
            cmd = self.get_exploration_command(current_time)
            
            # Track total rotation for logging
            self.total_rotation += abs(cmd.angular.z * dt)
            
            # Periodic logging
            if int((current_time - self.search_start_time).nanoseconds / 1e9) % 3 == 0:
                self.get_logger().info(
                    f'Exploring in {self.exploration_mode} mode - '
                    f'Linear: {cmd.linear.x:.2f}, Angular: {cmd.angular.z:.2f}'
                )
        
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