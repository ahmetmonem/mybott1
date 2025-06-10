#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import math

class FireFollower(Node):
    def __init__(self):
        super().__init__('fire_follower')
        
        # Parameters - INCREASED SPEEDS
        self.declare_parameter('angular_gain', 2.5)  # Increased for faster turning
        self.declare_parameter('linear_gain', 1.2)   # Increased for faster approach
        self.declare_parameter('max_linear_vel', 0.8)  # Increased max speed
        self.declare_parameter('max_angular_vel', 1.5)  # Increased turning speed
        self.declare_parameter('min_fire_area', 0.0005)
        self.declare_parameter('target_fire_area', 0.02)  # Larger target for faster approach
        self.declare_parameter('search_angular_vel', 0.6)  # Faster search
        self.declare_parameter('stop_distance_area', 0.03)
        self.declare_parameter('full_rotation_time', 10.5)  # Adjusted for faster rotation
        
        # Get parameters
        self.angular_gain = self.get_parameter('angular_gain').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.min_fire_area = self.get_parameter('min_fire_area').value
        self.target_fire_area = self.get_parameter('target_fire_area').value
        self.search_angular_vel = self.get_parameter('search_angular_vel').value
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
        
        # Timer for control loop - FASTER UPDATE RATE
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz instead of 10Hz
        
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
        
        self.get_logger().info('Fast fire follower started - Higher speeds enabled')
        
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
        else:
            # Check if fire was recently lost
            time_since_fire = (self.get_clock().now() - self.last_fire_time).nanoseconds / 1e9
            if time_since_fire > 1.0:  # Reduced timeout for faster response
                self.fire_detected = False
    
    def control_loop(self):
        """Main control loop for following fire"""
        cmd = Twist()
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if self.fire_detected:
            # Fire is detected - track and approach it FAST
            
            # Angular velocity - aggressive turning
            angular_error = -self.fire_x
            cmd.angular.z = self.angular_gain * angular_error
            
            # Limit angular velocity
            cmd.angular.z = max(-self.max_angular_vel, 
                              min(self.max_angular_vel, cmd.angular.z))
            
            # Linear velocity - aggressive approach
            if self.fire_area > self.stop_distance_area:
                # Fire is very close, stop
                cmd.linear.x = 0.9
                self.get_logger().info('Fire reached - stopping')
            else:
                # Move forward aggressively when fire is visible
                if abs(self.fire_x) < 0.5:  # Wider tolerance for movement
                    if self.fire_area < self.target_fire_area:
                        # Far from fire - move fast
                        cmd.linear.x = self.max_linear_vel * 0.8  # 80% of max speed
                    else:
                        # Getting close - slow down proportionally
                        area_error = self.target_fire_area - self.fire_area
                        cmd.linear.x = self.linear_gain * area_error
                        cmd.linear.x = max(0.1, min(self.max_linear_vel, cmd.linear.x))
                    
                    # Don't slow down too much when turning
                    if abs(cmd.angular.z) > 0.5:
                        cmd.linear.x *= 0.7  # Only reduce to 70% when turning
                else:
                    # Fire not centered - rotate fast but keep moving slowly
                    cmd.linear.x = 0.1  # Keep moving slowly while turning
            
            self.get_logger().debug(
                f'Fire tracking - X: {self.fire_x:.2f}, '
                f'Area: {self.fire_area:.4f}, '
                f'Linear: {cmd.linear.x:.2f}, '
                f'Angular: {cmd.angular.z:.2f}'
            )
            
        else:
            # No fire detected - search for it by rotating FAST
            search_duration = (current_time - self.search_start_time).nanoseconds / 1e9
            
            if search_duration < self.full_rotation_time:
                # Continue rotating
                cmd.linear.x = 0.0
                cmd.angular.z = self.search_angular_vel * self.search_direction
                
                # Track rotation
                self.total_rotation += abs(cmd.angular.z * dt)
                rotation_degrees = math.degrees(self.total_rotation)
                
                if int(search_duration * 10) % 10 == 0:  # Log every second
                    self.get_logger().info(
                        f'Fast search... Rotated {rotation_degrees:.1f}° '
                        f'({search_duration:.1f}s / {self.full_rotation_time}s)'
                    )
            else:
                # Completed full rotation
                self.search_start_time = current_time
                self.search_direction *= -1
                self.total_rotation = 0.0
                self.get_logger().info('Completed 360° rotation, reversing direction')
                
                cmd.linear.x = 0.0
                cmd.angular.z = self.search_angular_vel * self.search_direction
        
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