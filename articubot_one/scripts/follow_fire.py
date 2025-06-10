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
        self.declare_parameter('search_angular_vel', 0.5)
        self.declare_parameter('stop_distance_area', 0.025)
        self.declare_parameter('full_rotation_time', 12.6)  # Time for 360 degrees at 0.5 rad/s
        
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
        self.total_rotation = 0.0  # Track total rotation
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Fire follower started - Will do complete 360° searches')
        
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
            if time_since_fire > 1.5:  # Lost fire for more than 1.5 seconds
                self.fire_detected = False
    
    def control_loop(self):
        """Main control loop for following fire"""
        cmd = Twist()
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if self.fire_detected:
            # Fire is detected - track and approach it
            
            # Angular velocity - proportional to x offset
            angular_error = -self.fire_x  # Negative because positive x means turn left
            cmd.angular.z = self.angular_gain * angular_error
            
            # Limit angular velocity
            cmd.angular.z = max(-self.max_angular_vel, 
                              min(self.max_angular_vel, cmd.angular.z))
            
            # Linear velocity - based on fire area
            if self.fire_area > self.stop_distance_area:
                # Fire is very close, stop
                cmd.linear.x = 0.0
                self.get_logger().info('Fire reached - stopping')
            elif self.fire_area > self.min_fire_area:
                # Calculate distance error
                area_error = self.target_fire_area - self.fire_area
                
                # Only move forward if fire is roughly centered
                if abs(self.fire_x) < 0.3:  # Fire is centered
                    cmd.linear.x = self.linear_gain * area_error
                    
                    # Limit linear velocity
                    cmd.linear.x = max(-self.max_linear_vel, 
                                     min(self.max_linear_vel, cmd.linear.x))
                    
                    # Reduce linear velocity if turning sharply
                    if abs(cmd.angular.z) > 0.5:
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
            
        else:
            # No fire detected - search for it by rotating
            search_duration = (current_time - self.search_start_time).nanoseconds / 1e9
            
            # Calculate rotation based on angular velocity and time
            # 360 degrees = 2*pi radians
            # At 0.5 rad/s, full rotation takes 2*pi/0.5 = 12.566 seconds
            
            if search_duration < self.full_rotation_time:
                # Continue rotating
                cmd.linear.x = 0.0
                cmd.angular.z = self.search_angular_vel * self.search_direction
                
                # Track rotation
                self.total_rotation += abs(cmd.angular.z * dt)
                rotation_degrees = math.degrees(self.total_rotation)
                
                if int(search_duration * 10) % 10 == 0:  # Log every second
                    self.get_logger().info(
                        f'Searching... Rotated {rotation_degrees:.1f}° '
                        f'({search_duration:.1f}s / {self.full_rotation_time}s)'
                    )
            else:
                # Completed full rotation, reset and change direction
                self.search_start_time = current_time
                self.search_direction *= -1
                self.total_rotation = 0.0
                self.get_logger().info(
                    f'Completed 360° rotation, changing direction to '
                    f'{"counter-clockwise" if self.search_direction < 0 else "clockwise"}'
                )
                
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