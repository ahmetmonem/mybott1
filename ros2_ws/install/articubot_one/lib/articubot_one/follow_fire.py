#!/usr/bin/env python3
#!/usr/bin/env python3
#!/usr/bin/env python3
!/usr/bin/env python3
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import math

class FireFollower(Node):
    def __init__(self):
        super().__init__('fire_follower')
        
        # Parameters
        self.declare_parameter('angular_gain', 2.0)
        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('min_fire_area', 0.001)
        self.declare_parameter('target_fire_area', 0.02)
        self.declare_parameter('search_angular_vel', 0.5)
        
        # Get parameters
        self.angular_gain = self.get_parameter('angular_gain').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.min_fire_area = self.get_parameter('min_fire_area').value
        self.target_fire_area = self.get_parameter('target_fire_area').value
        self.search_angular_vel = self.get_parameter('search_angular_vel').value
        
        # Subscribe to fire detection
        self.fire_sub = self.create_subscription(
            Point,
            'detected_fire',
            self.fire_callback,
            10
        )
        
        # Publisher for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_tracker', 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # State variables
        self.fire_detected = False
        self.fire_x = 0.0
        self.fire_y = 0.0
        self.fire_area = 0.0
        self.last_fire_time = self.get_clock().now()
        
    def fire_callback(self, msg):
        """Handle incoming fire detection messages"""
        if msg.x != -999.0:  # Fire detected
            self.fire_detected = True
            self.fire_x = msg.x
            self.fire_y = msg.y
            self.fire_area = msg.z
            self.last_fire_time = self.get_clock().now()
        else:
            # Check if fire was recently lost
            time_since_fire = (self.get_clock().now() - self.last_fire_time).nanoseconds / 1e9
            if time_since_fire > 2.0:  # Lost fire for more than 2 seconds
                self.fire_detected = False
    
    def control_loop(self):
        """Main control loop for following fire"""
        cmd = Twist()
        
        if self.fire_detected:
            # Fire is detected - track it
            
            # Angular velocity - proportional to x offset
            angular_error = -self.fire_x  # Negative because positive x means turn left
            cmd.angular.z = self.angular_gain * angular_error
            
            # Limit angular velocity
            cmd.angular.z = max(-self.max_angular_vel, 
                              min(self.max_angular_vel, cmd.angular.z))
            
            # Linear velocity - based on fire area
            # Move forward if fire is too small (far away)
            # Move backward if fire is too large (too close)
            if self.fire_area > self.min_fire_area:
                area_error = self.target_fire_area - self.fire_area
                cmd.linear.x = self.linear_gain * area_error
                
                # Limit linear velocity
                cmd.linear.x = max(-self.max_linear_vel, 
                                 min(self.max_linear_vel, cmd.linear.x))
                
                # Reduce linear velocity if turning sharply
                if abs(cmd.angular.z) > 0.5:
                    cmd.linear.x *= 0.5
            else:
                # Fire too small, just rotate to center it
                cmd.linear.x = 0.0
            
            self.get_logger().debug(
                f'Fire tracking - X: {self.fire_x:.2f}, '
                f'Area: {self.fire_area:.4f}, '
                f'Linear: {cmd.linear.x:.2f}, '
                f'Angular: {cmd.angular.z:.2f}'
            )
        else:
            # No fire detected - search for it
            cmd.linear.x = 0.0
            cmd.angular.z = self.search_angular_vel
            self.get_logger().debug('Searching for fire...')
        
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