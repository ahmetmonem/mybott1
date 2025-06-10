#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DirectCmdTest(Node):
    def __init__(self):
        super().__init__('direct_cmd_test')
        
        # Create publishers for ALL possible cmd topics
        self.publishers = {
            'cmd_vel': self.create_publisher(Twist, '/cmd_vel', 10),
            'cmd_vel_out': self.create_publisher(Twist, '/cmd_vel_out', 10),
            'diff_drive/cmd_vel': self.create_publisher(Twist, '/diff_drive_controller/cmd_vel', 10),
            'cmd_vel_unstamped': self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        }
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()
        
        self.get_logger().info('Starting direct command test - robot should rotate')
        
    def timer_callback(self):
        cmd = Twist()
        
        # Rotate for 5 seconds, then move forward for 5 seconds
        elapsed = time.time() - self.start_time
        
        if elapsed < 5.0:
            # Rotate
            cmd.angular.z = 0.5
            cmd.linear.x = 0.0
            action = "ROTATING"
        elif elapsed < 10.0:
            # Move forward
            cmd.angular.z = 0.0
            cmd.linear.x = 0.2
            action = "MOVING FORWARD"
        else:
            # Reset
            self.start_time = time.time()
            action = "RESETTING"
        
        # Publish to ALL topics
        for topic_name, publisher in self.publishers.items():
            publisher.publish(cmd)
        
        self.get_logger().info(f'{action} - Publishing to all cmd topics')

def main(args=None):
    rclpy.init(args=args)
    test = DirectCmdTest()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 