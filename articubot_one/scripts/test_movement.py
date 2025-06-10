#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestMovement(Node):
    def __init__(self):
        super().__init__('test_movement')
        
        # Try different topics
        self.publishers = {
            'diff_cont': self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10),
            'cmd_vel': self.create_publisher(Twist, '/cmd_vel', 10),
            'cmd_vel_out': self.create_publisher(Twist, '/cmd_vel_out', 10),
        }
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0
        
        self.get_logger().info('Test movement node started. Robot should rotate.')
        
    def timer_callback(self):
        cmd = Twist()
        
        # Simple rotation
        cmd.angular.z = 0.5  # Rotate at 0.5 rad/s
        cmd.linear.x = 0.0
        
        # Publish to all possible topics
        for topic_name, publisher in self.publishers.items():
            publisher.publish(cmd)
        
        self.counter += 1
        if self.counter % 10 == 0:  # Every second
            self.get_logger().info(f'Publishing cmd_vel: angular.z = {cmd.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    test_movement = TestMovement()
    rclpy.spin(test_movement)
    test_movement.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()