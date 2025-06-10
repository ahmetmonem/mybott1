#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestMove(Node):
    def __init__(self):
        super().__init__('test_move')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.callback)
        self.get_logger().info('Test move started - robot should rotate')
        
    def callback(self):
        cmd = Twist()
        cmd.angular.z = 0.5
        self.pub.publish(cmd)
        
def main():
    rclpy.init()
    node = TestMove()
    rclpy.spin(node)
    
if __name__ == '__main__':
    main()
