#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState
import math

class WaterPumpController(Node):
    def __init__(self):
        super().__init__('water_pump_controller')
        
        # Publishers
        self.pump_position_publisher = self.create_publisher(
            Float64MultiArray,
            '/water_pump_cont/commands',
            10
        )
        
        # Subscribers
        self.spray_command_subscriber = self.create_subscription(
            Bool,
            '/water_spray_command',
            self.spray_command_callback,
            10
        )
        
        self.joint_states_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        # Parameters
        self.current_pump_angle = 0.0
        self.target_pump_angle = 0.0
        self.is_spraying = False
        self.pump_speed = 1.0  # radians per second
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Water spray simulation timer
        self.spray_timer = self.create_timer(0.5, self.spray_effect)
        
        self.get_logger().info('Water Pump Controller Node Started')
    
    def spray_command_callback(self, msg):
        """Receive spray commands from fire detector"""
        self.is_spraying = msg.data
        if self.is_spraying:
            self.get_logger().info('Water spray activated')
        else:
            self.get_logger().info('Water spray deactivated')
    
    def joint_states_callback(self, msg):
        """Get current pump angle from joint states"""
        try:
            if 'water_pump_joint' in msg.name:
                index = msg.name.index('water_pump_joint')
                self.current_pump_angle = msg.position[index]
        except (ValueError, IndexError):
            pass
    
    def set_pump_angle(self, angle):
        """Set target angle for water pump"""
        # Clamp angle to valid range
        self.target_pump_angle = max(-math.pi, min(math.pi, angle))
    
    def control_loop(self):
        """Main control loop for pump positioning"""
        # Calculate angle difference
        angle_diff = self.target_pump_angle - self.current_pump_angle
        
        # Apply proportional control
        if abs(angle_diff) > 0.01:  # Dead zone
            # Limit the rate of change
            max_change = self.pump_speed * 0.1  # 0.1 second timestep
            if abs(angle_diff) > max_change:
                if angle_diff > 0:
                    new_angle = self.current_pump_angle + max_change
                else:
                    new_angle = self.current_pump_angle - max_change
            else:
                new_angle = self.target_pump_angle
            
            # Publish pump angle command
            pump_msg = Float64MultiArray()
            pump_msg.data = [new_angle]
            self.pump_position_publisher.publish(pump_msg)
    
    def spray_effect(self):
        """Simulate water spray effects"""
        if self.is_spraying:
            # In a real robot, this would control actual water pump
            # For simulation, we can log the spray action
            self.get_logger().info(f'Spraying water at angle: {math.degrees(self.current_pump_angle):.1f} degrees')
            
            # You could add visual effects here for Gazebo simulation
            # such as particle effects or marker publications
    
    def aim_at_target(self, target_x, target_y, robot_x=0, robot_y=0):
        """Calculate pump angle to aim at a target position"""
        # Calculate angle to target
        dx = target_x - robot_x
        dy = target_y - robot_y
        target_angle = math.atan2(dy, dx)
        
        # Set pump angle
        self.set_pump_angle(target_angle)
        
        return target_angle

def main(args=None):
    rclpy.init(args=args)
    
    water_pump_controller = WaterPumpController()
    
    try:
        rclpy.spin(water_pump_controller)
    except KeyboardInterrupt:
        pass
    
    water_pump_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()