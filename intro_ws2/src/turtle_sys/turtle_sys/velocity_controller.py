#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import random
import math

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        # Subscribe to time to collision topic
        self.ttc_sub = self.create_subscription(
            Float32,
            'ttc',
            self.ttc_callback,
            10)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # State variables
        self.initial_turn = True  # To track initial turn state
        self.turning = False      # To track collision avoidance turning
        self.turn_start_time = None  # To track turning duration
        self.target_angle = random.uniform(0, math.pi/2)  # Random angle between 0 and 90 deg
        self.current_angle = 0.0
        
        # Start the initial turn
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Starting initial turn to {math.degrees(self.target_angle)} degrees')

    def ttc_callback(self, msg):
        current_time = self.get_clock().now()
        
        if not self.initial_turn and not self.turning and msg.data < 2.0:
            self.turning = True
            self.turn_start_time = current_time
            self.get_logger().info('Starting turn maneuver')
        elif self.turning:
            # Only stop turning if both conditions are met:
            # 1. Time to collision is safe (> 3.0)
            # 2. We've been turning for at least 1 second
            turning_duration = (current_time - self.turn_start_time).nanoseconds / 1e9
            if msg.data > 3.0 and turning_duration >= 1.0:
                self.turning = False
                self.get_logger().info('Completed turn maneuver')

    def control_loop(self):
        cmd = Twist()
        
        if self.initial_turn:
            # R1: Complete initial random turn
            cmd.angular.z = 0.5  # Moderate turning speed
            self.current_angle += abs(cmd.angular.z) * 0.1  # Approximate angle change
            
            if self.current_angle >= self.target_angle:
                self.initial_turn = False
                self.get_logger().info('Initial turn completed')
        
        elif self.turning:
            # R3: Turn until safe
            cmd.angular.z = 0.5
            cmd.linear.x = 0.2  # Slow forward movement while turning
            self.get_logger().info('Turning to avoid collision')
        
        else:
            # R2, R4: Move forward with speed 1
            cmd.linear.x = 1.0  # As per L1, speed should not exceed 1
            cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
