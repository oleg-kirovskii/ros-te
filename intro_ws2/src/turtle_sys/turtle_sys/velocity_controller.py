#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

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

    def ttc_callback(self, msg):
        # TODO: Implement velocity control based on time to collision
        cmd = Twist()
        # Placeholder values
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
