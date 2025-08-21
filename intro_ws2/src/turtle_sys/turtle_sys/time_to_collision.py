#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32

class TimeToCollision(Node):
    def __init__(self):
        super().__init__('time_to_collision')
        # Wall boundaries as per L1
        self.MIN_X = 1.0
        self.MAX_X = 10.0
        self.MIN_Y = 1.0
        self.MAX_Y = 10.0
        
        # Subscribe to pose topic
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        
        # Publisher for time to collision
        self.ttc_pub = self.create_publisher(Float32, 'ttc', 10)
        
        # Store previous pose to calculate velocity
        self.prev_pose = None
        self.prev_time = None

    def calculate_time_to_wall(self, x, y, vx, vy):
        """Calculate time to collision with nearest wall based on position and velocity"""
        times = []
        
        # Check time to X walls if moving
        if abs(vx) > 0.0001:  # Small threshold to avoid division by near-zero
            time_to_min_x = (self.MIN_X - x) / vx if vx < 0 else float('inf')
            time_to_max_x = (self.MAX_X - x) / vx if vx > 0 else float('inf')
            if time_to_min_x > 0: times.append(time_to_min_x)
            if time_to_max_x > 0: times.append(time_to_max_x)
            
        # Check time to Y walls if moving
        if abs(vy) > 0.0001:  # Small threshold to avoid division by near-zero
            time_to_min_y = (self.MIN_Y - y) / vy if vy < 0 else float('inf')
            time_to_max_y = (self.MAX_Y - y) / vy if vy > 0 else float('inf')
            if time_to_min_y > 0: times.append(time_to_min_y)
            if time_to_max_y > 0: times.append(time_to_max_y)
        
        # Return minimum positive time or infinity if no collision
        return min(times) if times else float('inf')

    def pose_callback(self, msg):
        current_time = self.get_clock().now()
        
        if self.prev_pose is not None and self.prev_time is not None:
            # Calculate time difference
            dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds
            
            # Calculate absolute velocities (R1)
            vax = (msg.x - self.prev_pose.x) / dt
            vay = (msg.y - self.prev_pose.y) / dt
            
            # Calculate time to collision with nearest wall (R2)
            ttc = self.calculate_time_to_wall(msg.x, msg.y, vax, vay)
            
            # Publish time to collision (R3)
            ttc_msg = Float32()
            ttc_msg.data = ttc
            self.ttc_pub.publish(ttc_msg)
        
        # Store current pose for next velocity calculation
        self.prev_pose = msg
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = TimeToCollision()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
