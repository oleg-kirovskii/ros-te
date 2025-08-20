import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import random
import math

class TurtleMover(Node):
    def __init__(self):
        super().__init__('turtle_mover')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Float32,
            'time_to_collision',
            self.ttc_callback,
            10)
        self.time_to_collision = float('inf')
        self.linear_speed = random.uniform(1.0, 2.0)
        self.angular_speed = 0.0
        self.turning = False
        self.get_logger().info('Turtle mover node started')
        self.move_turtle()

    def ttc_callback(self, msg):
        self.time_to_collision = msg.data
        self.get_logger().info(f'Time to collision: {self.time_to_collision}')
        if self.time_to_collision < 2.0 and not self.turning:
            self.avoid_collision()

    def move_turtle(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.publisher_.publish(twist)
        self.get_logger().info(f'Moving with linear speed: {self.linear_speed}, angular speed: {self.angular_speed}')
        self.timer = self.create_timer(0.1, self.move_turtle)

    def avoid_collision(self):
        self.turning = True
        self.get_logger().info('Avoiding collision')
        self.linear_speed = self.linear_speed/2
        self.angular_speed = random.uniform(-1.0, 1.0)
        self.get_logger().info(f'New linear speed: {self.linear_speed}, new angular speed: {self.angular_speed}')
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.publisher_.publish(twist)
        self.timer = self.create_timer(1.0, self.resume_movement)

    def resume_movement(self):
        self.turning = False
        self.linear_speed = random.uniform(1.0, 2.0)
        self.angular_speed = 0.0
        self.get_logger().info('Resuming movement')
        self.get_logger().info(f'New linear speed: {self.linear_speed}, new angular speed: {self.angular_speed}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()