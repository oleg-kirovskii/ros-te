import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import random

class TurtleMover(Node):
    def __init__(self):
        super().__init__('turtle_mover')
        # Create publisher for cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Subscribe to time_to_collision topic
        self.subscription = self.create_subscription(
            Float32,
            'time_to_collision',
            self.ttc_callback,
            10)
        
        # Initialize state variables
        self.time_to_collision = float('inf')
        self.base_linear_speed = random.uniform(0.5, 1.5)
        self.current_linear_speed = self.base_linear_speed
        self.angular_speed = 0.0
        self.turning = False
        
        # Start moving
        self.get_logger().info('Turtle mover node started')
        self.get_logger().info(f'Initial linear speed: {self.base_linear_speed}')
        self.move_timer = self.create_timer(0.1, self.move_turtle)

    def ttc_callback(self, msg):
        self.time_to_collision = msg.data
        self.update_movement()

    def update_movement(self):
        # Start turning if too close to wall
        if self.time_to_collision < 2.5 and not self.turning:
            self.turning = True
            self.current_linear_speed = self.base_linear_speed / 2.0
            self.angular_speed = 1.5
            self.get_logger().info('Starting turn maneuver')
            
        # Resume straight movement if safe distance achieved
        elif self.time_to_collision > 3.0 and self.turning:
            self.turning = False
            self.current_linear_speed = self.base_linear_speed
            self.angular_speed = 0.0
            self.get_logger().info('Resuming straight movement')

    def move_turtle(self):
        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = self.current_linear_speed
        twist.angular.z = self.angular_speed
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()