import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from turtlesim.msg import Pose
import random
import math

class TurtleMover(Node):
    def __init__(self):
        super().__init__('turtle_mover')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        self.subscription = self.create_subscription(
            Float32,
            'time_to_collision',
            self.ttc_callback,
            10)
        self.time_to_collision = float('inf')
        self.linear_speed = random.uniform(1.0, 2.0)
        self.angular_speed = 0.0
        self.turning = False
        self.initial_direction = 0
        self.current_pose = Pose()
        self.get_logger().info('Turtle mover node started')
        self.move_turtle()

    def pose_callback(self, msg):
        self.current_pose = msg

    def ttc_callback(self, msg):
        self.time_to_collision = msg.data
        self.get_logger().info(f'Time to collision: {self.time_to_collision}')
        if self.time_to_collision < 2.5 and not self.turning:
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
        self.initial_direction = 1 if random.random() < 0.5 else -1
        self.angular_speed = self.initial_direction * 2.0  # Set a fixed turning speed
        self.reduce_linear_speed = self.linear_speed / 2
        self.adjust_course()

    def adjust_course(self):
        twist = Twist()
        twist.linear.x = self.reduce_linear_speed
        twist.angular.z = self.angular_speed
        self.publisher_.publish(twist)
        self.get_logger().info(f'Adjusting course with linear speed: {self.reduce_linear_speed}, angular speed: {self.angular_speed}')
        self.timer = self.create_timer(0.5, self.resume_movement)

    def resume_movement(self):
        self.turning = False
        twist = Twist()
        self.linear_speed = random.uniform(1.0, 2.0)
        self.angular_speed = 0.0
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.publisher_.publish(twist)
        self.get_logger().info('Resuming movement')
        self.get_logger().info(f'New linear speed: {self.linear_speed}, new angular speed: {self.angular_speed}')


def main(args=None):
    rclpy.init(args=args)
    node = TurtleMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()