import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class SpeedCommander(Node):
    def __init__(self):
        super().__init__('speed_commander')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("Speed Commander topic has been initiated.")
        self.subscription = self.create_subscription(
            Float32,
            'time_to_collision',
            self.listener_callback,
            10
        )
        self.subscription
        self.get_logger().info("Subscription to time to collision has been created.")

    def listener_callback(self, msg):
        # Callback function that processes the time to collision and adjusts speed.
        speed = Twist()
        if msg.data > 2.5:
            speed.linear.x = 0.5  # Normal speed
            self.publisher_.publish(speed)

def main(args=None):
    rclpy.init(args=args)

    speed_commander = SpeedCommander()

    rclpy.spin(speed_commander)

    speed_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()