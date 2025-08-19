import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose

class BorderWatch(Node):
    def __init__(self):
        super().__init__('border_watch')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self,msg):
        self.get_logger().info("I heard the pose!")

def main(args=None):
    rclpy.init(args=args)

    border_watch = BorderWatch()

    rclpy.spin(border_watch)

    # border_watch.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
