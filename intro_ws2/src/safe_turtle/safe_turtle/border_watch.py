import rclpy
from rclpy.node import Node
import math

from turtlesim.msg import Pose
from safe_turtle.msg import ttc
from rclpy.action import ActionClient
from turtlesim.action import RotateAbsolute

class BorderWatch(Node):
    def __init__(self):
        super().__init__('border_watch')
        self.subscription = self.create_subscription(
            ttc,
            'time_to_collision',
            self.listener_callback,
            10
        )
        self.subscription
        self.get_logger().info("Border Watch Node has been started.")
        # Create an action client for RotateAbsolute action
        self._action_client = ActionClient(self, RotateAbsolute, '/turtle1/rotate_absolute')
        self.get_logger().info("Action client for RotateAbsolute has been created.")

    def rotate_turtle(self, angle):
        # Sends a goal to rotate the turtle to a specific angle.
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = angle
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info("Sent goal to rotate turtle to {angle} radians.")
        
    def listener_callback(self,msg):
        # Callback function that processes the time to collision message.
        ttc = msg.ttc
        if ttc < 1.0:
            self.get_logger().info("Turtle is close to the border, rotating to avoid collision.")
            # Rotate the turtle to avoid collision with the border.
            self.rotate_turtle(math.pi/2)  # Rotate 90 degrees to the right
        # else:
        #    self.get_logger().info("Turtle is safe from the border.")

def main(args=None):
    rclpy.init(args=args)

    border_watch = BorderWatch()

    rclpy.spin(border_watch)

    border_watch.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
