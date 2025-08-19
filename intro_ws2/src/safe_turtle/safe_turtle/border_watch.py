import rclpy
from rclpy.node import Node
import math

from turtlesim.msg import Pose
from std_msgs.msg import Float32
from rclpy.action import ActionClient
from turtlesim.action import RotateAbsolute

class BorderWatch(Node):

    heading = 0.0  # Initialize heading to 0 radians

    def __init__(self):
        super().__init__('border_watch')
        self.subscription = self.create_subscription(
            Float32,
            'time_to_collision',
            self.listener_ttc_callback,
            10
        )
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_pose_callback,
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
        # self.get_logger().info("Sent goal to rotate turtle to {angle} radians.")

    def listener_pose_callback(self, msg):
        # Callback function that processes the turtle's pose.
        self.heading = msg.theta    
    
    def listener_ttc_callback(self,msg):
        # Callback function that processes the time to collision message.
        ttc = msg.data
        if ttc < 1.0:
            self.get_logger().info("Turtle is close to the border, rotating to avoid collision.")
            # Rotate the turtle to avoid collision with the border.
            self.rotate_turtle(self.heading+math.pi/4)  # Rotate 45 degrees to the right
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
