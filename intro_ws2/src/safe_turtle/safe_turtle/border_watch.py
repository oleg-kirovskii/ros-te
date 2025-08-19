import rclpy
from rclpy.node import Node
import math

from turtlesim.msg import Pose
from rclpy.action import ActionClient
from turtlesim.action import RotateAbsolute

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
        self.get_logger().info("Border Watch Node has been started.")
        # Create an action client for RotateAbsolute action
        self._action_client = ActionClient(self, RotateAbsolute, '/turtle1/rotate_absolute')
        self.get_logger().info("Action client for RotateAbsolute has been created.")

    def rotate_turtle(self, theta, angle):
        # Sends a goal to rotate the turtle to a specific angle.
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = theta+angle
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Sent goal to rotate turtle to {angle} radians.")

    def time_to_collision(self,msg):
        # Calculates the time to collision with the border of the turtlesim world.
        # The world is a square with corners at (0,0), (10,0), (10,10), and (0,10).
        # The turtle's position is given by (msg.x, msg.y) and its orientation by msg.theta.
        # The turtle's linear velocity is given by msg.linear_velocity.
        # Returns the time to collision with the border.
        # If the turtle is not moving, returns a large number (1000.0).
        PI = math.pi
        X_MAX = 10.0
        Y_MAX = 10.0
        X_MIN = 0.0
        Y_MIN = 0.0
        v_x= msg.linear_velocity * math.cos(msg.theta)
        v_y= msg.linear_velocity * math.sin(msg.theta)
        if v_x > 0:
            t_x = (X_MAX - msg.x) / abs(v_x)
        elif v_x < 0:   
            t_x = (msg.x - X_MIN) / abs(v_x)
        if v_y > 0:
            t_y = (Y_MAX - msg.y) / abs(v_y)
        elif v_y < 0:   
            t_y = (msg.y - Y_MIN) / abs(v_y)
        else:
            t_x=1000.0
            t_y=1000.0
        return min(t_x, t_y)
        
    def listener_callback(self,msg):
        x = msg.x
        y = msg.y
        theta = msg.theta
        ttc = self.time_to_collision(msg)
        # self.get_logger().info("D: %f" % ttc)
        if ttc < 1.0:
            self.get_logger().info("Turtle is close to the border, rotating to avoid collision.")
            # Rotate the turtle to avoid collision with the border.
            self.rotate_turtle(theta, math.pi/4)  # Rotate 45 degrees to the right
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
