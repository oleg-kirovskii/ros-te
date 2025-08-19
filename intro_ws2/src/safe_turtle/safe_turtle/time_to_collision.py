import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from turtlesim.msg import Pose
import math

class TimeToCollision(Node):
    def __init__(self):
        super().__init__('time_to_collision')
        self.publisher_=self.create_publisher(
            Float32,
            'time_to_collision',
            10
        )
        self.get_logger().info("Time to Collision topic has been initiated.")
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10
        )
        self.subscription
        self.get_logger().info("Subscription to turtle pose has been created.")

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

    def listener_callback(self, msg):
        # Callback function that processes the turtle's pose and publishes the time to collision.
        ttc_value = self.time_to_collision(msg)
        
        ttc_msg = Float32()
        ttc_msg.data = ttc_value
        self.publisher_.publish(ttc_msg)

def main(args=None):
    rclpy.init(args=args)

    time_to_collision_node = TimeToCollision()

    rclpy.spin(time_to_collision_node)

    time_to_collision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
