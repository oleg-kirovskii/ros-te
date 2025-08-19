import rclpy
from rclpy.node import Node
import math

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

    def time_to_collision(self,msg):
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
        self.get_logger().info("D: %f" % ttc)

def main(args=None):
    rclpy.init(args=args)

    border_watch = BorderWatch()

    rclpy.spin(border_watch)

    border_watch.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
