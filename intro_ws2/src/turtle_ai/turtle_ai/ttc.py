import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from turtlesim.msg import Pose
import math

class TimeToCollisionNode(Node):
    def __init__(self):
        super().__init__('time_to_collision_node')
        self.publisher_ = self.create_publisher(
            Float32,
            'time_to_collision',
            10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        self.x = None
        self.y = None
        self.theta = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.get_logger().info('Time to collision node started')

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.linear_velocity = msg.linear_velocity
        self.angular_velocity = msg.angular_velocity
        time_to_collision = self.calculate_time_to_collision()
        msg = Float32()
        msg.data = time_to_collision
        self.publisher_.publish(msg)

    def calculate_time_to_collision(self):
        if self.linear_velocity == 0.0:
            return float('inf')

        # World boundaries
        x_min = 0.0
        x_max = 11.0
        y_min = 0.0
        y_max = 11.0

        # Calculate distances to walls
        dx_right = x_max - self.x
        dx_left = self.x - x_min
        dy_top = y_max - self.y
        dy_bottom = self.y - y_min

        # Calculate time to collision with each wall
        if self.theta==0.0:
            ttc_x = dx_right/self.linear_velocity
            ttc_y = float('inf')
        elif self.theta == math.pi:
            ttc_x = dx_left/abs(self.linear_velocity)
            ttc_y = float('inf')
        else:
            vx = self.linear_velocity * math.cos(self.theta)
            vy = self.linear_velocity * math.sin(self.theta)
            if vx > 0:
                ttc_x = dx_right / vx
            elif vx < 0:
                ttc_x = dx_left / abs(vx)
            else:
                ttc_x = float('inf')

            if vy > 0:
                ttc_y = dy_top / vy
            elif vy < 0:
                ttc_y = dy_bottom / abs(vy)
            else:
                ttc_y = float('inf')

        # Return the minimum time to collision
        return min(ttc_x, ttc_y)


def main(args=None):
    rclpy.init(args=args)
    node = TimeToCollisionNode()
    
    rclpy.spin(node)
    rclpy.shutdown()

if '__name__' == '__main__':
    main()