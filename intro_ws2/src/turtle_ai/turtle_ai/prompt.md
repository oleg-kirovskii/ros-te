**CONTEXT**: #file:turtle_ai is a ROS2 package that ensures that works together with #file:turtlesim package.
A node from turtlesim package called #file:turtlesim_node subscribes to cmd_vel topic of the type Twist which is contained within geometry_msgs package. The node #file:ttc.py of the turtle_ai package publishes the topic /time_to_collision of the type Float32 as described in std_msgs packages. This topic provides the time remaining before collision between the turtle and the nearest wall.

**TASK**: Write #file:turtle_mover.py node for the control of the turtle. The node subscribes to time_to_collision topic and publishes cmd_vel topic.

**ALGORITHM**: The turtle starts with a random linear velocity (between 0,5 and 1,5) and zero angular velocity. When time to collision is below 2,5 sec, the angular velocity is set to 1,5, and linear velocity is reduced by the factor of two. This condition remains until time to collision exceeds 3. After that, the angular velocity is set to zero and the linear velocity is restored.
