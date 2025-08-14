#!/bin/bash

source /opt/ros/humble/setup.bash
# First, we will start turtle sim
ros2 run turtlesim turtlesim_node &
echo turtle sim started
# Now, we will make the turtle run in a circle by sending twist messages into cmd_vel topic
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" --times 5
echo turtle has made some circles
# Lastly, we will make the turtle turn 90 deg counter-clockwise by involing RotateAbsolute action
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
echo turtle has turned
echo show is over!