from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='turtle_ai',
            executable='ttc',
            name='time_to_collision'
        ),
        Node(
            package='turtle_ai',
            executable='tmove',
            name='turtle_mover',
            output='screen'
        )])