from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Launch turtlesim_node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        
        # 2. Launch time_to_collision node
        Node(
            package='turtle_sys',
            executable='ttc',
            name='time_to_collision'
        ),
        
        # 3. Launch velocity_controller node
        Node(
            package='turtle_sys',
            executable='speed',
            name='velocity_controller'
        )
    ])