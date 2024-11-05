from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the Talker node
        Node(
            package='safety_node',
            executable='talker',
            name='talker',
            output='screen',
            parameters=[{'v': 5.0, 'd': 3.0}]  # Default speed and steering angle
        ),
        
        # Launch the Relay node
        Node(
            package='safety_node',
            executable='relay',
            name='relay',
            output='screen'
        )
    ])
