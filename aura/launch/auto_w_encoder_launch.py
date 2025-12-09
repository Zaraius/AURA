from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aura',
            namespace='teleop',
            executable='teleop',
            name='teleop'
        ),
        Node(
            package='aura',
            namespace='controller',
            executable='controller',
            name='controller'
        ),
        Node(
            package='joy',
            namespace='joy_node',
            executable='joy_node',
            name='joy'
        ),
        
    ])
