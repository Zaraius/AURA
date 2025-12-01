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
            namespace='subscribe',
            executable='subscribe',
            name='subscribe'
        ),
        Node(
            package='aura',
            namespace='state_machine',
            executable='state_machine',
            name='state_machine'
        ),
        Node(
            package='aura',
            namespace='auto',
            executable='auto',
            name='auto'
        ),
        Node(
            package='joy',
            namespace='joy_node',
            executable='joy_node',
            name='joy'
        ),
        
    ])
