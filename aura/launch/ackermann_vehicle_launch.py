from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='webots_ros2_driver',
            executable='driver',
            name='ackermann_vehicle_driver',
            parameters=[
                {'robot_description': open('ackermann_vehicle.urdf').read()},
            ],
            namespace='/ackermann_vehicle'
        ),
        Node(
            package='your_teleop_pkg',
            executable='ackermann_teleop'
        )
    ])
