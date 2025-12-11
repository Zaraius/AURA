from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=['/home/aura/ros2_ws/install/aura/share/aura/config/nav2_params.yaml']
        ),
        # Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=['/home/aura/ros2_ws/install/aura/share/aura/config/nav2_params.yaml']
        ),
        # Behavior tree navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=['/home/aura/ros2_ws/install/aura/share/aura/config/nav2_params.yaml']
        ),
        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['planner_server', 'controller_server', 'bt_navigator']
            }]
        ),
        # Static TF from odom → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['0','0','0','0','0','0','odom','base_link']
        )
    ])
