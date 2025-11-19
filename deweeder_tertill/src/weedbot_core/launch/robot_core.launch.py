from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='weedbot_core',
            executable='robot_state_node',
            name='robot_state_node',
            output='screen'
        ),
        Node(
            package='weedbot_core',
            executable='behavior_node',
            name='behavior_node',
            output='screen'
        ),
        Node(
            package='weedbot_core',
            executable='safety_node',
            name='safety_node',
            output='screen'
        )
    ])
