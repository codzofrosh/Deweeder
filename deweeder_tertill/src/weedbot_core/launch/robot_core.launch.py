from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    use_behavior = LaunchConfiguration('use_behavior').perform(context)
    nodes = []
    # robot_state_node
    nodes.append(Node(package='weedbot_core', executable='robot_state_node', name='robot_state_node'))
    # safety_node
    nodes.append(Node(package='weedbot_core', executable='safety_node', name='safety_node'))
    # behavior_node (optional)
    if use_behavior in [True, 'True', 'true', '1', 1]:
        nodes.append(Node(package='weedbot_core', executable='behavior_node', name='behavior_node'))
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_behavior', default_value='True', description='Enable behavior_node'),
        OpaqueFunction(function=launch_setup),
    ])
