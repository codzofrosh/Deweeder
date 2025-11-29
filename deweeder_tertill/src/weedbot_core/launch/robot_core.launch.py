# src/weedbot_core/launch/robot_core.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    use_behavior = LaunchConfiguration('use_behavior').perform(context)
    hw_mode = LaunchConfiguration('hw_mode').perform(context)
    hw_backend = LaunchConfiguration('hw_backend').perform(context)

    nodes = []
    # robot_state_node
    nodes.append(Node(package='weedbot_core', executable='robot_state_node', name='robot_state_node', output='screen'))
    # safety_node
    nodes.append(Node(package='weedbot_core', executable='safety_node', name='safety_node', output='screen'))
    # behavior_node (optional)
    if use_behavior in [True, 'True', 'true', '1', 1]:
        nodes.append(Node(package='weedbot_core', executable='behavior_node', name='behavior_node', output='screen'))
    # HAL and motor controller
    nodes.append(Node(package='weedbot_core', executable='hal_hw_gpio', name='hal_hw_gpio', output='screen',
                      parameters=[{"hw_mode": hw_mode, "hw_backend": hw_backend}]))
    nodes.append(Node(package='weedbot_core', executable='motor_controller', name='motor_controller', output='screen',
                      parameters=[{"hw_mode": hw_mode, "hw_backend": hw_backend}]))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_behavior', default_value='True', description='Enable behavior_node'),
        DeclareLaunchArgument('hw_mode', default_value='False', description='Hardware mode (True/False)'),
        DeclareLaunchArgument('hw_backend', default_value='mock', description='Hardware backend (mock/serial/socketcan)'),
        OpaqueFunction(function=launch_setup),
    ])

