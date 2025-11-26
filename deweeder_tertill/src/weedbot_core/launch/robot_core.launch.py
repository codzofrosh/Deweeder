# robot_core.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    hw_mode_arg = DeclareLaunchArgument("hw_mode", default_value="False", description="Hardware mode (True/False)")
    hw_backend_arg = DeclareLaunchArgument("hw_backend", default_value="mock", description="Hardware backend (mock/serial/socketcan)")

    hw_mode = LaunchConfiguration("hw_mode")
    hw_backend = LaunchConfiguration("hw_backend")

    nodes = []

    nodes.append(Node(
        package="weedbot_core",
        executable="robot_state_node",
        name="robot_state_node",
        output="screen",
    ))

    nodes.append(Node(
        package="weedbot_core",
        executable="behavior_node",
        name="behavior_node",
        output="screen",
    ))

    nodes.append(Node(
        package="weedbot_core",
        executable="safety_node",
        name="safety_node",
        output="screen",
    ))

    nodes.append(Node(
        package="weedbot_core",
        executable="hal_hw_gpio",
        name="hal_hw_gpio",
        output="screen",
        parameters=[{"hw_mode": hw_mode, "hw_backend": hw_backend}],
    ))

    nodes.append(Node(
        package="weedbot_core",
        executable="motor_controller",
        name="motor_controller",
        output="screen",
        parameters=[{"hw_mode": hw_mode, "hw_backend": hw_backend}],
    ))

    return LaunchDescription([
        hw_mode_arg,
        hw_backend_arg,
        *nodes
    ])
