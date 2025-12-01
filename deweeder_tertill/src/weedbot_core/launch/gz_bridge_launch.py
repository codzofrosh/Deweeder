from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # If you prefer the parameter_bridge binary directly:
    # this ExecuteProcess will run the bridge (clock + cmd_vel)
    # If you installed ros_gz_bridge package, the below invocation should work.
    cmd = [
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]',
        '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist]'
    ]
    return LaunchDescription([
        ExecuteProcess(cmd=cmd, output='screen'),
    ])

