from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('weedbot_simulation_tertill')
    urdf_file = os.path.join(pkg_path, 'urdf', 'weedbot.urdf')

    return LaunchDescription([

        # Start Gazebo Harmonic
        ExecuteProcess(
            cmd=['gz', 'sim', '-r'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_file).read()
            }]
        ),

        # Spawn entity
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'weedbot',
                '-topic', 'robot_description'
            ],
            output='screen'
        )
    ])
