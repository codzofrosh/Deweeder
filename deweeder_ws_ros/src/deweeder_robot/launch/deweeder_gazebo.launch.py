import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    # Get package share directory
    pkg_deweeder_robot = get_package_share_directory('deweeder_robot')
    
    # World file path
    world_file = os.path.join(pkg_deweeder_robot, 'worlds', 'crop_field.world')
    
    # URDF file path
    urdf_file = os.path.join(pkg_deweeder_robot, 'urdf', 'deweeder_robot.urdf')
    
    # Load URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Start Gazebo with world
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # Delayed spawn command (runs 5 seconds after Gazebo starts)
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', '/world/default/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '1000',
                    '--req', f'sdf_filename: "{urdf_file}"'
                ],
                output='screen'
            )
        ]
    )
    
    # Weed detection node
    weed_detector = Node(
        package='deweeder_robot',
        executable='weed_detector',
        name='weed_detector',
        output='screen'
    )
    
    # Arm controller node
    arm_controller = Node(
        package='deweeder_robot',
        executable='arm_controller',
        name='arm_controller',
        output='screen'
    )

    return LaunchDescription([
        gazebo_process,
        robot_state_publisher,
        joint_state_publisher,
        delayed_spawn,
        weed_detector,
        arm_controller,
    ])
